from __future__ import annotations

import math
import os
import pprint
import subprocess
import threading
import signal
import time
from pathlib import Path
from typing import Any, Optional, Dict, List

import logging
import uuid

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from tf2_ros import TransformBroadcaster

import rosgraph_msgs.msg as rosgraph_msgs
import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as geometry_msgs
import autoware_system_msgs.msg as autoware_system_msgs
import autoware_control_msgs.msg as autoware_control_msgs
import autoware_adapi_v1_msgs.srv as autoware_adapi_v1_msgs_srv
import autoware_vehicle_msgs.msg as autoware_vehicle_msgs
import sensor_msgs.msg as sensor_msgs
import autoware_perception_msgs.msg as autoware_perception_msgs


from sv.utils.position import Position
from sv.utils.util import get_cfg
from sv.utils.object import ObjectKinematic, ObjectState, RoadObjectType, ShapeType
from sv.registry import register_av
from sv.utils.control import Ctrl, CtrlMode
from sv.utils.sps import ScenarioPack
from sv.utils.publish_manager import PublishManager, PublishMode, TopicPublisher


CLOCK_PUB_HZ = 100.0  # Hz

# Mapping from RoadObjectType to Autoware's ObjectClassification
OBJECT_TYPE_MAP = {
    RoadObjectType.CAR: autoware_perception_msgs.ObjectClassification.CAR,
    RoadObjectType.TRUCK: autoware_perception_msgs.ObjectClassification.TRUCK,
    RoadObjectType.BUS: autoware_perception_msgs.ObjectClassification.BUS,
    RoadObjectType.TRAILER: autoware_perception_msgs.ObjectClassification.TRAILER,
    RoadObjectType.MOTORCYCLE: autoware_perception_msgs.ObjectClassification.MOTORCYCLE,
    RoadObjectType.BICYCLE: autoware_perception_msgs.ObjectClassification.BICYCLE,
    RoadObjectType.PEDESTRIAN: autoware_perception_msgs.ObjectClassification.PEDESTRIAN,
}


logger = logging.getLogger(__name__)


class AutowarePureAV:
    """
    Autoware AV adapter:
    - init(): Launch Autoware (subprocess) + create ROS node, pub/sub, services
    - reset(): Set initial pose + route via AD API services
    - step(): Send obs (ego + agents), wait for new control, convert to Ctrl to return to simulator
    - stop(): Stop Autoware process + ROS node
    - should_quit(): Decide whether to quit based on motion state / error / process status
    """

    def __init__(self, cfg_path: Path):
        cfg = get_cfg(Path(cfg_path))
        self._autoware_cfg = cfg.get("autoware", {})

        # —— Autoware Launch Settings ——
        self._root = Path(self._autoware_cfg.get("root", "/autoware"))
        self._ros_setup_script = self._autoware_cfg.get(
            "ros_setup_script", "/opt/ros/humble/setup.bash"
        )

        launch_cfg = self._autoware_cfg.get("launch", {})
        self._launch_package = launch_cfg.get("package", "autoware_launch")
        self._launch_file = launch_cfg.get("file", "sbsvf.launch.xml")
        self._headless = bool(launch_cfg.get("headless", True))
        self._extra_launch_args: List[str] = list(launch_cfg.get("extra_args", []))
        self._autoware_log_path = Path(
            launch_cfg.get("log_path", "/tmp/autoware_launch.log")
        )

        data_cfg = self._autoware_cfg.get("data", {})
        self._data_path = Path(data_cfg.get("data_path", "/autoware_data"))

        veh_cfg = self._autoware_cfg.get("vehicle", {})
        self._vehicle_model = veh_cfg.get("model", "sample_vehicle")
        self._sensor_model = veh_cfg.get("sensor_model", "sample_sensor_kit")

        self._rt_cfg = self._autoware_cfg.get("runtime", {})
        self._timeout_sec = float(self._rt_cfg.get("timeout_sec", 30.0))
        self._control_timeout_sec = float(self._rt_cfg.get("control_timeout_sec", 0.01))

        # ScenarioPack
        self._sps: Optional[ScenarioPack] = None
        self._map_path: Optional[Path] = None

        # —— Autoware process & ROS node status ——
        self._autoware_proc: Optional[subprocess.Popen] = None
        self._node: Optional[Node] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None

        # pub / sub / services
        self._publish_manager = None
        self._kinematic_state_pub = None
        self._accel_pub = None
        self._objects_pub = None
        self._dummy_pointcloud_pub = None
        self._control_mode_pub = None
        self._gear_report_pub = None
        self._steering_report_pub = None
        self._velocity_report_pub = None
        self._occupancy_grid_pub = None
        self._tf_broadcaster = None
        self._control_sub = None
        self._gear_cmd_sub = None
        self._autoware_state_sub = None
        self._client_initial_localization = None
        self._client_set_route_points = None
        self._client_change_to_auto = None

        # Adapter internal state
        self._initialized: bool = False
        self._base_time: float = 0.0  # time at sim_time = 0.0 (seconds)
        self._sim_time_stamp: float = 0.0  # time at current sim step (seconds)
        self._current_ros_time: float = (
            0.0  # _current_ros_time = _base_time + _sim_time_stamp
        )
        self._last_heavy_data_time: float = 0.0
        self._vehicle_state: Optional[int] = None
        self._control_mode: Optional[int] = (
            autoware_vehicle_msgs.ControlModeReport.NO_COMMAND
        )
        self._current_gear: Optional[int] = autoware_vehicle_msgs.GearCommand.NONE
        self._latest_control: autoware_control_msgs.Control = None
        self._latest_control_stamp = 0
        self._kinematic: ObjectKinematic = ObjectKinematic()
        self._prev_kinematic: ObjectKinematic = ObjectKinematic()
        self._prev_prev_kinematic: ObjectKinematic = ObjectKinematic()
        self._imu_state = sensor_msgs.Imu()
        self._quit_flag: bool = False
        self._last_error: Optional[str] = None
        self._agents: List[ObjectState] = []

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def init(self, sps: ScenarioPack) -> None:
        """
        - ROS node + spin thread
        - Launch Autoware (subprocess)
        - Wait for API services ready
        """

        self._setup_sps(sps)

        rclpy.init()
        self._ensure_ros_node()

        self._launch_autoware()

        # 等待 AD API services ready
        self._wait_for_service(
            self._client_initial_localization, "InitializeLocalization"
        )
        self._wait_for_service(self._client_set_route_points, "SetRoutePoints")
        self._wait_for_service(self._client_change_to_auto, "ChangeOperationMode")

        if self._quit_flag:
            self.stop()
            raise RuntimeError(
                f"AutowarePureAV init failed: {self._last_error or 'unknown error'}"
            )

        logger.info(f"Launching Autoware... (Current state: {self._vehicle_state})")
        logger.info("Autoware AV initialized and Autoware stack is ready.")

    def reset(
        self, sps: ScenarioPack, init_obs: Optional[list[ObjectState]] = None
    ) -> None:
        """
        Reset AV internal state when simulator resets.

        做兩件事：
        1. 如有換 map，就重啟 Autoware
        2. 用 AD API 設 initial pose / route
        """

        self._ensure_ros_node()

        # If the map has changed, restart Autoware process
        map_changed = self._setup_sps(sps)
        if map_changed:
            logger.info(
                f"Scenario uses new map_path={self._map_path}, restarting Autoware..."
            )
            self._stop_autoware_process()
            self._launch_autoware()

            self._wait_for_service(
                self._client_initial_localization, "InitializeLocalization"
            )
            self._wait_for_service(self._client_set_route_points, "SetRoutePoints")
            self._wait_for_service(self._client_change_to_auto, "ChangeOperationMode")

        # 清 internal state
        self._initialized = False
        self._base_time = self._current_ros_time
        self._sim_time_stamp = 0.0
        self._latest_control = None
        self._latest_control_stamp = 0
        self._control_mode = autoware_vehicle_msgs.ControlModeReport.NO_COMMAND
        self._current_gear = autoware_vehicle_msgs.GearCommand.NONE
        self._quit_flag = False
        self._last_error = None

        self._kinematic = ObjectKinematic()
        self._prev_kinematic = ObjectKinematic()
        self._prev_prev_kinematic = ObjectKinematic()

        # init_kinematic = ObjectKinematic.from_dict(ipos.to_dict())
        # init_kinematic.time = self._current_ros_time

        # # TODO: check position type consistency
        # init_kinematic.yaw = (
        #     ipos.h
        # )  # Position type and VehicleKinematic yaw consistency

        # # TODO: autoware init speed condition
        # init_kinematic.speed = 0.0  # Autoware state change condition

        # if init_obs is None or len(init_obs) == 0:

        init_kinematic = init_obs[0].kinematic
        init_kinematic.time = self._current_ros_time
        self._agents = init_obs[1:] if init_obs and len(init_obs) > 1 else []

        self._update_kinematic(init_kinematic)

        # 1) localization
        logger.info(f"Initializing Autoware... (Current state: {self._vehicle_state})")
        try:
            self._call_initialize_localization(sps)
        except RuntimeError as e:
            self._quit_flag = True
            self._last_error = str(e)
            raise RuntimeError("Failed to initialize Autoware localization.") from e

        # Wait for localization to be ready
        start = time.time()
        while (
            self._vehicle_state != autoware_system_msgs.AutowareState.WAITING_FOR_ROUTE
            and self._vehicle_state
            != autoware_system_msgs.AutowareState.WAITING_FOR_ENGAGE
            and time.time() - start < self._timeout_sec
        ):
            logger.info(
                f"Waiting for autoware localization... state:{self._vehicle_state} "
            )
            time.sleep(0.1)

        # Check if localization is ready
        if (
            self._vehicle_state != autoware_system_msgs.AutowareState.WAITING_FOR_ROUTE
            and self._vehicle_state
            != autoware_system_msgs.AutowareState.WAITING_FOR_ENGAGE
        ):
            logger.error("Autoware localization initialization timed out.")
            self._quit_flag = True
            self._last_error = "Autoware localization initialization timed out."
            raise RuntimeError("Autoware localization initialization timed out.")

        logger.info("Autoware localization initialized.")

        # 2) routing
        logger.info(
            f"Setting Autoware route points... (Current state: {self._vehicle_state})"
        )
        try:
            self._call_set_route_points(sps)
        except RuntimeError as e:
            self._quit_flag = True
            self._last_error = str(e)
            raise RuntimeError("Failed to set Autoware route points.") from e

        # Wait for route to be set
        start = time.time()
        while (
            self._vehicle_state != autoware_system_msgs.AutowareState.WAITING_FOR_ENGAGE
            and time.time() - start < self._timeout_sec
        ):
            logger.info(f"Waiting for autoware planning... ")
            time.sleep(0.1)

        # Check if route is set
        if self._vehicle_state != autoware_system_msgs.AutowareState.WAITING_FOR_ENGAGE:
            logger.error("Autoware planning timed out.")
            self._quit_flag = True
            self._last_error = "Autoware planning timed out."
            raise RuntimeError("Autoware planning timed out.")

        logger.info("Autoware reset ready. Ready to engage.")

    def step(self, obs: Dict[str, Any], time_stamp: float) -> Ctrl:
        """
        - 發 ego state + optional agents 給 Autoware
        - 等待一筆「新的」 control_cmd（最多 control_timeout_sec）
        - 轉成 Ctrl 回傳

        obs 假設格式：
        {
          "ego": {...},      # 必要
          "agents": [...],   # 可選
        }
        """
        self._ensure_ros_node()

        self._sim_time_stamp = time_stamp
        self._current_ros_time = self._base_time + self._sim_time_stamp

        # Check Autoware vehicle state
        if (
            self._vehicle_state != autoware_system_msgs.AutowareState.WAITING_FOR_ENGAGE
            and self._vehicle_state != autoware_system_msgs.AutowareState.DRIVING
        ):
            logger.warning(
                f"Autoware not in driving mode, current state: {self._vehicle_state}"
            )
            return Ctrl(mode=CtrlMode.None_)

        # First step: change to autonomous mode
        if self._vehicle_state == autoware_system_msgs.AutowareState.WAITING_FOR_ENGAGE:
            logger.info("Changing Autoware to autonomous mode...")
            try:
                # input("Press Enter to change Autoware to autonomous mode...")
                self._control_mode = autoware_vehicle_msgs.ControlModeReport.AUTONOMOUS
                self._call_change_to_autonomous()
            except RuntimeError as e:
                self._quit_flag = True
                self._last_error = str(e)
                raise RuntimeError(
                    "Failed to change Autoware to autonomous mode."
                ) from e

            # Wait for change to autonomous
            start = time.time()
            while (
                self._vehicle_state != autoware_system_msgs.AutowareState.DRIVING
                and time.time() - start < self._timeout_sec
            ):
                logger.info(f"Waiting for autoware to enter autonomous mode... ")
                time.sleep(0.1)

            # Check if changed to autonomous
            if self._vehicle_state != autoware_system_msgs.AutowareState.DRIVING:
                logger.error("Autoware change to autonomous mode timed out.")
                self._quit_flag = True
                self._last_error = "Autoware change to autonomous mode timed out."
                raise RuntimeError("Autoware change to autonomous mode timed out.")

            self._initialized = True
            logger.info("Autoware is running...")

        # Update ego's kinematic state
        ego = obs[0]
        if ego is not None:
            cur_kinematic = ego.kinematic
            cur_kinematic.time = self._current_ros_time
            self._update_kinematic(cur_kinematic)

        self._agents = obs[1:] if len(obs) > 1 else []

        # publish
        now = self._convert_float_to_ros_time(self._current_ros_time)
        self._publish_manager.publish_all(now)

        if self._rt_cfg.get("wait_control", False):
            deadline = time.time() + self._control_timeout_sec
            while time.time() < deadline:
                if (
                    self._latest_control is not None
                    and self._latest_control.stamp.sec * 1e9
                    + self._latest_control.stamp.nanosec
                    > self._latest_control_stamp
                ):
                    break
                time.sleep(0.001)

        print(
            f"Latest control stamp: {self._latest_control.stamp.sec * 1e9 + self._latest_control.stamp.nanosec if self._latest_control is not None else None}, Current time: {self._current_ros_time*1e9}"
        )

        if self._latest_control is None:
            logger.warning(
                "No control message received from Autoware, returning zero Ctrl"
            )
            return Ctrl(mode=CtrlMode.None_)

        # Apply control
        self._latest_control_stamp = (
            self._latest_control.stamp.sec * 1e9 + self._latest_control.stamp.nanosec
        )
        speed = float(self._latest_control.longitudinal.velocity)
        steering = float(self._latest_control.lateral.steering_tire_angle)

        return Ctrl(
            mode=CtrlMode.VEL_STEER,
            payload={
                "speed": speed,
                "h": steering,
            },
        )

    def stop(self) -> None:
        """關閉 Autoware process + ROS node / executor"""
        self._stop_autoware_process()

        if self._executor and self._node:
            self._executor.remove_node(self._node)

        if self._node:
            self._node.destroy_node()
            self._node = None

        rclpy.try_shutdown()

        logger.info("Autoware AV stopped.")

    def should_quit(self) -> bool:
        """
        True if:
        - internal error / service 失敗
        - Autoware process 掛掉
        - MotionState: 曾 MOVING，現在 STOPPED
        """
        if self._quit_flag:
            logger.info("AutowareAV.should_quit: quit_flag set")
            return True

        # Autoware process 狀態
        if self._autoware_proc is not None and self._autoware_proc.poll() is not None:
            logger.info("Autoware process has exited unexpectedly.")
            return True

        return False

    # ------------------------------------------------------------------
    # ROS node / spin / process
    # ------------------------------------------------------------------
    def _ensure_ros_node(self) -> None:

        if self._node is not None:
            return

        self._node = rclpy.create_node("autoware_av_adapter")
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._node.create_timer(1.0 / CLOCK_PUB_HZ, self._timer_callback)
        self._publish_manager = PublishManager()

        # QoS profile:
        # Reliability: RELIABLE
        # History (Depth): KEEP_LAST (1)
        # Durability: VOLATILE
        # Lifespan: Infinite
        # Deadline: Infinite
        # Liveliness: AUTOMATIC
        # Liveliness lease duration: Infinite

        qos_profile = QoSProfile(depth=1)

        # publishers
        self._kinematic_state_pub = self._node.create_publisher(
            nav_msgs.Odometry,
            "/localization/kinematic_state",
            qos_profile,
        )
        self._publish_manager.add(
            TopicPublisher(
                name="kinematic_state",
                rate_hz=40.0,
                mode=PublishMode.FIXED_RATE,
                enabled=True,
                publish_fn=lambda t: self._publish_kinematic_state(t),
            )
        )

        self._accel_pub = self._node.create_publisher(
            geometry_msgs.AccelWithCovarianceStamped,
            "/localization/acceleration",
            qos_profile,
        )
        self._publish_manager.add(
            TopicPublisher(
                name="accel",
                rate_hz=40.0,
                mode=PublishMode.FIXED_RATE,
                enabled=True,
                publish_fn=lambda t: self._publish_accel(t),
            )
        )

        self._objects_pub = self._node.create_publisher(
            autoware_perception_msgs.DetectedObjects,
            "/perception/object_recognition/detection/objects",
            qos_profile,
        )
        self._publish_manager.add(
            TopicPublisher(
                name="dynamic_objects",
                rate_hz=10.0,
                mode=PublishMode.FIXED_RATE,
                enabled=True,
                publish_fn=lambda t: self._publish_dynamic_objects(t),
            )
        )

        self._dummy_pointcloud_pub = self._node.create_publisher(
            sensor_msgs.PointCloud2,
            "/perception/obstacle_segmentation/pointcloud",
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._publish_manager.add(
            TopicPublisher(
                name="dummy_pointcloud",
                rate_hz=10.0,
                mode=PublishMode.FIXED_RATE,
                enabled=True,
                publish_fn=lambda t: self._publish_dummy_pointcloud(t),
            )
        )

        self._control_mode_pub = self._node.create_publisher(
            autoware_vehicle_msgs.ControlModeReport,
            "/vehicle/status/control_mode",
            qos_profile,
        )
        self._publish_manager.add(
            TopicPublisher(
                name="control_mode",
                rate_hz=40.0,
                mode=PublishMode.FIXED_RATE,
                enabled=True,
                publish_fn=lambda t: self._publish_control_mode(t),
            )
        )

        self._gear_report_pub = self._node.create_publisher(
            autoware_vehicle_msgs.GearReport,
            "/vehicle/status/gear_status",
            qos_profile,
        )
        self._publish_manager.add(
            TopicPublisher(
                name="gear_report",
                rate_hz=40.0,
                mode=PublishMode.FIXED_RATE,
                enabled=True,
                publish_fn=lambda t: self._publish_gear_report(t),
            )
        )

        self._steering_report_pub = self._node.create_publisher(
            autoware_vehicle_msgs.SteeringReport,
            "/vehicle/status/steering_status",
            qos_profile,
        )
        self._publish_manager.add(
            TopicPublisher(
                name="steering_report",
                rate_hz=40.0,
                mode=PublishMode.FIXED_RATE,
                enabled=True,
                publish_fn=lambda t: self._publish_steering_report(t),
            )
        )

        self._velocity_report_pub = self._node.create_publisher(
            autoware_vehicle_msgs.VelocityReport,
            "/vehicle/status/velocity_status",
            qos_profile,
        )
        self._publish_manager.add(
            TopicPublisher(
                name="velocity_report",
                rate_hz=40.0,
                mode=PublishMode.FIXED_RATE,
                enabled=True,
                publish_fn=lambda t: self._publish_velocity_report(t),
            )
        )

        self._occupancy_grid_pub = self._node.create_publisher(
            nav_msgs.OccupancyGrid,
            "/perception/occupancy_grid_map/map",
            qos_profile,
        )
        self._publish_manager.add(
            TopicPublisher(
                name="occupancy_grid",
                rate_hz=10.0,
                mode=PublishMode.FIXED_RATE,
                enabled=True,
                publish_fn=lambda t: self._publish_occupancy_grid(t),
            )
        )

        self._clock_pub = self._node.create_publisher(
            rosgraph_msgs.Clock,
            "/clock",
            qos_profile,
        )
        self._publish_manager.add(
            TopicPublisher(
                name="clock",
                rate_hz=100.0,
                mode=PublishMode.ALWAYS,
                enabled=True,
                publish_fn=lambda t: self._publish_clock(t),
            )
        )

        self._tf_broadcaster = TransformBroadcaster(self._node)
        self._publish_manager.add(
            TopicPublisher(
                name="tf",
                rate_hz=40.0,
                mode=PublishMode.FIXED_RATE,
                enabled=True,
                publish_fn=lambda t: self._publish_tf(t),
            )
        )

        # subscribers
        self._control_sub = self._node.create_subscription(
            autoware_control_msgs.Control,
            "/control/command/control_cmd",
            self._on_control,
            qos_profile,
        )

        self._autoware_state_sub = self._node.create_subscription(
            autoware_system_msgs.AutowareState,
            "/autoware/state",
            self._on_autoware_state,
            qos_profile,
        )

        self._gear_cmd_sub = self._node.create_subscription(
            autoware_vehicle_msgs.GearCommand,
            "/control/command/gear_cmd",
            self._on_gear_command,
            qos_profile,
        )

        # services
        self._client_initial_localization = self._node.create_client(
            autoware_adapi_v1_msgs_srv.InitializeLocalization,
            "/api/localization/initialize",
        )
        self._client_clear_route = self._node.create_client(
            autoware_adapi_v1_msgs_srv.ClearRoute,
            "/api/routing/clear_route",
        )
        self._client_set_route_points = self._node.create_client(
            autoware_adapi_v1_msgs_srv.SetRoutePoints,
            "/api/routing/set_route_points",
        )
        self._client_change_to_auto = self._node.create_client(
            autoware_adapi_v1_msgs_srv.ChangeOperationMode,
            "/api/operation_mode/change_to_autonomous",
        )

        # spin thread
        self._spin_thread = threading.Thread(target=self._spin, daemon=False)
        self._spin_thread.start()

    def _spin(self) -> None:
        assert self._executor is not None
        try:
            self._executor.spin()
        except Exception as e:  # noqa: BLE001
            logger.error(f"AutowareAV executor error: {e}")
            self._last_error = str(e)
            self._quit_flag = True
            # break

    def _launch_autoware(self) -> None:
        launch_parts = [
            f"cd {self._root}",
            f"source {self._ros_setup_script}",
            f"""ros2 launch {self._launch_package} {self._launch_file} \
            map_path:={self._map_path.parent} \
            lanelet2_map_file:={self._map_path.name} \
            data_path:={self._data_path} \
            vehicle_model:={self._vehicle_model} \
            sensor_model:={self._sensor_model} \
            launch_sensing:=false \
            launch_localization:=false \
            launch_perception:=false \
            launch_vehicle_interface:=false \
            system_run_mode:=planning_simulation \
            launch_system_monitor:=false \
            launch_dummy_diag_publisher:=true \
            enable_all_modules_auto_mode:=true \
            is_simulation:=true \
            rviz:={'false' if self._headless else 'true'} \
            """,
        ]
        launch_parts.extend(self._extra_launch_args)

        full_cmd = " && ".join(launch_parts)
        logger.info(f"Launching Autoware: {full_cmd}")
        log = open(self._autoware_log_path, "ab", buffering=0)
        self._autoware_proc = subprocess.Popen(
            ["bash", "-lc", full_cmd],
            stdout=log,
            stderr=log,
            preexec_fn=os.setsid,
        )

    def _stop_autoware_process(self) -> None:
        """
        安全地關掉整個 Autoware process group
        """
        if self._autoware_proc is None:
            return

        if self._autoware_proc.poll() is not None:
            self._autoware_proc = None
            return

        logger.info("Terminating Autoware process group...")

        try:
            pgid = os.getpgid(self._autoware_proc.pid)

            # 先優雅關閉
            os.killpg(pgid, signal.SIGTERM)

            # 等待 bash（group leader）結束
            self._autoware_proc.wait(timeout=5.0)

        except subprocess.TimeoutExpired:
            logger.warning(
                "Autoware did not terminate gracefully killing process group..."
            )
            os.killpg(pgid, signal.SIGKILL)

        except ProcessLookupError:
            # process 已經不存在
            pass

        finally:
            self._autoware_proc = None

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _timer_callback(self):
        if not self._initialized:
            self._base_time += 1.0 / CLOCK_PUB_HZ
            self._current_ros_time = self._base_time

            self._kinematic.time = self._current_ros_time
            self._update_kinematic(self._kinematic)

            now = self._convert_float_to_ros_time(self._current_ros_time)
            self._publish_manager.publish_all(now)

    def _on_control(self, msg: autoware_control_msgs.Control) -> None:
        self._latest_control = msg

    def _on_autoware_state(self, msg: autoware_system_msgs.AutowareState) -> None:
        self._vehicle_state = msg.state

    def _on_gear_command(self, msg: autoware_vehicle_msgs.GearCommand) -> None:
        self._current_gear = msg.command

    # ------------------------------------------------------------------
    # AD API calls
    # ------------------------------------------------------------------
    def _wait_for_service(
        self, client, name: str, timeout_sec: Optional[float] = None
    ) -> None:
        timeout = timeout_sec or self._timeout_sec
        start = time.time()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() - start > timeout:
                msg = f"Service {name} not available after {timeout}s"
                self._last_error = msg
                self._quit_flag = True
                return
            logger.info(f"Waiting for Autoware service {name}...")
        logger.info(f"Service {name} is available.")

    def _call_initialize_localization(self, sps: ScenarioPack) -> None:
        assert self._node is not None
        now = self._convert_float_to_ros_time(self._current_ros_time).to_msg()

        t = geometry_msgs.TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = float(self._kinematic.x)
        t.transform.translation.y = float(self._kinematic.y)
        t.transform.translation.z = float(self._kinematic.z)

        # Euler angle to quaternion
        qz, qw = self._yaw_to_quat(self._kinematic.yaw)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        req = autoware_adapi_v1_msgs_srv.InitializeLocalization.Request()
        pose_msg = geometry_msgs.PoseWithCovarianceStamped()
        # pose_msg.header.stamp = self._node.get_clock().now().to_msg()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "map"

        pose_msg.pose.pose.position.x = float(self._kinematic.x)
        pose_msg.pose.pose.position.y = float(self._kinematic.y)
        pose_msg.pose.pose.position.z = float(self._kinematic.z)
        logger.info(
            f"Setting initial position: x={self._kinematic.x}, y={self._kinematic.y}, z={self._kinematic.z}, h={self._kinematic.yaw}, speed={self._kinematic.speed}"
        )
        qz, qw = self._yaw_to_quat(self._kinematic.yaw)
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw

        pose_msg.pose.covariance = [0.0] * 36

        req.pose = [pose_msg]

        fut = self._client_initial_localization.call_async(req)

        start = time.time()
        while rclpy.ok() and not fut.done() and time.time() - start < self._timeout_sec:
            time.sleep(0.01)

        res = fut.result()
        if res is None or not res.status.success:
            status_msg = getattr(res.status, "message", None) if res else "no response"
            code = getattr(res.status, "code", "unknown") if res else "no response"
            succ = getattr(res.status, "success", "unknown") if res else "no response"
            msg = f"InitializeLocalization failed: code={code}, success={succ}, message={status_msg}"
            raise RuntimeError(msg)

        logger.debug("Called InitializeLocalization service.")

    def _call_set_route_points(self, sps: ScenarioPack) -> None:
        assert self._node is not None
        # Clear route
        req = autoware_adapi_v1_msgs_srv.ClearRoute.Request()
        fut = self._client_clear_route.call_async(req)
        start = time.time()
        while rclpy.ok() and not fut.done() and time.time() - start < self._timeout_sec:
            time.sleep(0.01)
        res = fut.result()
        if res is None or not res.status.success:
            status_msg = getattr(res.status, "message", None) if res else "no response"
            code = getattr(res.status, "code", "unknown") if res else "no response"
            succ = getattr(res.status, "success", "unknown") if res else "no response"
            msg = (
                f"ClearRoute failed: code={code}, success={succ}, message={status_msg}"
            )
            raise RuntimeError(msg)

        req = autoware_adapi_v1_msgs_srv.SetRoutePoints.Request()
        req.header.frame_id = "map"
        # req.header.stamp = self._node.get_clock().now().to_msg()
        req.header.stamp = self._convert_float_to_ros_time(
            self._current_ros_time
        ).to_msg()

        gp = sps.ego.goal.position
        goal = geometry_msgs.Pose()
        goal.position.x = float(gp.x)
        goal.position.y = float(gp.y)
        goal.position.z = float(gp.z)

        qz, qw = self._yaw_to_quat(gp.h)
        goal.orientation.z = qz
        goal.orientation.w = qw

        req.goal = goal
        req.waypoints = []

        req.option.allow_goal_modification = bool(
            self._rt_cfg.get("allow_goal_modification", False)
        )

        fut = self._client_set_route_points.call_async(req)
        start = time.time()
        while rclpy.ok() and not fut.done() and time.time() - start < self._timeout_sec:
            time.sleep(0.01)
        res = fut.result()
        if res is None or not res.status.success:
            status_msg = getattr(res.status, "message", None) if res else "no response"
            code = getattr(res.status, "code", "unknown") if res else "no response"
            succ = getattr(res.status, "success", "unknown") if res else "no response"
            msg = f"SetRoutePoints failed: code={code}, success={succ}, message={status_msg}"
            raise RuntimeError(msg)

    def _call_change_to_autonomous(self) -> None:
        assert self._node is not None

        req = autoware_adapi_v1_msgs_srv.ChangeOperationMode.Request()
        fut = self._client_change_to_auto.call_async(req)
        # rclpy.spin_until_future_complete(self._node, fut)
        start = time.time()
        while rclpy.ok() and not fut.done() and time.time() - start < self._timeout_sec:
            time.sleep(0.01)
        res = fut.result()
        if res is None or not res.status.success:
            msg = f"ChangeOperationMode(AUTONOMOUS) failed: {getattr(res.status, 'message', 'unknown') if res else 'no response'}"
            self._last_error = msg
            self._quit_flag = True
            raise RuntimeError(msg)

    # ------------------------------------------------------------------
    # publish helpers
    # ------------------------------------------------------------------
    def _publish_kinematic_state(self, t: rclpy.time.Time) -> None:
        assert self._node is not None

        now = t.to_msg()

        # ODOMETRY
        msg = nav_msgs.Odometry()
        msg.header.stamp = now
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self._kinematic.x
        msg.pose.pose.position.y = self._kinematic.y
        msg.pose.pose.position.z = self._kinematic.z

        yaw = self._kinematic.yaw
        qz, qw = self._yaw_to_quat(yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = self._kinematic.speed

        self._kinematic_state_pub.publish(msg)

    def _publish_accel(self, t: rclpy.time.Time) -> None:
        now = t.to_msg()
        accel = geometry_msgs.AccelWithCovarianceStamped()
        accel.header.stamp = now
        accel.header.frame_id = "base_link"
        accel.accel.accel.linear = self._imu_state.linear_acceleration
        accel.accel.accel.angular = self._imu_state.angular_velocity
        self._accel_pub.publish(accel)

    def _publish_dynamic_objects(self, t: rclpy.time.Time) -> None:
        msg = autoware_perception_msgs.DetectedObjects()
        msg.header.stamp = t.to_msg()

        msg.header.frame_id = "map"

        for ag in self._agents:
            obj = autoware_perception_msgs.DetectedObject()

            # 1. existence_probability
            obj.existence_probability = 1.0

            # 2. Classification
            clas = autoware_perception_msgs.ObjectClassification()
            clas.label = OBJECT_TYPE_MAP.get(
                ag.type, autoware_perception_msgs.ObjectClassification.UNKNOWN
            )
            clas.probability = 1.0
            obj.classification = [clas]

            # 3. Shape
            shp = autoware_perception_msgs.Shape()

            if ag.shape.type == ShapeType.CYLINDER:
                shp.type = autoware_perception_msgs.Shape.CYLINDER
            elif ag.shape.type == ShapeType.BOUNDING_BOX:
                shp.type = autoware_perception_msgs.Shape.BOUNDING_BOX
            elif ag.shape.type == ShapeType.POLYGON:
                shp.type = autoware_perception_msgs.Shape.POLYGON
            else:
                raise ValueError(f"Unknown shape type: {ag.shape.type}")

            if ag.shape.type != ShapeType.POLYGON:
                shp.dimensions.x = ag.shape.dimensions[0]
                shp.dimensions.y = ag.shape.dimensions[1]
                shp.dimensions.z = ag.shape.dimensions[2]
            else:
                for pt in ag.shape.polygon:
                    p = geometry_msgs.Point32()
                    p.x = pt[0]
                    p.y = pt[1]
                    p.z = pt[2]
                    shp.footprint.points.append(p)
                shp.height = ag.shape.dimensions[2]

            obj.shape = shp

            # 4. Kinematics
            kin = autoware_perception_msgs.DetectedObjectKinematics()

            kin.orientation_availability = (
                2  # (0:UNAVAILABLE, 1:SIGN_UNKNOWN, 2:AVAILABLE)
            )
            kin.has_position_covariance = True

            # Pose
            kin.pose_with_covariance.pose.position.x = ag.kinematic.x
            kin.pose_with_covariance.pose.position.y = ag.kinematic.y
            kin.pose_with_covariance.pose.position.z = ag.kinematic.z

            qz, qw = self._yaw_to_quat(ag.kinematic.yaw)
            kin.pose_with_covariance.pose.orientation.z = qz
            kin.pose_with_covariance.pose.orientation.w = qw

            sx, sy, sz = 0.05, 0.05, 0.10  # meters
            syaw = 0.017  # rad (≈1 deg)

            kin.pose_with_covariance.covariance[0] = sx * sx
            kin.pose_with_covariance.covariance[7] = sy * sy
            kin.pose_with_covariance.covariance[14] = sz * sz
            kin.pose_with_covariance.covariance[35] = syaw * syaw

            # Twist
            kin.has_twist = True
            kin.twist_with_covariance.twist.linear.x = ag.kinematic.speed
            kin.twist_with_covariance.twist.angular.z = 0.0
            # TODO: Agent's twist calculation
            kin.has_twist_covariance = False
            # agent_speed = ag.kinematic.speed
            # kin.twist_with_covariance.twist.linear.x = agent_speed

            obj.kinematics = kin
            msg.objects.append(obj)

        self._objects_pub.publish(msg)

    def _publish_dummy_pointcloud(self, t: rclpy.time.Time) -> None:
        # Empty PointCloud2
        msg = sensor_msgs.PointCloud2()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = "base_link"
        msg.height = 1
        msg.width = 0
        msg.is_dense = True
        msg.is_bigendian = False
        x = sensor_msgs.PointField()
        x.name = "x"
        x.offset = 0
        x.datatype = sensor_msgs.PointField.FLOAT32
        x.count = 1
        y = sensor_msgs.PointField()
        y.name = "y"
        y.offset = 4
        y.datatype = sensor_msgs.PointField.FLOAT32
        y.count = 1
        z = sensor_msgs.PointField()
        z.name = "z"
        z.offset = 8
        z.datatype = sensor_msgs.PointField.FLOAT32
        z.count = 1
        intensity = sensor_msgs.PointField()
        intensity.name = "intensity"
        intensity.offset = 12
        intensity.datatype = sensor_msgs.PointField.UINT8
        intensity.count = 1
        returntype = sensor_msgs.PointField()
        returntype.name = "return_type"
        returntype.offset = 13
        returntype.datatype = sensor_msgs.PointField.UINT8
        returntype.count = 1
        channel = sensor_msgs.PointField()
        channel.name = "channel"
        channel.offset = 14
        channel.datatype = sensor_msgs.PointField.UINT16
        channel.count = 1
        msg.fields = [x, y, z, intensity, returntype, channel]
        msg.point_step = 16
        msg.row_step = 0
        msg.data = b""
        self._dummy_pointcloud_pub.publish(msg)

    def _publish_tf(self, t: rclpy.time.Time) -> None:
        if self._kinematic is None:
            logger.warning("No kinematic state skipping TF publish")
            return
        now = t.to_msg()

        t = geometry_msgs.TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self._kinematic.x
        t.transform.translation.y = self._kinematic.y
        t.transform.translation.z = self._kinematic.z

        t.transform.rotation.z, t.transform.rotation.w = self._yaw_to_quat(
            self._kinematic.yaw
        )

        # publish
        self._tf_broadcaster.sendTransform(t)

    def _publish_control_mode(self, t: rclpy.time.Time) -> None:
        msg = autoware_vehicle_msgs.ControlModeReport()
        msg.stamp = t.to_msg()
        msg.mode = self._control_mode
        self._control_mode_pub.publish(msg)

    def _publish_gear_report(self, t: rclpy.time.Time) -> None:
        msg = autoware_vehicle_msgs.GearReport()
        msg.stamp = t.to_msg()
        msg.report = self._current_gear
        self._gear_report_pub.publish(msg)

    def _publish_steering_report(self, t: rclpy.time.Time) -> None:
        msg = autoware_vehicle_msgs.SteeringReport()
        msg.stamp = t.to_msg()
        angle = (
            self._latest_control.lateral.steering_tire_angle
            if self._latest_control
            else 0.0
        )
        msg.steering_tire_angle = angle
        self._steering_report_pub.publish(msg)

    def _publish_velocity_report(self, t: rclpy.time.Time) -> None:
        msg = autoware_vehicle_msgs.VelocityReport()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = "base_link"
        msg.longitudinal_velocity = self._kinematic.speed
        msg.lateral_velocity = 0.0
        # TODO: 直接從sim拿 heading rate
        msg.heading_rate = self._imu_state.angular_velocity.z

        self._velocity_report_pub.publish(msg)

    def _publish_occupancy_grid(self, t: rclpy.time.Time) -> None:
        msg = nav_msgs.OccupancyGrid()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = "map"

        # 這裡可以根據需要填入實際的 occupancy grid 資料
        # 目前先填入空的 grid
        msg.info.resolution = 0.5  # 每個格子的大小 (公尺)
        msg.info.width = 200  # 格子數量 (寬)
        msg.info.height = 200  # 格子數量 (高)
        msg.info.origin.position.x = -50.0  # 原點位置
        msg.info.origin.position.y = -50.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # 填入資料 (全部未知)
        msg.data = [-1] * (msg.info.width * msg.info.height)

        self._occupancy_grid_pub.publish(msg)

    def _publish_clock(self, t: rclpy.time.Time) -> None:
        msg = rosgraph_msgs.Clock()
        msg.clock = t.to_msg()
        self._clock_pub.publish(msg)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _convert_float_to_ros_time(self, t: float) -> rclpy.time.Time:
        sec = int(t)
        nanosec = int((t - sec) * 1e9)
        return rclpy.time.Time(seconds=sec, nanoseconds=nanosec)

    def _update_kinematic(self, kinematic: ObjectKinematic) -> None:
        self._prev_prev_kinematic = self._prev_kinematic
        self._prev_kinematic = self._kinematic
        self._kinematic = kinematic
        self._calc_imu_state()

    def _setup_sps(self, sps: ScenarioPack) -> bool:
        """
        Update map path from ScenarioPack.
        Return True if map path has changed.
        """

        # Update sps
        self._sps = sps

        # Map path
        map_full_path = Path(sps.maps.get("osm"))
        if not map_full_path.exists():
            raise FileNotFoundError(f"Autoware map file not found: {map_full_path}")

        if map_full_path.suffix.lower() != ".osm":
            raise ValueError(
                f"Autoware map file must be .osm format, got: {map_full_path}"
            )

        # Check if changed
        is_changed = self._map_path != map_full_path

        # Update map path
        self._map_path = map_full_path

        return is_changed

    def _calc_imu_state(self) -> None:
        """根據 kinematic 計算 IMU 狀態
        修正重點：
        1. Y軸變數修正
        2. 加速度轉回 Body Frame
        3. 處理 Yaw 角度跳變
        """
        # 避免除以 0
        dt1 = max(self._kinematic.time - self._prev_kinematic.time, 1e-5)
        dt2 = max(self._prev_kinematic.time - self._prev_prev_kinematic.time, 1e-5)
        dt_avg = (dt1 + dt2) / 2.0

        # 1. 計算全域速度 (Global Velocity)
        cur_v_x = (self._kinematic.x - self._prev_kinematic.x) / dt1
        cur_v_y = (self._kinematic.y - self._prev_kinematic.y) / dt1

        prev_v_x = (self._prev_kinematic.x - self._prev_prev_kinematic.x) / dt2
        prev_v_y = (self._prev_kinematic.y - self._prev_prev_kinematic.y) / dt2

        # 2. 計算全域加速度 (Global Acceleration)
        acc_x_global = (cur_v_x - prev_v_x) / dt_avg
        acc_y_global = (cur_v_y - prev_v_y) / dt_avg

        # 3. [關鍵] 將全域加速度轉回車身座標 (Map -> Base_Link)
        # 使用旋轉矩陣:
        # ax_body =  ax_global * cos(yaw) + ay_global * sin(yaw)
        # ay_body = -ax_global * sin(yaw) + ay_global * cos(yaw)
        current_yaw = self._kinematic.yaw
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)

        linear_acceleration = geometry_msgs.Vector3()
        linear_acceleration.x = acc_x_global * cos_yaw + acc_y_global * sin_yaw
        linear_acceleration.y = -acc_x_global * sin_yaw + acc_y_global * cos_yaw
        linear_acceleration.z = 0.0  # 2D 平面假設，忽略重力

        # 4. 計算角速度 (Angular Velocity) 並處理 Wrap-around
        diff_yaw = self._kinematic.yaw - self._prev_kinematic.yaw

        # [修正] 處理角度跨越 +-PI 的情況
        while diff_yaw > math.pi:
            diff_yaw -= 2.0 * math.pi
        while diff_yaw < -math.pi:
            diff_yaw += 2.0 * math.pi

        angular_velocity = geometry_msgs.Vector3()
        angular_velocity.x = 0.0
        angular_velocity.y = 0.0
        # 這裡建議使用 dt1 (當前區間) 比較能代表當下瞬間角速度
        angular_velocity.z = diff_yaw / dt1

        # 賦值
        self._imu_state.linear_acceleration = linear_acceleration
        self._imu_state.angular_velocity = angular_velocity
        # self._imu_state.header.stamp = self._node.get_clock().now().to_msg()
        self._imu_state.header.stamp = self._convert_float_to_ros_time(
            self._current_ros_time
        ).to_msg()
        self._imu_state.header.frame_id = "base_link"

    @staticmethod
    def _yaw_to_quat(yaw: float) -> tuple[float, float]:
        """Supposing roll=pitch=0, return quaternion z,w from yaw"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return sy, cy

    @staticmethod
    def _quat_to_yaw(q) -> float:
        """Supposing roll=pitch=0, return yaw from quaternion"""
        return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
