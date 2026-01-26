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


@register_av("autoware_pure")
class AutowarePureAV:
    """
    Autoware AV adapter:
    - init(): 啟 Autoware (subprocess) + 建 ROS node、pub/sub、services
    - reset(): 用 AD API 設 initial pose + route + 切 autonomous
    - step(): 送 obs（ego + agents），等待新控制，轉成 Ctrl 回 simulator
    - stop(): 關掉 Autoware process + ROS node
    - should_quit(): 依 motion state / error / process 狀態決定是否結束
    """

    def __init__(self, cfg_path: Path, sps: Any, runtime_cfg: dict):
        cfg = get_cfg(Path(cfg_path))
        self._autoware_cfg = cfg.get("autoware", {})

        # —— Autoware 啟動相關設定 ——
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

        self._dt = runtime_cfg.get("dt", 0.002)
        if self._dt <= 0.0:
            self._dt = 0.01  # default 100Hz

        data_cfg = self._autoware_cfg.get("data", {})
        self._data_path = Path(data_cfg.get("data_path", "/autoware_data"))

        veh_cfg = self._autoware_cfg.get("vehicle", {})
        self._vehicle_model = veh_cfg.get("model", "sample_vehicle")
        self._sensor_model = veh_cfg.get("sensor_model", "sample_sensor_kit")

        rt_cfg = self._autoware_cfg.get("runtime", {})
        self._timeout_sec = float(rt_cfg.get("timeout_sec", 30.0))
        self._control_timeout_sec = float(rt_cfg.get("control_timeout_sec", 0.1))
        # self._spin_rate_hz = float(rt_cfg.get("spin_rate_hz", 100.0))

        # ScenarioPack
        self._sps: Optional[ScenarioPack] = sps
        self._current_map_path: Optional[Path] = None
        self._map_base_path: Optional[Path] = None
        self._lanelet2_map_file: Optional[Path] = None

        # —— Autoware process & ROS node 狀態 ——
        self._autoware_proc: Optional[subprocess.Popen] = None
        self._node: Optional[Node] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None

        # pub / sub / services
        self._kinematic_state_pub = None
        self._accel_pub = None
        self._objects_pub = None
        self._dummy_pointcloud_pub = None
        self._control_mode_pub = None
        self._gear_report_pub = None
        self._steering_report_pub = None
        self._velocity_report_pub = None
        self._occupancy_grid_pub = None
        # self._imu_pub = None
        self._tf_broadcaster = None
        self._control_sub = None
        self._gear_cmd_sub = None
        self._autoware_state_sub = None
        self._client_initial_localization = None
        self._client_set_route_points = None
        self._client_change_to_auto = None

        # 狀態
        self._initialized: bool = False
        self._base_time: float = 0.0  # time at sim_time = 0.0 (seconds)
        self._sim_time_stamp: float = 0.0  # time at current sim step (seconds)
        self._current_ros_time: float = 0.0  # = _base_time + _sim_time_stamp
        self._last_heavy_data_time: float = 0.0
        self._vehicle_state: Optional[int] = None
        self._control_mode = autoware_vehicle_msgs.ControlModeReport.MANUAL
        self._current_gear: Optional[int] = None
        self._latest_control: autoware_control_msgs.Control = (
            autoware_control_msgs.Control()
        )
        self._latest_control_stamp = None
        self._kinematic: ObjectKinematic = ObjectKinematic()
        self._prev_kinematic: ObjectKinematic = ObjectKinematic()
        self._prev_prev_kinematic: ObjectKinematic = ObjectKinematic()
        self._imu_state = sensor_msgs.Imu()
        # self._motion_state: int = MotionState.UNKNOWN
        self._quit_flag: bool = False
        self._last_error: Optional[str] = None
        self._agents: List[ObjectState] = []
        # AutowarePureAV._instance_count += 1

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def init(self) -> None:
        """
        - ROS node + spin thread
        - Launch Autoware (subprocess)
        - Wait for API services ready
        """

        rclpy.init()
        self._ensure_ros_node()

        self._current_map_path = self._resolve_map_path(self._sps)
        # input("press enter to launch autoware with map: " + str(map_path))
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

        self._sps = sps

        # If the map has changed, restart Autoware process
        new_map_path = self._resolve_map_path(sps)
        if self._current_map_path is None or new_map_path != self._current_map_path:
            logger.info(
                f"Scenario uses new map_path={new_map_path}, restarting Autoware..."
            )
            self._stop_autoware_process()
            self._launch_autoware()
            self._current_map_path = new_map_path

            self._wait_for_service(
                self._client_initial_localization, "InitializeLocalization"
            )
            self._wait_for_service(self._client_set_route_points, "SetRoutePoints")
            self._wait_for_service(self._client_change_to_auto, "ChangeOperationMode")

        # 清 internal state
        self._latest_control = autoware_control_msgs.Control()
        self._latest_control_stamp = None
        # self._motion_state = MotionState.UNKNOWN
        self._current_gear = None
        self._quit_flag = False
        self._last_error = None
        # self._imu_state = sensor_msgs.Imu()

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
            and time.time() - start < self._timeout_sec
        ):
            logger.info(f"Waiting for autoware localization...")
            time.sleep(0.1)

        # Check if localization is ready
        if self._vehicle_state != autoware_system_msgs.AutowareState.WAITING_FOR_ROUTE:
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
            self._call_set_route_points(sps, self._autoware_cfg.get("runtime", {}))
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

        if (
            self._vehicle_state != autoware_system_msgs.AutowareState.WAITING_FOR_ENGAGE
            and self._vehicle_state != autoware_system_msgs.AutowareState.DRIVING
        ):
            logger.warning(
                f"Autoware not in driving mode, current state: {self._vehicle_state}"
            )
            return Ctrl(mode=CtrlMode.None_)

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

        ego = obs[0]
        if ego is not None:
            cur_kinematic = ego.kinematic
            cur_kinematic.time = self._current_ros_time
            self._update_kinematic(cur_kinematic)
        else:
            logger.debug("AutowareAV.step called without 'ego' state in obs")

        self._agents = obs[1:] if len(obs) > 1 else []

        # publish
        # if self._sim_time_stamp - self._last_heavy_data_time >= 0.01:
        self._publish_tf()
        self._publish_control_mode()
        self._publish_gear_report()
        self._publish_steering_report()
        self._publish_velocity_report()
        self._publish_ego_state()
        self._publish_dynamic_objects()
        self._publish_occupancy_grid()
        self._publish_dummy_pointcloud()
        # self._publish_occupancy_grid()
        # self._publish_dummy_pointcloud()
        self._last_heavy_data_time = self._sim_time_stamp

        self._publish_clock(self._current_ros_time)
        # ros2 topic pub -r 50 /clock rosgraph_msgs/msg/Clock "{clock: 'now'}"

        # wait for new control message
        last_stamp = self._latest_control.stamp
        last_second = last_stamp.sec + last_stamp.nanosec * 1e-9

        if self._current_ros_time - last_second > 0.3:
            wait_time = max(self._control_timeout_sec, float(0.001))
            deadline = time.time() + wait_time
            while time.time() < deadline:
                if (
                    self._latest_control_stamp is not None
                    and self._latest_control_stamp != last_stamp
                ):
                    break
                time.sleep(0.001)

        if self._latest_control is None:
            logger.warning(
                "No control message received from Autoware, returning zero Ctrl"
            )
            return Ctrl(mode=CtrlMode.None_)

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

        # scenario 執行完成（粗略判準）
        # if self._ever_moving and self._motion_state == MotionState.STOPPED:
        #     logger.info("Scenario execution completed: vehicle stopped after moving.")
        #     return True

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
        duration = self._dt if self._dt > 0 else 0.01  # default 100Hz
        self._node.create_timer(duration, self._timer_callback)

        # QoS Profile
        # qos_profile = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,  # 關鍵：允許掉包，相容 Autoware 的設定
        #     durability=DurabilityPolicy.VOLATILE,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=10,
        # )
        qos_profile = 10

        # publishers
        # self._pub_initialpose3d = self._node.create_publisher(
        #     geometry_msgs.PoseWithCovarianceStamped,
        #     "/initialpose3d",
        #     qos_profile,
        # )

        # self._init_state_pub = self._node.create_publisher(
        #     LocalizationInitializationState,
        #     "/localization/initialization_state",
        #     qos_profile,
        # )

        self._kinematic_state_pub = self._node.create_publisher(
            nav_msgs.Odometry,
            "/localization/kinematic_state",
            qos_profile,
        )

        self._accel_pub = self._node.create_publisher(
            geometry_msgs.AccelWithCovarianceStamped,
            "/localization/acceleration",
            qos_profile,
        )

        self._objects_pub = self._node.create_publisher(
            autoware_perception_msgs.DetectedObjects,
            "/perception/object_recognition/detection/objects",
            1,
        )

        self._dummy_pointcloud_pub = self._node.create_publisher(
            sensor_msgs.PointCloud2,
            "/perception/obstacle_segmentation/pointcloud",
            qos_profile,
        )
        self._control_mode_pub = self._node.create_publisher(
            autoware_vehicle_msgs.ControlModeReport,
            "/vehicle/status/control_mode",
            qos_profile,
        )

        self._gear_report_pub = self._node.create_publisher(
            autoware_vehicle_msgs.GearReport,
            "/vehicle/status/gear_status",
            qos_profile,
        )

        self._steering_report_pub = self._node.create_publisher(
            autoware_vehicle_msgs.SteeringReport,
            "/vehicle/status/steering_status",
            qos_profile,
        )

        self._velocity_report_pub = self._node.create_publisher(
            autoware_vehicle_msgs.VelocityReport,
            "/vehicle/status/velocity_status",
            qos_profile,
        )

        self._occupancy_grid_pub = self._node.create_publisher(
            nav_msgs.OccupancyGrid,
            "/perception/occupancy_grid_map/map",
            qos_profile,
        )

        # self._imu_pub = self._node.create_publisher(
        #     sensor_msgs.Imu,
        #     "/sensing/imu/imu_data",
        #     qos_profile,
        # )

        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.durability = QoSDurabilityPolicy.VOLATILE
        self._clock_pub = self._node.create_publisher(
            rosgraph_msgs.Clock,
            "/clock",
            qos,
        )
        self._tf_broadcaster = TransformBroadcaster(self._node)

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
            map_path:={self._map_base_path} \
            lanelet2_map_file:={self._lanelet2_map_file} \
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
            self._base_time += self._dt
            self._current_ros_time = self._base_time
            self._kinematic.time = self._current_ros_time
            self._update_kinematic(self._kinematic)
            # if self._current_ros_time - self._last_heavy_data_time >= 0.01:
            self._publish_tf()
            self._publish_control_mode()
            self._publish_gear_report()
            self._publish_steering_report()
            self._publish_velocity_report()
            self._publish_ego_state()
            self._publish_dynamic_objects()
            self._publish_occupancy_grid()
            self._publish_dummy_pointcloud()
            # self._publish_occupancy_grid()
            # self._publish_imu()
            # self._publish_dummy_pointcloud()
            self._publish_clock(self._current_ros_time)

    def _on_control(self, msg: autoware_control_msgs.Control) -> None:
        self._latest_control = msg
        self._latest_control_stamp = msg.stamp

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

        ipos = sps.ego.spawn.position
        ispeed = sps.ego.spawn.speed

        t = geometry_msgs.TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = float(ipos.x)
        t.transform.translation.y = float(ipos.y)
        t.transform.translation.z = float(ipos.z)

        # Euler angle to quaternion
        qz, qw = self._yaw_to_quat(ipos.h)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw


        req = autoware_adapi_v1_msgs_srv.InitializeLocalization.Request()
        pose_msg = geometry_msgs.PoseWithCovarianceStamped()
        # pose_msg.header.stamp = self._node.get_clock().now().to_msg()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "map"

        pose_msg.pose.pose.position.x = float(ipos.x)
        pose_msg.pose.pose.position.y = float(ipos.y)
        pose_msg.pose.pose.position.z = float(ipos.z)
        logger.info(
            f"Setting initial position: x={ipos.x}, y={ipos.y}, z={ipos.z}, h={ipos.h}, speed={ispeed}"
        )
        qz, qw = self._yaw_to_quat(ipos.h)
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

    def _call_set_route_points(self, sps: ScenarioPack, params: Dict[str, Any]) -> None:
        assert self._node is not None

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
            params.get("allow_goal_modification", False)
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
    def _publish_ego_state(self) -> None:
        assert self._node is not None

        # now = self._node.get_clock().now().to_msg()
        now = self._convert_float_to_ros_time(self._current_ros_time).to_msg()

        ego = self._kinematic

        # ODOMETRY
        msg = nav_msgs.Odometry()
        msg.header.stamp = now
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = ego.x
        msg.pose.pose.position.y = ego.y
        msg.pose.pose.position.z = ego.z

        # yaw = float(ego.get("yaw", 0.0))
        yaw = self._kinematic.yaw
        qz, qw = self._yaw_to_quat(yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = self._kinematic.speed

        self._kinematic_state_pub.publish(msg)

        # ACC
        accel = geometry_msgs.AccelWithCovarianceStamped()
        accel.header.stamp = now
        accel.header.frame_id = "base_link"
        accel.accel.accel.linear = self._imu_state.linear_acceleration
        accel.accel.accel.angular = self._imu_state.angular_velocity
        self._accel_pub.publish(accel)

    def _publish_dynamic_objects(self) -> None:
        msg = autoware_perception_msgs.DetectedObjects()
        # msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.stamp = self._convert_float_to_ros_time(
            self._current_ros_time
        ).to_msg()
        msg.header.frame_id = "map"
        try:
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
                # print(obj.shape)
                # 4. Kinematics
                kin = autoware_perception_msgs.DetectedObjectKinematics()

                kin.orientation_availability = (
                    2  # (0:UNAVAILABLE, 1:SIGN_UNKNOWN, 2:AVAILABLE)
                )
                kin.has_position_covariance = False

                # Pose
                kin.pose_with_covariance.pose.position.x = ag.kinematic.x
                kin.pose_with_covariance.pose.position.y = ag.kinematic.y
                kin.pose_with_covariance.pose.position.z = ag.kinematic.z

                # qz, qw = self._yaw_to_quat(float(ag.get("yaw", 0.0)))
                qz, qw = self._yaw_to_quat(ag.kinematic.yaw)
                kin.pose_with_covariance.pose.orientation.z = qz
                kin.pose_with_covariance.pose.orientation.w = qw

                # Twist
                kin.has_twist = True
                # TODO: Agent's twist calculation
                kin.has_twist_covariance = False
                agent_speed = ag.kinematic.speed
                kin.twist_with_covariance.twist.linear.x = agent_speed

                # 賦值
                obj.kinematics = kin

                # 加入列表
                msg.objects.append(obj)
        except Exception as e:
            logger.error(f"Error publishing dynamic objects: {e}")
        self._objects_pub.publish(msg)

    def _publish_dummy_pointcloud(self) -> None:
        # Empty PointCloud2
        msg = sensor_msgs.PointCloud2()
        # msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.stamp = self._convert_float_to_ros_time(
            self._current_ros_time
        ).to_msg()
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

    # def _publish_initialization_state(self, state: int = None) -> None:
    #     msg = LocalizationInitializationState()

    #     # 這裡有兩個重點欄位：
    #     # 1. stamp: 當下時間
    #     msg.stamp = self._node.get_clock().now().to_msg()

    #     # 2. state: 告訴它已經初始化完成 (3 = INITIALIZED)
    #     # 定義在: autoware_adapi_v1_msgs/msg/LocalizationInitializationState.msg
    #     # UNKNOWN = 0, UNINITIALIZED = 1, INITIALIZING = 2, INITIALIZED = 3
    #     # msg.state = LocalizationInitializationState.INITIALIZED
    #     msg.state = state or LocalizationInitializationState.INITIALIZED
    #     logger.info("Publishing LocalizationInitializationState: INITIALIZED")
    #     self._init_state_pub.publish(msg)

    def _publish_tf(self) -> None:
        if self._kinematic is None:
            logger.warning("No kinematic state skipping TF publish")
            return

        # now = self._node.get_clock().now().to_msg()
        now = self._convert_float_to_ros_time(self._current_ros_time).to_msg()

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

        # 發送 TF
        self._tf_broadcaster.sendTransform(t)

    def _publish_control_mode(self) -> None:
        msg = autoware_vehicle_msgs.ControlModeReport()
        # msg.stamp = self._node.get_clock().now().to_msg()
        msg.stamp = self._convert_float_to_ros_time(self._current_ros_time).to_msg()
        msg.mode = self._control_mode
        self._control_mode_pub.publish(msg)

    def _publish_gear_report(self) -> None:
        msg = autoware_vehicle_msgs.GearReport()
        # msg.stamp = self._node.get_clock().now().to_msg()
        msg.stamp = self._convert_float_to_ros_time(self._current_ros_time).to_msg()
        if self._current_gear is None:
            logger.warning("No gear command received, defaulting to DRIVE")
            self._current_gear = autoware_vehicle_msgs.GearCommand.DRIVE
        msg.report = self._current_gear
        self._gear_report_pub.publish(msg)

    def _publish_steering_report(self) -> None:
        msg = autoware_vehicle_msgs.SteeringReport()
        # msg.stamp = self._node.get_clock().now().to_msg()
        msg.stamp = self._convert_float_to_ros_time(self._current_ros_time).to_msg()
        msg.steering_tire_angle = self._latest_control.lateral.steering_tire_angle
        self._steering_report_pub.publish(msg)

    def _publish_velocity_report(self) -> None:
        msg = autoware_vehicle_msgs.VelocityReport()
        # msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.stamp = self._convert_float_to_ros_time(
            self._current_ros_time
        ).to_msg()
        msg.header.frame_id = "base_link"
        msg.longitudinal_velocity = self._kinematic.speed
        msg.lateral_velocity = 0.0
        # TODO: 直接從sim拿 heading rate
        msg.heading_rate = self._imu_state.angular_velocity.z

        self._velocity_report_pub.publish(msg)

    def _publish_occupancy_grid(self) -> None:
        msg = nav_msgs.OccupancyGrid()
        # msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.stamp = self._convert_float_to_ros_time(
            self._current_ros_time
        ).to_msg()
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

    # def _publish_imu(self) -> None:
    #     self._imu_pub.publish(self._imu_state)

    def _publish_clock(self, t: float) -> None:
        msg = rosgraph_msgs.Clock()
        time = self._convert_float_to_ros_time(t)
        msg.clock = time.to_msg()
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

    def _resolve_map_path(self, sps: ScenarioPack) -> Path:
        """
        根據 ScenarioPack / cfg 決定 map_path:
        1) 從sps.maps取得.osm file path (若有)
        2) 拆解成絕對路徑(map_path + lanelet2_map_file)
        """
        full_path = Path(sps.maps.get("osm"))
        if not full_path.exists():
            raise FileNotFoundError(f"Autoware map file not found: {full_path}")

        if full_path.suffix.lower() != ".osm":
            raise ValueError(f"Autoware map file must be .osm format, got: {full_path}")

        self._map_base_path = full_path.parent
        self._lanelet2_map_file = full_path.name

        return full_path

    def _calc_imu_state(self):
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
        """回傳 (z, w)，假設 roll=pitch=0"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return sy, cy

    @staticmethod
    def _quat_to_yaw(q) -> float:
        """假設 roll=pitch=0，從 quaternion 回傳 yaw"""
        return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
