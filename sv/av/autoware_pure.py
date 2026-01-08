from __future__ import annotations

import math
import os
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import (
    Vector3,
    Pose,
    PoseWithCovarianceStamped,
    TransformStamped,
    AccelWithCovarianceStamped,
)

# tier4_perception_msgs/msg/DetectedObjectsWithFeature
from tier4_perception_msgs.msg import DetectedObjectsWithFeature

from autoware_system_msgs.msg import AutowareState
from autoware_control_msgs.msg import Control
from autoware_adapi_v1_msgs.srv import (
    InitializeLocalization,
    SetRoutePoints,
    ChangeOperationMode,
)
from autoware_adapi_v1_msgs.msg import (
    MotionState,
    LocalizationInitializationState,
    VehicleKinematics,
)
from autoware_vehicle_msgs.msg import (
    ControlModeReport,
    GearReport,
    SteeringReport,
    VelocityReport,
)
from sensor_msgs.msg import Imu, PointCloud2, PointField
from unique_identifier_msgs.msg import UUID

try:
    from autoware_perception_msgs.msg import (
        PredictedObjects,
        PredictedObject,
        PredictedObjectKinematics,
        Shape,
        ObjectClassification,
        TrackedObject,
        TrackedObjects,
        TrackedObjectKinematics,
        DetectedObjects,
        DetectedObject,
        DetectedObjectKinematics,
    )

    # HAVE_PERCEPTION_MSGS = True
except ImportError:
    PredictedObjects = PredictedObject = PredictedObjectKinematics = Shape = ObjectClassification = None  # type: ignore
    # HAVE_PERCEPTION_MSGS = False

from sv.utils.position import Position
from sv.utils.util import get_cfg
from sv.utils.kinematic import VehicleKinematic
from sv.registry import register_av
from sv.utils.control import Ctrl, CtrlMode
from sv.utils.sps import ScenarioPack

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

    _rclpy_inited: bool = False
    _instance_count: int = 0

    def __init__(self, cfg_path: Path, sps: Any):
        self.cfg = get_cfg(Path(cfg_path))
        self._aw_cfg = self.cfg["autoware"]

        # —— Autoware 啟動相關設定 ——
        self._root = Path(self._aw_cfg.get("root", "/autoware"))
        self._ros_setup_script = self._aw_cfg.get(
            "ros_setup_script", "/opt/ros/humble/setup.bash"
        )

        launch_cfg = self._aw_cfg.get("launch", {})
        self._launch_pkg = launch_cfg.get("package", "autoware_launch")
        self._launch_file = launch_cfg.get("file", "planning_simulator.launch.xml")
        # self._launch_file = launch_cfg.get("file", "sbsvf.launch.xml")
        self._headless = bool(launch_cfg.get("headless", True))
        self._extra_launch_args: List[str] = list(launch_cfg.get("extra_args", []))

        data_cfg = self._aw_cfg.get("data", {})
        self._map_base_path = Path(data_cfg.get("map_base_path", "/autoware_map"))
        self._default_map_subdir = data_cfg.get(
            "default_map_subdir", "sample-map-planning"
        )
        self._data_path = Path(data_cfg.get("data_path", "/autoware_data"))

        veh_cfg = self._aw_cfg.get("vehicle", {})
        self._vehicle_model = veh_cfg.get("model", "sample_vehicle")
        self._sensor_model = veh_cfg.get("sensor_model", "sample_sensor_kit")

        rt_cfg = self._aw_cfg.get("runtime", {})
        self._service_wait_timeout = float(rt_cfg.get("service_wait_timeout_sec", 30.0))
        self._control_timeout = float(rt_cfg.get("control_timeout_sec", 0.1))
        self._spin_rate_hz = float(rt_cfg.get("spin_rate_hz", 160.0))
        # self._publish_objects = bool(rt_cfg.get("publish_objects", True))
        # self._use_dynamic_objects_topic = bool(
        #     rt_cfg.get("use_dynamic_objects_topic", True)
        # )

        # 初始 ScenarioPack（主要用來在 init() 時決定第一個 map）
        self._current_sps: Optional[ScenarioPack] = sps
        self._current_map_path: Optional[Path] = None
        self._base_map_path: Optional[Path] = None
        self._lanelet2_map_file: Optional[Path] = None

        self._uuid_map: Dict[str, UUID] = {}

        # —— Autoware process & ROS node 狀態 ——
        self._autoware_proc: Optional[subprocess.Popen] = None
        self._node: Optional[Node] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None

        # pub / sub / services
        self._kinematic_state_pub = None
        self._objects_pub = None
        self._control_sub = None
        self._motion_state_sub = None

        self._cli_init_loc = None
        self._cli_set_route_points = None
        self._cli_change_to_auto = None

        # 狀態
        self._latest_control: Control = Control()
        self._latest_control_stamp = None  # builtin_interfaces/Time or類似
        self._kinematic: VehicleKinematic = VehicleKinematic()
        self._past_kinematic: VehicleKinematic = VehicleKinematic()
        self._past_past_kinematic: VehicleKinematic = VehicleKinematic()
        # self._kinematic_state_from_aw = None
        # self._kinematic_state_from_aw.orientation.w = (
        #     1.0  # default to a valid quaternion
        # )
        self._imu_state = Imu()
        self._motion_state: int = MotionState.UNKNOWN
        self._ever_moving: bool = False
        self._quit_flag: bool = False
        self._last_error: Optional[str] = None
        self._vehicle_state: Optional[int] = None
        self._agents: List[Any] = []
        # AutowarePureAV._instance_count += 1

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def init(self, sps) -> None:
        """
        - 啟 ROS node + spin thread
        - 啟動 Autoware launch (subprocess)
        - 等待 AD API services ready
        """
        rclpy.init()

        self._ensure_ros_node()
        # 用目前的 Scenario 決定 map_path
        if self._current_sps is None:
            raise RuntimeError(
                "AutowareAV.init() 需要先有 ScenarioPack 來決定 map_path"
            )

        map_path = self._resolve_map_path(self._current_sps)
        self._current_map_path = map_path
        # input("press enter to launch autoware with map: " + str(map_path))
        self._launch_autoware()

        # 等待 AD API services ready
        self._wait_for_service(self._cli_init_loc, "InitializeLocalization")
        self._wait_for_service(self._cli_set_route_points, "SetRoutePoints")
        self._wait_for_service(self._cli_change_to_auto, "ChangeOperationMode")

        logger.info("Autoware AV initialized and Autoware stack is ready.")

    def reset(self, sps: ScenarioPack, params: Optional[dict] = None) -> None:
        """
        Reset AV internal state when simulator resets.

        做兩件事：
        1. 如有換 map，就重啟 Autoware
        2. 用 AD API 設 initial pose / route / autonomous
        """
        params = params or {}
        self._ensure_ros_node()
        logger.info(f"INITNTINTINTINTITNITNTIT autowre state: {self._vehicle_state}")

        self._current_sps = sps

        # 如果 map 有變，重啟 Autoware process
        new_map_path = self._resolve_map_path(sps)
        if self._current_map_path is None or new_map_path != self._current_map_path:
            logger.info(
                f"Scenario uses new map_path={new_map_path}, restarting Autoware..."
            )
            self._stop_autoware_process()
            self._launch_autoware()
            self._current_map_path = new_map_path

            self._wait_for_service(self._cli_init_loc, "InitializeLocalization")
            self._wait_for_service(self._cli_set_route_points, "SetRoutePoints")
            self._wait_for_service(self._cli_change_to_auto, "ChangeOperationMode")

        # 清 internal state
        self._latest_control = Control()
        self._latest_control_stamp = None
        self._motion_state = MotionState.UNKNOWN
        self._ever_moving = False
        self._quit_flag = False
        self._last_error = None
        self._kinematic = VehicleKinematic()
        self._past_kinematic = VehicleKinematic()
        self._past_past_kinematic = VehicleKinematic()

        ipos = sps.ego.spawn.position
        ispeed = sps.ego.spawn.speed
        logger.info("autoware state: before init loc: %s", self._vehicle_state)
        # publish tf, odom, initializa state for initial pose
        # self._publish_tf(ego_init)
        # self._publish_initialization_state(1)

        # input("set init position, press enter")
        init_kinematic = VehicleKinematic()
        init_kinematic.time = float(self._node.get_clock().now().nanoseconds) * 1e-9
        init_kinematic.x = ipos.x
        init_kinematic.y = ipos.y
        init_kinematic.z = ipos.z
        init_kinematic.yaw = ipos.h
        init_kinematic.speed = 0.0  # Autoware state change condition
        self._update_kinematic(init_kinematic)

        # self._kinematic.time = float(
        #     self._node.get_clock().now().seconds_nanoseconds()[0]
        # )
        # self._kinematic.x = ipos.x
        # self._kinematic.y = ipos.y
        # self._kinematic.z = ipos.z
        # self._kinematic.yaw = ipos.h
        # # self._kinematic.p = ipos.p
        # # self._kinematic.r = ipos.r
        # # self._kinematic.speed = ispeed
        # self._kinematic.speed = 0.0

        # self._publish_initialization_state(2)
        # self._publish_ego_state()

        # 1) localization
        logger.info("Autoware state: before init loc: %s", self._vehicle_state)
        self._call_initialize_localization(sps)
        # self._publish_dynamic_objects()
        # self._publish_initial_pose()
        # self._publish_initialization_state(3)
        while self._vehicle_state != AutowareState.WAITING_FOR_ROUTE:
            logger.info(
                f"Waiting for autoware localization, autoware state: {self._vehicle_state}"
            )
            # command = input("press '1' to retry, '2' to skip wait: ")
            # if command.strip() == "2":
            #     break
            # elif command.strip() == "1":
            #     # self._call_initialize_localization(sps)
            #     self._publish_initial_pose()
            time.sleep(0.1)
        # # 2) routing
        # input("set route points, press enter")
        self._call_set_route_points(sps, params)
        time.sleep(0.5)
        logger.info("Autoware state: after set route: %s", self._vehicle_state)
        dead_line = time.time() + 20.0
        while (
            self._vehicle_state != AutowareState.WAITING_FOR_ENGAGE
            and time.time() < dead_line
        ):
            logger.info(
                f"Waiting for autoware planning, autoware state: {self._vehicle_state}"
            )
            time.sleep(0.1)

        logger.info("Autoware planning ready. Changing to autonomous mode...")
        # # 3) mode -> autonomous
        input("change to autonomous mode, press enter")
        self._call_change_to_autonomous()
        # while self._vehicle_state != AutowareState.DRIVING:
        #     logger.info(
        #         f"Waiting for autoware to enter autonomous mode, autoware state: {self._vehicle_state}"
        #     )
        #     time.sleep(0.1)
        logger.info("Autoware AV reset completed.")

    def step(self, obs: Dict[str, Any], dt: float) -> Ctrl:
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
        ego = obs.get("ego", None)
        if ego is not None:
            cur_kinematic = VehicleKinematic.from_dict(ego)
            cur_kinematic.time = float(self._node.get_clock().now().nanoseconds) * 1e-9
            self._update_kinematic(cur_kinematic)
            # self._publish_ego_state(ego)

        else:
            logger.debug("AutowareAV.step called without 'ego' state in obs")

        # if self._publish_objects and self._use_dynamic_objects_topic:
        self._agents = obs.get("agents", [])
        # self._publish_dynamic_objects()

        # # 等一筆「新」的 control，最多 control_timeout_sec 或 dt，取較大的以避免太頻繁 timeout
        wait_time = max(self._control_timeout, float(dt))
        deadline = time.time() + wait_time
        last_stamp = self._latest_control.stamp

        while time.time() < deadline:
            if (
                self._latest_control_stamp is not None
                and self._latest_control_stamp != last_stamp
            ):
                break
            time.sleep(0.001)

        ctrl_msg = self._latest_control
        if ctrl_msg is None:
            # 尚未收到控制，先回一個只帶 AUTO mode 的 Ctrl
            return Ctrl(mode=CtrlMode.None_)

        # 轉成 Ctrl
        try:
            speed = float(ctrl_msg.longitudinal.velocity)
        except AttributeError:
            logger.info(
                "Control message missing longitudinal.velocity defaulting to 1.0"
            )
            speed = 1.0
        try:
            steering = float(ctrl_msg.lateral.steering_tire_angle)
        except AttributeError:
            logger.info(
                "Control message missing lateral.steering_tire_angle defaulting to 0.0"
            )
            steering = 0.0
        # logger.info(f"Received control: speed={speed}, steering={steering}")
        # return Ctrl(
        #     mode=CtrlMode.THROTTLE_STEER,
        #     payload={
        #         "pedal": 1,
        #         "wheel": 0,
        #     },
        # )
        return Ctrl(
            mode=CtrlMode.VEL_STEER,
            payload={
                "speed": speed,
                "h": steering,
            },
        )

        # if self._kinematic_state_from_aw is None:
        #     logger.warning(
        #         "AutowareAV.step: no kinematic state from Autoware returning zero Ctrl"
        #     )
        #     return Ctrl(mode=CtrlMode.None_)

        # cur_x = self._kinematic_state_from_aw.pose.pose.pose.position.x
        # cur_y = self._kinematic_state_from_aw.pose.pose.pose.position.y
        # cur_h = self._quat_to_yaw(
        #     self._kinematic_state_from_aw.pose.pose.pose.orientation
        # )
        # return Ctrl(
        #     mode=CtrlMode.POSITION,
        #     payload={
        #         "x": float(cur_x),
        #         "y": float(cur_y),
        #         "h": float(cur_h),
        #     },
        # )

    def stop(self) -> None:
        """關閉 Autoware process + ROS node / executor"""
        self._stop_autoware_process()

        if self._executor and self._node:
            self._executor.remove_node(self._node)

        if self._node:
            self._node.destroy_node()
            self._node = None

        # AutowareAV._instance_count -= 1
        # if AutowareAV._instance_count <= 0 and AutowareAV._rclpy_inited:
        #     rclpy.shutdown()
        #     AutowareAV._rclpy_inited = False

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
        # if not AutowareAV._rclpy_inited:
        #     rclpy.init()
        #     AutowareAV._rclpy_inited = True

        if self._node is not None:
            return

        self._node = rclpy.create_node("autoware_av_adapter")
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._node.create_timer(0.02, self._timer_callback)

        # 定義一個相容的 QoS Profile
        # qos_profile = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,  # 關鍵：允許掉包，相容 Autoware 的設定
        #     durability=DurabilityPolicy.VOLATILE,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=10,
        # )
        qos_profile = 10

        # publishers
        self._pub_initialpose3d = self._node.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose3d",
            qos_profile,
        )

        # self._init_state_pub = self._node.create_publisher(
        #     LocalizationInitializationState,
        #     "/localization/initialization_state",
        #     qos_profile,
        # )

        self._kinematic_state_pub = self._node.create_publisher(
            Odometry,
            "/localization/kinematic_state",
            qos_profile,
        )

        self._accel_pub = self._node.create_publisher(
            AccelWithCovarianceStamped,
            "/localization/acceleration",
            qos_profile,
        )

        # HAVE_PERCEPTION_MSGS = True
        # if HAVE_PERCEPTION_MSGS:  # and self._use_dynamic_objects_topic:
        #     self._objects_pub = self._node.create_publisher(
        #         PredictedObjects,
        #         "/perception/object_recognition/objects",
        #         10,
        #     )
        # else:
        #     self._objects_pub = None
        #     if self._use_dynamic_objects_topic:
        #         self._node.get_logger().warn(
        #             "autoware_perception_msgs not available or disabled dynamic objects not published."
        #         )

        # self._objects_pub = self._node.create_publisher(
        #     TrackedObjects, "/perception/object_recognition/tracking/objects", 10
        # )

        self._objects_pub = self._node.create_publisher(
            DetectedObjects,
            "/perception/object_recognition/detection/objects",
            1,
        )

        # self._object_with_feature_pub = self._node.create_publisher(
        #     DetectedObjectsWithFeature,
        #     "/perception/object_recognition/detection/labeled_clusters",
        #     1,
        # )

        self._dummy_pointcloud_pub = self._node.create_publisher(
            PointCloud2,
            "/perception/obstacle_segmentation/pointcloud",
            qos_profile,
        )
        self._control_mode_pub = self._node.create_publisher(
            ControlModeReport,
            "/vehicle/status/control_mode",
            qos_profile,
        )

        self._gear_report_pub = self._node.create_publisher(
            GearReport,
            "/vehicle/status/gear_status",
            qos_profile,
        )

        self._steering_report_pub = self._node.create_publisher(
            SteeringReport,
            "/vehicle/status/steering_status",
            qos_profile,
        )

        self._velocity_report_pub = self._node.create_publisher(
            VelocityReport,
            "/vehicle/status/velocity_status",
            qos_profile,
        )

        self._occupancy_grid_pub = self._node.create_publisher(
            OccupancyGrid,
            "/perception/occupancy_grid_map/map",
            qos_profile,
        )

        self._imu_pub = self._node.create_publisher(
            Imu,
            "/sensing/imu/imu_data",
            qos_profile,
        )

        self._tf_broadcaster = TransformBroadcaster(self._node)

        # subscribers
        self._control_sub = self._node.create_subscription(
            Control,
            "/control/command/control_cmd",
            self._on_control,
            qos_profile,
        )
        self._motion_state_sub = self._node.create_subscription(
            MotionState,
            "/api/motion/state",
            self._on_motion_state,
            qos_profile,
        )
        self._vehicle_state_sub = self._node.create_subscription(
            AutowareState,
            "/autoware/state",
            self._on_autoware_state,
            qos_profile,
        )
        # self._kinematic_state_sub = self._node.create_subscription(
        #     VehicleKinematics,
        #     "/api/vehicle/kinematics",
        #     self._on_kinematic_state,
        #     QoSProfile(
        #         reliability=ReliabilityPolicy.BEST_EFFORT,  # 關鍵：允許掉包，相容 Autoware 的設定
        #         durability=DurabilityPolicy.VOLATILE,
        #         history=HistoryPolicy.KEEP_LAST,
        #         depth=10,
        #     ),
        # )

        # services
        self._cli_init_loc = self._node.create_client(
            InitializeLocalization,
            "/api/localization/initialize",
        )
        self._cli_set_route_points = self._node.create_client(
            SetRoutePoints,
            "/api/routing/set_route_points",
        )
        self._cli_change_to_auto = self._node.create_client(
            ChangeOperationMode,
            "/api/operation_mode/change_to_autonomous",
        )

        # spin thread
        self._spin_thread = threading.Thread(target=self._spin, daemon=False)
        self._spin_thread.start()

    def _spin(self) -> None:
        assert self._executor is not None
        period = 1.0 / self._spin_rate_hz if self._spin_rate_hz > 0 else 0.01
        while rclpy.ok() and self._node is not None:
            try:
                self._executor.spin_once(timeout_sec=period)
            except Exception as e:  # noqa: BLE001
                logger.error(f"AutowareAV executor error: {e}")
                self._last_error = str(e)
                self._quit_flag = True
                break

    def _launch_autoware(self) -> None:
        # launch_parts = [
        #     f"cd {self._root}",
        #     f"source {self._ros_setup_script}",
        #     f"""ros2 launch {self._launch_pkg} {self._launch_file} \
        #     map_path:={self._base_map_path} \
        #     lanelet2_map_file:={self._lanelet2_map_file} \
        #     data_path:={self._data_path} \
        #     vehicle_model:={self._vehicle_model} \
        #     sensor_model:={self._sensor_model} \
        #     """,
        # ]

        launch_parts = [
            f"cd {self._root}",
            f"source {self._ros_setup_script}",
            f"""ros2 launch {self._launch_pkg} {self._launch_file} \
            map_path:={self._base_map_path} \
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
            """,
        ]
        launch_parts.extend(self._extra_launch_args)

        full_cmd = " && ".join(launch_parts)
        logger.info(f"Launching Autoware: {full_cmd}")
        log = open("/tmp/autoware_launch.log", "ab", buffering=0)
        print(full_cmd)
        self._autoware_proc = subprocess.Popen(
            ["bash", "-lc", full_cmd],
            stdout=log,
            stderr=log,
            preexec_fn=os.setsid,
        )
        # self._autoware_proc = subprocess.Popen(
        #     ["bash", "-c", full_cmd],
        #     preexec_fn=os.setsid,
        # )

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
        self._publish_tf()
        self._publish_control_mode()
        self._publish_gear_report()
        self._publish_steering_report()
        self._publish_velocity_report()
        self._publish_ego_state()
        self._publish_dynamic_objects()
        self._publish_occupancy_grid()
        self._publish_imu()
        self._publish_dummy_pointcloud()

    def _on_control(self, msg: Control) -> None:
        self._latest_control = msg
        # Control message 可能有 stamp 或 header.stamp，視實際 msg 定義調整
        stamp = getattr(msg, "stamp", None)
        if stamp is None and hasattr(msg, "header"):
            stamp = msg.header.stamp
        self._latest_control_stamp = stamp

    def _on_motion_state(self, msg: MotionState) -> None:
        self._motion_state = msg.state
        if msg.state == MotionState.MOVING:
            self._ever_moving = True

    def _on_autoware_state(self, msg: AutowareState) -> None:
        self._vehicle_state = msg.state

    def _on_kinematic_state(self, msg: VehicleKinematics) -> None:
        self._kinematic_state_from_aw = msg

    # ------------------------------------------------------------------
    # AD API calls
    # ------------------------------------------------------------------
    def _wait_for_service(
        self, client, name: str, timeout_sec: Optional[float] = None
    ) -> None:
        timeout = timeout_sec or self._service_wait_timeout
        start = time.time()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() - start > timeout:
                msg = f"Service {name} not available after {timeout}s"
                logger.error(msg)
                self._last_error = msg
                self._quit_flag = True
                return
            logger.info(f"Waiting for Autoware service {name}...")
        logger.info(f"Service {name} is available.")

    def _call_initialize_localization(self, sps: ScenarioPack) -> None:
        assert self._node is not None
        now = self._node.get_clock().now().to_msg()

        ipos = sps.ego.spawn.position
        ispeed = sps.ego.spawn.speed

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = float(ipos.x)
        t.transform.translation.y = float(ipos.y)
        t.transform.translation.z = float(ipos.z)

        # 歐拉角轉四元數
        qz, qw = self._yaw_to_quat(ipos.h)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        # 發送 TF
        self._tf_broadcaster.sendTransform(t)

        req = InitializeLocalization.Request()
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self._node.get_clock().now().to_msg()
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

        fut = self._cli_init_loc.call_async(req)
        while rclpy.ok() and not fut.done():
            logger.info("Waiting for InitializeLocalization response...")
            time.sleep(0.01)
        res = fut.result()
        if res is None or not res.status.success:
            msg = f"InitializeLocalization failed: {getattr(res.status, 'message', 'unknown') if res else 'no response'}"
            code = getattr(res.status, "code", "unknown") if res else "no response"
            succ = getattr(res.status, "success", "unknown") if res else "no response"
            logger.info(f"InitializeLocalization response: code={code}, success={succ}")
            logger.error(msg)
            self._last_error = msg
            self._quit_flag = True

        logger.info("Localization initialized.")

    def _call_set_route_points(self, sps: ScenarioPack, params: Dict[str, Any]) -> None:
        assert self._node is not None

        req = SetRoutePoints.Request()
        req.header.frame_id = "map"
        req.header.stamp = self._node.get_clock().now().to_msg()

        gp = sps.ego.goal.position
        goal = Pose()
        goal.position.x = float(gp.x)
        goal.position.y = float(gp.y)
        goal.position.z = float(gp.z)

        qz, qw = self._yaw_to_quat(gp.h)
        goal.orientation.z = qz
        goal.orientation.w = qw

        req.goal = goal
        req.waypoints = []

        allow_mod = bool(params.get("allow_goal_modification", True))
        # RouteOption 欄位名稱依實際 msg 定義調整
        req.option.allow_goal_modification = allow_mod  # type: ignore[attr-defined]

        fut = self._cli_set_route_points.call_async(req)
        while rclpy.ok() and not fut.done():
            time.sleep(0.01)
        res = fut.result()
        if res is None or not res.status.success:
            msg = f"SetRoutePoints failed: {getattr(res.status, 'message', 'unknown') if res else 'no response'}"
            logger.error(msg)
            self._last_error = msg
            self._quit_flag = True

    def _call_change_to_autonomous(self) -> None:
        assert self._node is not None

        req = ChangeOperationMode.Request()
        fut = self._cli_change_to_auto.call_async(req)
        # rclpy.spin_until_future_complete(self._node, fut)
        while rclpy.ok() and not fut.done():
            time.sleep(0.01)
        res = fut.result()
        if res is None or not res.status.success:
            msg = f"ChangeOperationMode(AUTONOMOUS) failed: {getattr(res.status, 'message', 'unknown') if res else 'no response'}"
            logger.error(msg)
            self._last_error = msg
            self._quit_flag = True

    # ------------------------------------------------------------------
    # publish helpers
    # ------------------------------------------------------------------
    def _publish_ego_state(self) -> None:
        now = self._node.get_clock().now().to_msg()

        ego = self._kinematic.__dict__
        # ego = self._kinematic
        # TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = float(ego.get("x", 0.0))
        t.transform.translation.y = float(ego.get("y", 0.0))
        t.transform.translation.z = float(ego.get("z", 0.0))

        # 歐拉角轉四元數
        qz, qw = self._yaw_to_quat(ego.get("yaw", 0.0))
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        # 發送 TF
        # self._tf_broadcaster.sendTransform(t)

        # ODOMETRY
        if self._kinematic_state_pub is None:
            return
        assert self._node is not None

        msg = Odometry()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = float(ego.get("x", 0.0))
        msg.pose.pose.position.y = float(ego.get("y", 0.0))
        msg.pose.pose.position.z = float(ego.get("z", 0.0))

        # msg.pose.pose.position.x = ego.x
        # msg.pose.pose.position.y = ego.y
        # msg.pose.pose.position.z = ego.z

        yaw = float(ego.get("yaw", 0.0))
        qz, qw = self._yaw_to_quat(yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = float(ego.get("speed", 0.0))
        # msg.twist.twist.linear.x = self._latest_control.longitudinal.velocity

        self._kinematic_state_pub.publish(msg)
        # logger.info(
        #     f"Published ego state: x={ego.get('x', 0.0)}, y={ego.get('y', 0.0)}, speed={ego.get('speed', 0.0)}"
        # )

        # ACC
        accel = AccelWithCovarianceStamped()
        accel.header.stamp = now
        accel.header.frame_id = "base_link"  # 加速度通常是相對於車身的
        accel.accel.accel.linear = self._imu_state.linear_acceleration
        accel.accel.accel.angular = self._imu_state.angular_velocity
        self._accel_pub.publish(accel)

    def _publish_dynamic_objects(self) -> None:
        msg = DetectedObjects()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        for ag in self._agents:
            obj = DetectedObject()

            # 1. UUID (一樣要固定)
            # agent_id = ag.get("id", "unknown")
            # obj.object_id = self._get_stable_uuid(agent_id)

            # 2. 存在機率
            obj.existence_probability = 1.0

            # 3. 分類 (維持不變)
            clas = ObjectClassification()
            # clas.label = int(ag.get("type", ObjectClassification.CAR))
            # clas.label = ObjectClassification.MOTORCYCLE
            clas.label = ObjectClassification.CAR
            clas.probability = 1.0
            obj.classification = [clas]

            # 4. 外型 (維持不變，注意 X/Y 定義)
            shp = Shape()
            shp.type = Shape.BOUNDING_BOX
            shp.dimensions.x = float(ag.get("length", 4.0))
            shp.dimensions.y = float(ag.get("width", 2.0))
            shp.dimensions.z = float(ag.get("height", 1.6))
            obj.shape = shp

            # 5. 運動學 (變更為 TrackedObjectKinematics)
            # TrackedObjectKinematics 不需要 predicted_paths，只要現在狀態
            kin = DetectedObjectKinematics()

            # 填入 Pose
            kin.pose_with_covariance.pose.position.x = float(ag.get("x", 0.0))
            kin.pose_with_covariance.pose.position.y = float(ag.get("y", 0.0))
            kin.pose_with_covariance.pose.position.z = float(ag.get("z", 0.0))

            qz, qw = self._yaw_to_quat(float(ag.get("yaw", 0.0)))
            kin.pose_with_covariance.pose.orientation.z = qz
            kin.pose_with_covariance.pose.orientation.w = qw

            # 填入 Twist (速度)
            # 注意：這裡假設是縱向速度
            kin.twist_with_covariance.twist.linear.x = float(ag.get("speed", 0.0))

            # 賦值
            obj.kinematics = kin

            # 加入列表
            msg.objects.append(obj)

        self._objects_pub.publish(msg)

    # def _publish_objects_with_feature(self) -> None:
    #     msg = DetectedObjectsWithFeature()
    #     msg.header.stamp = self._node.get_clock().now().to_msg()
    #     self._object_with_feature_pub.publish(msg)

    def _publish_dummy_pointcloud(self) -> None:
        # Empty PointCloud2
        msg = PointCloud2()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.height = 1
        msg.width = 0
        msg.is_dense = True
        msg.is_bigendian = False
        x = PointField()
        x.name = "x"
        x.offset = 0
        x.datatype = PointField.FLOAT32
        x.count = 1
        y = PointField()
        y.name = "y"
        y.offset = 4
        y.datatype = PointField.FLOAT32
        y.count = 1
        z = PointField()
        z.name = "z"
        z.offset = 8
        z.datatype = PointField.FLOAT32
        z.count = 1
        intensity = PointField()
        intensity.name = "intensity"
        intensity.offset = 12
        intensity.datatype = PointField.UINT8
        intensity.count = 1
        returntype = PointField()
        returntype.name = "return_type"
        returntype.offset = 13
        returntype.datatype = PointField.UINT8
        returntype.count = 1
        channel = PointField()
        channel.name = "channel"
        channel.offset = 14
        channel.datatype = PointField.UINT16
        channel.count = 1
        msg.fields = [x, y, z, intensity, returntype, channel]
        msg.point_step = 16
        msg.row_step = 0
        msg.data = b""
        self._dummy_pointcloud_pub.publish(msg)

    def _publish_initialization_state(self, state: int = None) -> None:
        msg = LocalizationInitializationState()

        # 這裡有兩個重點欄位：
        # 1. stamp: 當下時間
        msg.stamp = self._node.get_clock().now().to_msg()

        # 2. state: 告訴它已經初始化完成 (3 = INITIALIZED)
        # 定義在: autoware_adapi_v1_msgs/msg/LocalizationInitializationState.msg
        # UNKNOWN = 0, UNINITIALIZED = 1, INITIALIZING = 2, INITIALIZED = 3
        # msg.state = LocalizationInitializationState.INITIALIZED
        msg.state = state or LocalizationInitializationState.INITIALIZED
        logger.info("Publishing LocalizationInitializationState: INITIALIZED")
        self._init_state_pub.publish(msg)

    def _publish_tf(self) -> None:
        if self._kinematic is None:
            logger.warning("No kinematic state skipping TF publish")
            return

        now = self._node.get_clock().now().to_msg()

        t = TransformStamped()
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

    def _publish_control_mode(self, mode: int = ControlModeReport.AUTONOMOUS) -> None:
        msg = ControlModeReport()
        msg.stamp = self._node.get_clock().now().to_msg()
        msg.mode = mode
        self._control_mode_pub.publish(msg)

    def _publish_gear_report(self, gear: int = GearReport.DRIVE) -> None:
        msg = GearReport()
        msg.stamp = self._node.get_clock().now().to_msg()
        msg.report = gear
        self._gear_report_pub.publish(msg)

    def _publish_steering_report(self) -> None:
        msg = SteeringReport()
        msg.stamp = self._node.get_clock().now().to_msg()
        msg.steering_tire_angle = self._latest_control.lateral.steering_tire_angle
        self._steering_report_pub.publish(msg)

    def _publish_velocity_report(self) -> None:
        msg = VelocityReport()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        prevx = self._past_kinematic.x if self._past_kinematic else 0.0
        prevy = self._past_kinematic.y if self._past_kinematic else 0.0
        prevt = self._past_kinematic.time if self._past_kinematic else 1.0
        curx = self._kinematic.x
        cury = self._kinematic.y
        curt = self._kinematic.time
        dt = curt - prevt if curt - prevt > 0.0 else 1.0
        vx = (curx - prevx) / dt
        vy = (cury - prevy) / dt
        # msg.lateral_velocity = (vx**2 + vy**2) ** 0.5
        msg.longitudinal_velocity = self._latest_control.longitudinal.velocity
        # msg.longitudinal_velocity = (vx**2 + vy**2) ** 0.5
        self._velocity_report_pub.publish(msg)

    # def _publish_initial_pose(self) -> None:
    #     assert self._node is not None

    #     ipos = self._current_sps.ego.spawn.position

    #     pose_msg = PoseWithCovarianceStamped()
    #     pose_msg.header.stamp = self._node.get_clock().now().to_msg()
    #     pose_msg.header.frame_id = "map"

    #     pose_msg.pose.pose.position.x = float(ipos.x)
    #     pose_msg.pose.pose.position.y = float(ipos.y)
    #     pose_msg.pose.pose.position.z = float(ipos.z)
    #     logger.info(
    #         f"Publishing initial pose: x={ipos.x}, y={ipos.y}, z={ipos.z}, h={ipos.h}"
    #     )
    #     qz, qw = self._yaw_to_quat(ipos.h)
    #     pose_msg.pose.pose.orientation.z = qz
    #     pose_msg.pose.pose.orientation.w = qw

    #     pose_msg.pose.covariance = [0.0] * 36

    #     self._pub_initialpose3d.publish(pose_msg)

    def _publish_occupancy_grid(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self._node.get_clock().now().to_msg()
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

    def _publish_imu(self) -> None:
        # if self._kinematic_state_from_aw is None:
        #     # logger.info("No kinematic state from Autoware skipping IMU publish")
        #     return
        # msg = Imu()
        # self._imu_pub.publish(msg)
        # return
        # msg.header.stamp = self._node.get_clock().now().to_msg()
        # msg.header.frame_id = "base_link"

        # msg.orientation.x = self._kinematic_state_from_aw.pose.pose.pose.orientation.x
        # msg.orientation.y = self._kinematic_state_from_aw.pose.pose.pose.orientation.y
        # msg.orientation.z = self._kinematic_state_from_aw.pose.pose.pose.orientation.z
        # msg.orientation.w = self._kinematic_state_from_aw.pose.pose.pose.orientation.w

        # msg.angular_velocity.x = (
        #     self._kinematic_state_from_aw.twist.twist.twist.angular.x
        # )
        # msg.angular_velocity.y = (
        #     self._kinematic_state_from_aw.twist.twist.twist.angular.y
        # )
        # msg.angular_velocity.z = (
        #     self._kinematic_state_from_aw.twist.twist.twist.angular.z
        # )

        # msg.linear_acceleration.x = (
        #     self._kinematic_state_from_aw.accel.accel.accel.linear.x
        # )
        # msg.linear_acceleration.y = (
        #     self._kinematic_state_from_aw.accel.accel.accel.linear.y
        # )
        # msg.linear_acceleration.z = (
        #     self._kinematic_state_from_aw.accel.accel.accel.linear.z
        # )

        self._imu_pub.publish(self._imu_state)

    # def _publish_control_cmd

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _update_kinematic(self, kinematic: VehicleKinematic) -> None:
        self._past_past_kinematic = self._past_kinematic
        self._past_kinematic = self._kinematic
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

        self._base_map_path = full_path.parent
        self._lanelet2_map_file = full_path.name

        return full_path

    def _get_stable_uuid(self, agent_id):
        """確保同一個 agent_id 永遠拿到同一個 UUID"""
        if agent_id not in self._uuid_map:
            # 使用 uuid.uuid4() 生成隨機 ID，轉成 byte array
            # 注意：這裡假設 agent_id 是 simulator 來的唯一整數或字串
            random_uuid = uuid.uuid4()
            self._uuid_map[agent_id] = list(random_uuid.bytes)

        uuid_msg = UUID()
        uuid_msg.uuid = self._uuid_map[agent_id]
        return uuid_msg

    def _calc_imu_state(self):
        """根據 kinematic 計算 IMU 狀態
        past_past_kinematic --> past_kinematic --> current_kinematic
        """
        dt1 = max(self._kinematic.time - self._past_kinematic.time, 1e-5)
        dt2 = max(self._past_kinematic.time - self._past_past_kinematic.time, 1e-5)
        dt = (dt1 + dt2) / 2.0

        cur_v = Vector3()
        cur_v.x = (self._kinematic.x - self._past_kinematic.x) / dt1
        cur_v.y = (self._kinematic.x - self._past_kinematic.x) / dt1
        cur_v.z = 0.0
        prev_v = Vector3()
        prev_v.x = (self._past_kinematic.x - self._past_past_kinematic.x) / dt2
        prev_v.y = (self._past_kinematic.x - self._past_past_kinematic.x) / dt2
        prev_v.z = 0.0
        linear_acceleration = Vector3()
        linear_acceleration.x = (cur_v.x - prev_v.x) / dt
        linear_acceleration.y = (cur_v.y - prev_v.y) / dt
        linear_acceleration.z = 0.0

        angular_velocity = Vector3()
        angular_velocity.x = 0.0
        angular_velocity.y = 0.0
        angular_velocity.z = (self._kinematic.yaw - self._past_kinematic.yaw) / dt
        self._imu_state.linear_acceleration = linear_acceleration
        self._imu_state.angular_velocity = angular_velocity
        self._imu_state.header.stamp = self._node.get_clock().now().to_msg()
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
