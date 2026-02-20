from __future__ import annotations

import importlib
import importlib.util
import logging
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Optional

from sv.utils.control import Ctrl, CtrlMode
from sv.utils.object import ObjectState, ObjectKinematic
from sv.utils.sps import ScenarioPack
from sv.utils.util import get_cfg

logger = logging.getLogger(__name__)


class LeaderboardAgentAV:
    """
    Adapter for CARLA Leaderboard agents.
    - init(): prepare imports and connect to CARLA
    - reset(): find ego, create agent instance, spawn sensors
    - step(): build input_data (from sensors or obs), call agent.run_step()
    - stop(): destroy sensors and agent
    """

    def __init__(self, output_dir: Path, cfg_path: Optional[Path]):
        self._output_dir = output_dir
        self.cfg = get_cfg(Path(cfg_path)) if cfg_path else {}

        self._host = self.cfg.get("host", "localhost")
        self._port = int(self.cfg.get("port", 2000))
        self._timeout = float(self.cfg.get("timeout", 10.0))

        self._carla_root = self.cfg.get("carla_root") or os.environ.get("CARLA_ROOT")
        self._carla_egg = self.cfg.get("carla_egg")
        self._leaderboard_root = self.cfg.get("leaderboard_root")
        self._agent_path = self.cfg.get("agent_path")
        self._agent_entrypoint = self.cfg.get("agent_entrypoint")
        self._agent_module = self.cfg.get("agent_module")
        self._agent_class = self.cfg.get("agent_class")
        self._agent_file = self.cfg.get("agent_file")
        self._agent_config = self.cfg.get("agent_config")
        self._agent_kwargs = dict(self.cfg.get("agent_kwargs", {}))

        self._ego_role_name = self.cfg.get("ego_role_name", "hero")
        self._yaw_sign = float(self.cfg.get("yaw_sign", -1.0))
        self._yaw_offset_deg = float(self.cfg.get("yaw_offset_deg", 0.0))
        self._spawn_z_offset = float(self.cfg.get("spawn_z_offset", 0.0))
        self._max_wait_sec = float(self.cfg.get("max_wait_sec", 10.0))

        self._use_carla_sensors = bool(self.cfg.get("use_carla_sensors", True))
        self._sensor_timeout_sec = float(self.cfg.get("sensor_timeout_sec", 0.2))
        self._sensor_tick = self.cfg.get("sensor_tick", None)

        self._carla = None
        self._client = None
        self._world = None
        self._map = None
        self._ego_vehicle = None

        self._agent = None
        self._sensor_specs: list[dict[str, Any]] = []
        self._sensors: dict[str, Any] = {}
        self._sensor_data: dict[str, tuple[float, Any]] = {}

        self._sps: Optional[ScenarioPack] = None
        self._quit_flag = False

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def init(self, runtime_spec: dict, sps: ScenarioPack) -> None:
        self._sps = sps
        self._ensure_import_paths()
        self._connect()
        logger.info("LeaderboardAgentAV initialized.")

    def reset(
        self,
        output_dir: Path,
        sps: ScenarioPack,
        init_obs: Optional[list[ObjectState]] = None,
    ) -> None:
        self._output_dir = output_dir
        self._sps = sps
        self._quit_flag = False

        self._connect()
        self._world = self._client.get_world()
        self._map = self._world.get_map() if self._world else None

        self._destroy_sensors()
        self._destroy_agent()

        self._ego_vehicle = self._wait_for_ego(self._max_wait_sec)
        if self._ego_vehicle is None:
            raise RuntimeError(
                f"Ego vehicle with role_name={self._ego_role_name!r} not found."
            )

        self._agent = self._build_agent()
        self._prepare_agent_plan()
        self._setup_sensors()

    def step(self, obs: Any, time_stamp_ns: int) -> Ctrl:
        if self._agent is None:
            return Ctrl(mode=CtrlMode.None_)

        input_data = self._build_input_data(obs, time_stamp_ns)
        timestamp_s = float(time_stamp_ns) / 1e9

        try:
            if hasattr(self._agent, "run_step"):
                control = self._agent.run_step(input_data, timestamp_s)
            else:
                control = self._agent(input_data, timestamp_s)
        except Exception:
            logger.exception("Leaderboard agent run_step failed.")
            self._quit_flag = True
            return Ctrl(mode=CtrlMode.None_)

        return self._control_to_ctrl(control)

    def stop(self) -> None:
        self._destroy_sensors()
        self._destroy_agent()

    def should_quit(self) -> bool:
        if self._quit_flag:
            return True
        if self._agent is None:
            return False
        if hasattr(self._agent, "done"):
            try:
                return bool(self._agent.done())
            except Exception:
                return False
        if hasattr(self._agent, "is_done"):
            try:
                return bool(self._agent.is_done())
            except Exception:
                return False
        return False

    # ------------------------------------------------------------------
    # imports / connection
    # ------------------------------------------------------------------
    def _ensure_import_paths(self) -> None:
        def add_path(p: Optional[str]) -> None:
            if not p:
                return
            path = str(Path(p).expanduser().resolve())
            if path not in sys.path:
                sys.path.insert(0, path)

        if self._leaderboard_root:
            add_path(self._leaderboard_root)
        if self._agent_path:
            add_path(self._agent_path)

        if self._carla_root:
            root = Path(self._carla_root)
            add_path(str(root / "PythonAPI"))
            add_path(str(root / "PythonAPI" / "carla"))
            dist_dir = root / "PythonAPI" / "carla" / "dist"
            if self._carla_egg is None and dist_dir.exists():
                matches = sorted(dist_dir.glob("*.whl")) + sorted(dist_dir.glob("*.egg"))
                if matches:
                    self._carla_egg = str(matches[0])

        if self._carla_egg:
            add_path(self._carla_egg)

        try:
            import carla  # type: ignore
        except Exception as e:
            raise RuntimeError("CARLA Python API not available") from e
        self._carla = carla

    def _connect(self) -> None:
        if self._client is not None:
            return
        self._ensure_import_paths()
        client = self._carla.Client(self._host, self._port)
        client.set_timeout(self._timeout)
        self._client = client

    # ------------------------------------------------------------------
    # agent / sensors
    # ------------------------------------------------------------------
    def _build_agent(self) -> Any:
        agent_cls = self._load_agent_class()
        agent = agent_cls(**self._agent_kwargs)

        if hasattr(agent, "setup"):
            try:
                agent.setup(self._agent_config or "")
            except TypeError:
                agent.setup()

        if hasattr(agent, "set_world"):
            agent.set_world(self._world)
        elif hasattr(agent, "_world"):
            try:
                agent._world = self._world
            except Exception:
                pass

        if hasattr(agent, "set_ego_vehicle"):
            agent.set_ego_vehicle(self._ego_vehicle)
        elif hasattr(agent, "_vehicle"):
            try:
                agent._vehicle = self._ego_vehicle
            except Exception:
                pass

        return agent

    def _destroy_agent(self) -> None:
        if self._agent is None:
            return
        if hasattr(self._agent, "destroy"):
            try:
                self._agent.destroy()
            except Exception:
                logger.exception("Leaderboard agent destroy() failed.")
        self._agent = None

    def _load_agent_class(self) -> Any:
        if self._agent_entrypoint:
            module_name, class_name = self._agent_entrypoint.split(":")
            module = importlib.import_module(module_name)
            return getattr(module, class_name)

        if self._agent_file:
            agent_path = Path(self._agent_file).expanduser().resolve()
            if not agent_path.exists():
                raise FileNotFoundError(f"agent_file not found: {agent_path}")
            module_name = agent_path.stem
            spec = importlib.util.spec_from_file_location(module_name, agent_path)
            if spec is None or spec.loader is None:
                raise RuntimeError(f"Failed to load agent module from {agent_path}")
            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            spec.loader.exec_module(module)
            class_name = self._agent_class
            if not class_name and hasattr(module, "get_entry_point"):
                class_name = module.get_entry_point()
            if not class_name:
                raise RuntimeError("agent_class not set and get_entry_point missing")
            return getattr(module, class_name)

        if self._agent_module and self._agent_class:
            module = importlib.import_module(self._agent_module)
            return getattr(module, self._agent_class)

        raise RuntimeError(
            "Agent entrypoint not configured. Set agent_entrypoint or "
            "agent_module/agent_class or agent_file."
        )

    def _prepare_agent_plan(self) -> None:
        if self._agent is None or self._map is None or self._ego_vehicle is None:
            return

        if not hasattr(self._agent, "set_global_plan"):
            return

        try:
            from agents.navigation.global_route_planner import (  # type: ignore
                GlobalRoutePlanner,
            )
            from agents.navigation.global_route_planner_dao import (  # type: ignore
                GlobalRoutePlannerDAO,
            )
        except Exception:
            logger.info("GlobalRoutePlanner not available; skip set_global_plan.")
            return

        try:
            goal_pos = self._sps.ego.goal.position if self._sps else None
            if goal_pos is None:
                return
            start = self._ego_vehicle.get_location()
            goal = self._carla.Location(
                x=float(goal_pos.x),
                y=float(goal_pos.y) * self._yaw_sign,
                z=float(goal_pos.z) + self._spawn_z_offset,
            )
            dao = GlobalRoutePlannerDAO(self._map, 2.0)
            grp = GlobalRoutePlanner(dao)
            grp.setup()
            route = grp.trace_route(start, goal)

            global_plan = [
                (wp.transform.location, road_opt) for (wp, road_opt) in route
            ]
            global_plan_world = [(wp.transform, road_opt) for (wp, road_opt) in route]
            self._agent.set_global_plan(global_plan, global_plan_world)
        except Exception:
            logger.exception("Failed to set global plan for leaderboard agent.")

    def _setup_sensors(self) -> None:
        self._sensor_specs = []
        if self._agent is None:
            return

        if hasattr(self._agent, "sensors"):
            try:
                self._sensor_specs = list(self._agent.sensors())
            except Exception:
                logger.exception("Leaderboard agent sensors() failed.")
                self._sensor_specs = []

        if not self._use_carla_sensors:
            return
        if self._world is None or self._ego_vehicle is None:
            return

        bp_lib = self._world.get_blueprint_library()
        for spec in self._sensor_specs:
            sensor_type = spec.get("type", "")
            sensor_id = spec.get("id", sensor_type)
            if self._is_virtual_sensor(sensor_type):
                continue
            try:
                bp = bp_lib.find(sensor_type)
            except Exception:
                logger.warning("Unknown sensor type: %s", sensor_type)
                continue

            for key, val in spec.items():
                if key in ("type", "id", "x", "y", "z", "roll", "pitch", "yaw"):
                    continue
                try:
                    bp.set_attribute(str(key), str(val))
                except Exception:
                    pass
            if self._sensor_tick is not None:
                try:
                    bp.set_attribute("sensor_tick", str(self._sensor_tick))
                except Exception:
                    pass

            transform = self._carla.Transform(
                self._carla.Location(
                    x=float(spec.get("x", 0.0)),
                    y=float(spec.get("y", 0.0)),
                    z=float(spec.get("z", 0.0)),
                ),
                self._carla.Rotation(
                    roll=float(spec.get("roll", 0.0)),
                    pitch=float(spec.get("pitch", 0.0)),
                    yaw=float(spec.get("yaw", 0.0)),
                ),
            )
            try:
                sensor = self._world.spawn_actor(
                    bp, transform, attach_to=self._ego_vehicle
                )
            except Exception:
                logger.exception("Failed to spawn sensor: %s", sensor_id)
                continue

            def _make_cb(sid: str):
                def _cb(data):
                    try:
                        ts = float(getattr(data, "timestamp", 0.0))
                    except Exception:
                        ts = 0.0
                    self._sensor_data[sid] = (ts, data)

                return _cb

            sensor.listen(_make_cb(sensor_id))
            self._sensors[sensor_id] = sensor

    def _destroy_sensors(self) -> None:
        for sensor in list(self._sensors.values()):
            try:
                sensor.stop()
            except Exception:
                pass
            try:
                sensor.destroy()
            except Exception:
                pass
        self._sensors.clear()
        self._sensor_data.clear()

    def _is_virtual_sensor(self, sensor_type: str) -> bool:
        return sensor_type in (
            "sensor.other.speedometer",
            "sensor.opendrive_map",
            "sensor.other.opendrive_map",
        )

    # ------------------------------------------------------------------
    # input building
    # ------------------------------------------------------------------
    def _build_input_data(self, obs: Any, time_stamp_ns: int) -> dict[str, Any]:
        input_data: dict[str, Any] = {}
        timestamp_s = float(time_stamp_ns) / 1e9

        ego_kin = self._extract_ego_kinematic(obs)

        for spec in self._sensor_specs:
            sensor_type = spec.get("type", "")
            sensor_id = spec.get("id", sensor_type)
            if sensor_id in self._sensor_data:
                input_data[sensor_id] = self._sensor_data[sensor_id]
                continue

            if self._is_virtual_sensor(sensor_type):
                virtual = self._build_virtual_sensor(sensor_type, ego_kin, timestamp_s)
                if virtual is not None:
                    input_data[sensor_id] = virtual
                    continue

            if sensor_type == "sensor.other.gnss":
                virtual = self._build_virtual_gnss(ego_kin, timestamp_s)
                if virtual is not None:
                    input_data[sensor_id] = virtual
                    continue

            if sensor_type == "sensor.other.imu":
                virtual = self._build_virtual_imu(ego_kin, timestamp_s)
                if virtual is not None:
                    input_data[sensor_id] = virtual
                    continue

            # fallback: provide empty tuple
            input_data[sensor_id] = (timestamp_s, None)

        return input_data

    def _build_virtual_sensor(
        self, sensor_type: str, ego_kin: Optional[ObjectKinematic], timestamp_s: float
    ) -> Optional[tuple[float, Any]]:
        if sensor_type == "sensor.other.speedometer":
            speed = float(ego_kin.speed) if ego_kin else 0.0
            return (timestamp_s, speed)

        if sensor_type in ("sensor.opendrive_map", "sensor.other.opendrive_map"):
            if self._map is None:
                return (timestamp_s, "")
            try:
                return (timestamp_s, self._map.to_opendrive())
            except Exception:
                return (timestamp_s, "")

        return None

    def _build_virtual_gnss(
        self, ego_kin: Optional[ObjectKinematic], timestamp_s: float
    ) -> Optional[tuple[float, Any]]:
        if ego_kin is None:
            return None

        lat = lon = alt = 0.0
        if self._world is not None:
            try:
                loc = self._carla.Location(
                    x=float(ego_kin.x),
                    y=float(ego_kin.y) * self._yaw_sign,
                    z=float(ego_kin.z),
                )
                geo = self._world.get_map().transform_to_geolocation(loc)
                lat = float(geo.latitude)
                lon = float(geo.longitude)
                alt = float(geo.altitude)
            except Exception:
                pass

        gnss = SimpleNamespace(latitude=lat, longitude=lon, altitude=alt)
        return (timestamp_s, gnss)

    def _build_virtual_imu(
        self, ego_kin: Optional[ObjectKinematic], timestamp_s: float
    ) -> Optional[tuple[float, Any]]:
        if ego_kin is None:
            return None

        acc = float(ego_kin.acceleration)
        yaw_rate = float(ego_kin.yaw_rate)
        compass = float(ego_kin.yaw)

        vec = self._carla.Vector3D if self._carla else None
        if vec:
            accel = vec(x=acc, y=0.0, z=0.0)
            gyro = vec(x=0.0, y=0.0, z=yaw_rate)
        else:
            accel = SimpleNamespace(x=acc, y=0.0, z=0.0)
            gyro = SimpleNamespace(x=0.0, y=0.0, z=yaw_rate)

        imu = SimpleNamespace(accelerometer=accel, gyroscope=gyro, compass=compass)
        return (timestamp_s, imu)

    def _extract_ego_kinematic(self, obs: Any) -> Optional[ObjectKinematic]:
        if obs is None:
            return None
        if isinstance(obs, list) and obs:
            first = obs[0]
            if hasattr(first, "kinematic"):
                return first.kinematic
            try:
                return ObjectState.from_pb(first).kinematic
            except Exception:
                pass
        if isinstance(obs, dict) and "ego" in obs:
            ego = obs["ego"]
            if isinstance(ego, dict):
                return ObjectKinematic.from_dict(ego)
            if hasattr(ego, "kinematic"):
                return ego.kinematic
        return None

    # ------------------------------------------------------------------
    # control conversion
    # ------------------------------------------------------------------
    def _control_to_ctrl(self, control: Any) -> Ctrl:
        if control is None:
            return Ctrl(mode=CtrlMode.None_)

        if hasattr(control, "throttle") and hasattr(control, "steer"):
            throttle = float(getattr(control, "throttle", 0.0))
            steer = float(getattr(control, "steer", 0.0))
            brake = float(getattr(control, "brake", 0.0))
            return Ctrl(
                mode=CtrlMode.THROTTLE_STEER,
                payload={"throttle": throttle, "steer": steer, "brake": brake},
            )

        if isinstance(control, dict):
            throttle = float(control.get("throttle", 0.0))
            steer = float(control.get("steer", 0.0))
            brake = float(control.get("brake", 0.0))
            return Ctrl(
                mode=CtrlMode.THROTTLE_STEER,
                payload={"throttle": throttle, "steer": steer, "brake": brake},
            )

        return Ctrl(mode=CtrlMode.None_)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _wait_for_ego(self, timeout_s: float):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self._world is None:
                return None
            actors = self._world.get_actors().filter("vehicle.*")
            for actor in actors:
                role = actor.attributes.get("role_name", "")
                if role == self._ego_role_name:
                    return actor
            time.sleep(0.05)
        return None
