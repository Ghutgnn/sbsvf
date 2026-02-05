import logging
import math
import os
import pprint
import subprocess
import sys
import time
import zipfile
from pathlib import Path
from typing import Optional, Union

import py_trees


from sv.utils.util import get_cfg
from sv.utils.sps import ScenarioPack
from sv.utils.control import Ctrl, CtrlMode
from sv.utils.object import (
    ObjectState,
    ObjectKinematic,
    RoadObjectType,
    Shape,
    ShapeType,
)

logger = logging.getLogger(__name__)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class CarlaAdapter:
    def __init__(self, output_dir: Path, cfg_path: Optional[Union[str, Path]]):
        self._output_dir = output_dir
        self._time_ns = 0
        self.cfg = get_cfg(cfg_path) if cfg_path else {}

        self._carla = None
        self._client = None
        self._world = None
        self._ego_vehicle = None
        self._objects_by_id: dict[int, ObjectState] = {}
        self._prev_yaw_rate: dict[int, float] = {}
        self._original_settings = None
        self._fixed_delta_seconds = None

        self._sync = bool(self.cfg.get("synchronous_mode", True))
        self._no_rendering = bool(self.cfg.get("no_rendering_mode", False))
        self._yaw_sign = float(self.cfg.get("yaw_sign", 1.0))
        self._yaw_offset_deg = float(self.cfg.get("yaw_offset_deg", 0.0))
        self._spawn_z_offset = float(self.cfg.get("spawn_z_offset", 3.0))

        self._ego_bp_id = self.cfg.get("ego_vehicle_bp", "vehicle.tesla.model3")
        self._max_steer_rad: Optional[float] = None
        self._quit_flag = False

        self._spawned_actor_ids: set[int] = set()
        self._scenario_runner_path = self.cfg.get("scenario_runner_path", None)
        self._ego_role_name = self.cfg.get("ego_role_name", "hero")
        self._carla_root = self.cfg.get("carla_root", None)
        self._carla_egg = self.cfg.get("carla_egg", None)
        self._carla_cache_dir = self.cfg.get("carla_cache_dir", "/tmp/carla_pyapi")
        # )
        self._scenario_runner_tm_port = int(
            self.cfg.get("scenario_runner_tm_port", 8000)
        )
        self._scenario_runner_tm_seed = int(self.cfg.get("scenario_runner_tm_seed", 0))

        self._sr_manager = None
        self._sr_scenario = None
        self._sr_tree = None
        self._sr_running = False
        self._sr_ego_vehicles: list = []

    def _connect(self) -> None:
        if self._client is not None:
            return
        try:
            import carla
        except Exception as e:
            raise RuntimeError("CARLA Python API not available") from e

        self._carla = carla
        host = self.cfg.get("host", "localhost")
        port = int(self.cfg.get("port", 2000))
        timeout = float(self.cfg.get("timeout", 10.0))

        client = carla.Client(host, port)
        client.set_timeout(timeout)
        self._client = client

    def _to_carla_yaw(self, yaw_rad: float) -> float:
        return self._yaw_sign * math.degrees(yaw_rad) + self._yaw_offset_deg

    def _from_carla_yaw(self, yaw_deg: float) -> float:
        return math.radians((yaw_deg - self._yaw_offset_deg) * self._yaw_sign)

    def _ensure_world(self, sps: Optional[ScenarioPack]) -> None:
        if self._client is None:
            self._connect()
        carla_map_name = sps.maps.get("carla_map_name", None)
        opendrive_path = sps.maps.get("xodr_path", None)

        world = None
        if carla_map_name:
            world = self._client.load_world(carla_map_name, reset_settings=False)
        elif opendrive_path and hasattr(self._client, "generate_opendrive_world"):
            opendrive_path = Path(opendrive_path)
            if not opendrive_path.exists():
                raise RuntimeError(
                    "OpenDRIVE path not found for CARLA world generation"
                )

            # read opendrive file
            with open(opendrive_path, "r", encoding="utf-8") as f:
                opendrive_str = f.read()
            world = self._client.generate_opendrive_world(
                opendrive_str,
                self._carla.OpendriveGenerationParameters(
                    vertex_distance=2.0,
                    max_road_length=3000.0,
                    wall_height=10.0,
                    additional_width=0.6,
                    smooth_junctions=True,
                    enable_mesh_visibility=True,
                ),
            )
        else:
            raise RuntimeError("Cannot determine CARLA world to load")

        if world is None:
            world = self._client.get_world()

        self._world = world
        if self._original_settings is None:
            self._original_settings = world.get_settings()

    def _apply_world_settings(self) -> None:
        if self._world is None:
            return
        settings = self._world.get_settings()
        settings.synchronous_mode = self._sync
        logger.info("Synchronous mode = %s", settings.synchronous_mode)
        settings.no_rendering_mode = self._no_rendering
        logger.info("No rendering mode = %s", settings.no_rendering_mode)
        if self._fixed_delta_seconds is not None:
            logger.info("Setting fixed_delta_seconds = %s", self._fixed_delta_seconds)
            settings.fixed_delta_seconds = float(self._fixed_delta_seconds)
        self._world.apply_settings(settings)

    def init(self, runtime_spec: dict, sps: ScenarioPack) -> None:
        self._fixed_delta_seconds = runtime_spec.get("dt", 0.01)
        self._connect()

    def _pythonpath_entries(self, sr_script: Optional[Path] = None) -> list[str]:
        entries: list[str] = []
        if sr_script is not None:
            entries.append(str(self._scenario_runner_root(sr_script)))

        if self._carla_root:
            carla_root = Path(self._carla_root)
            entries.append(str(carla_root / "PythonAPI"))
            entries.append(str(carla_root / "PythonAPI" / "carla"))

        dist_path = self._resolve_carla_dist_path()
        if dist_path:
            entries.append(dist_path)

        seen: set[str] = set()
        deduped: list[str] = []
        for entry in entries:
            if not entry or entry in seen:
                continue
            seen.add(entry)
            deduped.append(entry)
        return deduped

    def _scenario_runner_root(self, sr_script: Path) -> Path:
        sr_root = sr_script.parent
        if sr_root.name == "srunner":
            sr_root = sr_root.parent
        return sr_root

    def _resolve_carla_dist_path(self) -> Optional[str]:
        if not self._carla_egg:
            return None
        path = Path(self._carla_egg)
        if path.is_dir():
            return str(path)
        if not path.exists():
            logger.warning("CARLA Python API path not found: %s", path)
            return None
        if zipfile.is_zipfile(path):
            cache_root = Path(self._carla_cache_dir)
            cache_dir = cache_root / path.stem
            carla_pkg_dir = cache_dir / "carla"
            if not carla_pkg_dir.exists():
                cache_dir.mkdir(parents=True, exist_ok=True)
                try:
                    with zipfile.ZipFile(path, "r") as zf:
                        zf.extractall(cache_dir)
                except Exception:
                    logger.exception("Failed to extract CARLA Python API: %s", path)
            return str(cache_dir)
        return str(path)

    def _build_scenario_runner_env(self, sr_script: Path) -> dict[str, str]:
        env = os.environ.copy()
        entries = self._pythonpath_entries(sr_script)
        if entries:
            existing = env.get("PYTHONPATH", "")
            python_path = os.pathsep.join(entries + ([existing] if existing else []))
            env["PYTHONPATH"] = python_path
        env.setdefault(
            "SCENARIO_RUNNER_ROOT", str(self._scenario_runner_root(sr_script))
        )
        if self._carla_root and "CARLA_ROOT" not in env:
            env["CARLA_ROOT"] = str(self._carla_root)
        return env

    def _ensure_scenario_runner_imports(self, sr_script: Path) -> None:
        for entry in self._pythonpath_entries(sr_script):
            if entry not in sys.path:
                sys.path.append(entry)
        os.environ.setdefault(
            "SCENARIO_RUNNER_ROOT", str(self._scenario_runner_root(sr_script))
        )

    def _destroy_spawned_actors(self) -> None:
        if self._world is None:
            return
        if not self._spawned_actor_ids:
            return
        for actor_id in list(self._spawned_actor_ids):
            actor = self._world.get_actor(actor_id)
            if actor is not None:
                try:
                    actor.destroy()
                except Exception:
                    logger.exception("Failed to destroy actor %s", actor_id)
        self._spawned_actor_ids.clear()
        self._objects_by_id.clear()
        self._prev_yaw_rate.clear()

    def _spawn_ego(self, sps: ScenarioPack):
        input("Press Enter to spawn ego vehicle...")
        if self._world is None:
            raise RuntimeError("CARLA world not available")

        bp_lib = self._world.get_blueprint_library()
        try:
            ego_bp = bp_lib.find(self._ego_bp_id)
        except Exception:
            candidates = bp_lib.filter("vehicle.*")
            if not candidates:
                raise RuntimeError("No vehicle blueprints available in CARLA")
            ego_bp = candidates[0]

        if ego_bp.has_attribute("role_name"):
            ego_bp.set_attribute("role_name", self._ego_role_name)

        pos = sps.ego.spawn.position
        loc = self._carla.Location(
            x=float(pos.x),
            y=float(pos.y) * self._yaw_sign,
            z=float(pos.z) + self._spawn_z_offset,
        )
        rot = self._carla.Rotation(
            pitch=math.degrees(float(pos.p)),
            yaw=self._to_carla_yaw(float(pos.h)),
            roll=math.degrees(float(pos.r)),
        )
        transform = self._carla.Transform(loc, rot)

        ego = self._world.try_spawn_actor(ego_bp, transform)
        if ego is None:
            logger.warning("Initial spawn failed, trying spawn points...")
            spawn_points = self._world.get_map().get_spawn_points()
            if not spawn_points:
                raise RuntimeError("Failed to spawn ego vehicle (no spawn points)")
            ego = self._world.try_spawn_actor(ego_bp, spawn_points[0])
            if ego is None:
                raise RuntimeError("Failed to spawn ego vehicle")

        self._ego_vehicle = ego
        self._spawned_actor_ids.add(ego.id)

        try:
            phys = ego.get_physics_control()
            max_steer = max([w.max_steer_angle for w in phys.wheels])
            self._max_steer_rad = math.radians(max_steer)
        except Exception:
            self._max_steer_rad = None

    def reset(self, output_dir: Path, sps: ScenarioPack, params: Optional[dict] = None):
        self._output_dir = output_dir
        self._time_ns = 0
        self._quit_flag = False

        self._ensure_world(sps)
        self._apply_world_settings()
        self._destroy_spawned_actors()
        self._stop_scenario_runner_module()

        logger.info("Starting ScenarioRunner...")
        self._start_scenario_runner(sps, params)

        if self._ego_vehicle is None:
            logger.warning("Ego vehicle not found after waiting, spawning ego...")
            self._spawn_ego(sps)

        if self._sync:
            self._world.tick()
        objects = self._collect_objects()

        return objects

    def _start_scenario_runner(self, sps: ScenarioPack, params: Optional[dict]) -> None:
        if self._scenario_runner_path is None:
            raise RuntimeError(
                "scenario_runner_path is required when use_scenario_runner"
            )

        sr_path = Path(self._scenario_runner_path)
        if sr_path.is_dir():
            sr_script = sr_path / "scenario_runner.py"
        else:
            sr_script = sr_path

        self._start_scenario_runner_module(sps, params, sr_script)

    def _start_scenario_runner_module(
        self, sps: ScenarioPack, params: Optional[dict], sr_script: Path
    ) -> None:
        if self._client is None or self._world is None:
            raise RuntimeError("CARLA client/world not available")
        self._ensure_scenario_runner_imports(sr_script)

        from srunner.scenarioconfigs.openscenario_configuration import (
            OpenScenarioConfiguration,
        )
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
        from srunner.scenariomanager.scenario_manager import ScenarioManager
        from srunner.scenariomanager.timer import GameTime
        from srunner.scenariomanager.watchdog import Watchdog
        from srunner.scenarios.open_scenario import OpenScenario

        CarlaDataProvider.set_client(self._client)
        CarlaDataProvider.set_world(self._world)
        CarlaDataProvider.set_traffic_manager_port(self._scenario_runner_tm_port)
        tm = self._client.get_trafficmanager(self._scenario_runner_tm_port)
        tm.set_random_device_seed(self._scenario_runner_tm_seed)
        if self._sync:
            tm.set_synchronous_mode(True)

        openscenario_params: dict[str, str] = {}
        if params:
            openscenario_params = {str(k): str(v) for k, v in params.items()}

        xosc_path = sps.scenarios.get("xosc", None)
        if xosc_path is None:
            raise RuntimeError("ScenarioPack has no xosc scenario to run")
        config = OpenScenarioConfiguration(
            str(xosc_path), self._client, openscenario_params
        )

        ego_vehicles = []
        for ego_cfg in config.ego_vehicles:
            actor = CarlaDataProvider.request_new_actor(
                ego_cfg.model,
                ego_cfg.transform,
                ego_cfg.rolename,
                random_location=ego_cfg.random_location,
                color=ego_cfg.color,
                actor_category=ego_cfg.category,
            )
            if actor is None:
                raise RuntimeError(f"Failed to spawn ego vehicle '{ego_cfg.rolename}'")
            ego_vehicles.append(actor)

        scenario = OpenScenario(
            world=self._world,
            ego_vehicles=ego_vehicles,
            config=config,
            config_file=str(xosc_path),
            timeout=sps.timeout_ns / 1e9 if sps.timeout_ns > 0 else 10000.0,
        )

        self._sr_scenario = scenario
        self._sr_tree = scenario.scenario_tree

        ### Debug: render scenario tree
        # py_trees.display.render_dot_tree(self._sr_tree, name="ScenarioRunnerTree")

        self._sr_ego_vehicles = ego_vehicles
        self._ego_vehicle = ego_vehicles[0] if ego_vehicles else None

        GameTime.restart()
        self._sr_running = True

    def _stop_scenario_runner_module(self) -> None:
        if (
            self._sr_manager is None
            and self._sr_scenario is None
            and self._sr_tree is None
        ):
            return
        try:
            if self._sr_manager is not None:
                self._sr_manager.cleanup()
        except Exception:
            logger.exception("Failed to cleanup ScenarioRunner manager")
        try:
            if self._sr_scenario is not None:
                self._sr_scenario.remove_all_actors()
        except Exception:
            logger.exception("Failed to remove ScenarioRunner actors")
        try:
            from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

            CarlaDataProvider.cleanup()
        except Exception:
            logger.exception("Failed to cleanup ScenarioRunner data provider")

        self._sr_manager = None
        self._sr_scenario = None
        self._sr_tree = None
        self._sr_running = False
        self._sr_ego_vehicles = []

    def _tick_scenario_runner_module(self) -> None:
        if self._world is None:
            return
        if self._sr_scenario is None:
            return

        snapshot = self._world.get_snapshot()
        timestamp = snapshot.timestamp

        if self._sr_tree is None or not self._sr_running:
            return
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
        from srunner.scenariomanager.timer import GameTime
        import py_trees

        GameTime.on_carla_tick(timestamp)
        CarlaDataProvider.on_carla_tick()
        self._sr_tree.tick_once()
        if self._sr_tree.status != py_trees.common.Status.RUNNING:
            
            self._sr_running = False
            self._quit_flag = True

    def _find_ego_vehicle(self):
        if self._world is None:
            return None
        actors = self._world.get_actors().filter("vehicle.*")
        for actor in actors:
            role = actor.attributes.get("role_name", "")
            if role == self._ego_role_name:
                return actor
        return None

    def _wait_for_ego(self):
        if self._world is None:
            return None
        deadline = time.time() + self._wait_for_ego_sec
        while time.time() < deadline:
            ego = self._find_ego_vehicle()
            if ego is not None:
                return ego
            try:
                self._world.wait_for_tick()
            except Exception:
                time.sleep(0.1)
        logger.warning("Ego vehicle not found within %.1f sec", self._wait_for_ego_sec)
        return None

    def _actor_type(self, actor) -> RoadObjectType:
        type_id = actor.type_id.lower()
        print(type_id)
        if type_id.startswith("walker.pedestrian"):
            return RoadObjectType.PEDESTRIAN
        if type_id.startswith("vehicle."):
            if "bus" in type_id:
                return RoadObjectType.BUS
            if "truck" in type_id:
                return RoadObjectType.TRUCK
            if "trailer" in type_id:
                return RoadObjectType.TRAILER
            if "motorcycle" in type_id or "motorbike" in type_id:
                return RoadObjectType.MOTORCYCLE
            if "bicycle" in type_id or "bike" in type_id or "diamondback" in type_id:
                return RoadObjectType.BICYCLE
            if "van" in type_id:
                return RoadObjectType.VAN
            return RoadObjectType.CAR
        return RoadObjectType.UNKNOWN

    def _shape_from_actor(self, actor) -> Shape:
        try:
            bb = actor.bounding_box
            dims = (
                float(bb.extent.x * 2.0),
                float(bb.extent.y * 2.0),
                float(bb.extent.z * 2.0),
            )
            return Shape(type=ShapeType.BOUNDING_BOX, dimensions=dims)
        except Exception:
            return Shape(type=ShapeType.BOUNDING_BOX, dimensions=(0.0, 0.0, 0.0))

    def _get_forward_speed(self, actor) -> float:
        vel = actor.get_velocity()
        fwd = actor.get_transform().get_forward_vector()
        return float(vel.x * fwd.x + vel.y * fwd.y + vel.z * fwd.z)

    def _get_forward_accel(self, actor) -> float:
        acc = actor.get_acceleration()
        fwd = actor.get_transform().get_forward_vector()
        return float(acc.x * fwd.x + acc.y * fwd.y + acc.z * fwd.z)

    def _collect_objects(self) -> list[ObjectState]:
        if self._world is None:
            return []

        snapshot = self._world.get_snapshot()
        sim_time_ns = int(snapshot.timestamp.elapsed_seconds * 1e9)

        actors = []
        actors.extend(self._world.get_actors().filter("vehicle.*"))
        actors.extend(self._world.get_actors().filter("walker.pedestrian.*"))

        actor_ids = {a.id for a in actors}
        for stale_id in list(self._objects_by_id.keys()):
            if stale_id not in actor_ids:
                self._objects_by_id.pop(stale_id, None)
                self._prev_yaw_rate.pop(stale_id, None)

        objects: list[ObjectState] = []

        def upsert(actor):
            transform = actor.get_transform()
            ang = actor.get_angular_velocity()

            yaw = self._from_carla_yaw(float(transform.rotation.yaw))
            speed = self._get_forward_speed(actor)
            accel = self._get_forward_accel(actor)
            yaw_rate = math.radians(float(ang.z)) * self._yaw_sign

            prev_rate = self._prev_yaw_rate.get(actor.id, yaw_rate)
            dt_s = 0.0
            if self._time_ns > 0 and sim_time_ns > self._time_ns:
                dt_s = (sim_time_ns - self._time_ns) / 1e9
            yaw_acc = (yaw_rate - prev_rate) / dt_s if dt_s > 0 else 0.0
            self._prev_yaw_rate[actor.id] = yaw_rate

            kin = ObjectKinematic(
                time_ns=sim_time_ns,
                x=float(transform.location.x),
                y=float(transform.location.y) * self._yaw_sign,
                z=float(transform.location.z),
                yaw=float(yaw),
                speed=float(speed),
                accel=float(accel),
                yaw_rate=float(yaw_rate),
                yaw_acc=float(yaw_acc),
            )

            obj = self._objects_by_id.get(actor.id)
            if obj is None:
                obj = ObjectState.create(
                    type=self._actor_type(actor),
                    kinematic=kin,
                    shape=self._shape_from_actor(actor),
                )
                self._objects_by_id[actor.id] = obj
            else:
                obj.update(kin)

            return obj

        if self._ego_vehicle is not None:
            objects.append(upsert(self._ego_vehicle))

        for actor in actors:
            if self._ego_vehicle is not None and actor.id == self._ego_vehicle.id:
                continue
            objects.append(upsert(actor))

        self._time_ns = sim_time_ns

        return objects

    def _apply_ctrl(self, ctrl: Ctrl, dt_s: float) -> None:
        if self._ego_vehicle is None:
            return
        if ctrl is None or ctrl.mode == CtrlMode.None_:
            return

        if ctrl.mode == CtrlMode.THROTTLE_STEER:
            payload = ctrl.payload or {}
            if "throttle" in payload or "brake" in payload:
                throttle = float(payload.get("throttle", 0.0))
                brake = float(payload.get("brake", 0.0))
                steer = float(payload.get("steer", payload.get("wheel", 0.0)))
                throttle = _clamp(throttle, 0.0, 1.0)
                brake = _clamp(brake, 0.0, 1.0)
                steer = _clamp(steer, -1.0, 1.0)
            else:
                pedal = float(payload.get("pedal", 0.0))
                wheel = float(payload.get("wheel", 0.0))
                if pedal >= 0:
                    throttle = _clamp(abs(pedal), 0.0, 1.0)
                    brake = 0.0
                else:
                    throttle = 0.0
                    brake = _clamp(abs(pedal), 0.0, 1.0)
                steer = _clamp(wheel, -1.0, 1.0)

            control = self._carla.VehicleControl(
                throttle=throttle, steer=steer * self._yaw_sign, brake=brake
            )
            self._ego_vehicle.apply_control(control)
            return

        if ctrl.mode == CtrlMode.VEL_STEER:
            payload = ctrl.payload or {}
            target_speed = float(
                payload.get("speed", self._get_forward_speed(self._ego_vehicle))
            )
            steering_angle = float(payload.get("h", 0.0)) * self._yaw_sign

            if self._max_steer_rad:
                steer = _clamp(steering_angle / self._max_steer_rad, -1.0, 1.0)
            else:
                steer = _clamp(steering_angle, -1.0, 1.0)

            cur_speed = self._get_forward_speed(self._ego_vehicle)
            kp = float(self.cfg.get("speed_kp", 0.5))
            kb = float(self.cfg.get("brake_kp", kp))
            speed_err = target_speed - cur_speed
            throttle = _clamp(speed_err * kp, 0.0, 1.0)
            brake = _clamp(-speed_err * kb, 0.0, 1.0)

            control = self._carla.VehicleControl(
                throttle=throttle, steer=steer, brake=brake
            )
            self._ego_vehicle.apply_control(control)
            return

        if ctrl.mode == CtrlMode.POSITION:
            payload = ctrl.payload or {}
            transform = self._ego_vehicle.get_transform()
            x = float(payload.get("x", transform.location.x))
            y = float(payload.get("y", transform.location.y))
            z = float(payload.get("z", transform.location.z))
            h = float(payload.get("h", self._from_carla_yaw(transform.rotation.yaw)))
            yaw_deg = self._to_carla_yaw(h)

            loc = self._carla.Location(x=x, y=y, z=z)
            rot = self._carla.Rotation(
                pitch=transform.rotation.pitch,
                yaw=yaw_deg,
                roll=transform.rotation.roll,
            )
            self._ego_vehicle.set_transform(self._carla.Transform(loc, rot))
            return

        logger.warning("Unsupported control mode: %s", ctrl.mode)

    def step(self, ctrl: Ctrl, time_stamp_ns: int):
        if self._world is None:
            return []

        dt_s = 0.0
        if self._time_ns > 0:
            dt_s = (time_stamp_ns - self._time_ns) / 1e9

        self._apply_ctrl(ctrl, dt_s)
        self._tick_scenario_runner_module()
        if self._sync:
            self._world.tick()
        else:
            self._world.wait_for_tick()
        return self._collect_objects()

    def stop(self):
        try:
            self._destroy_spawned_actors()
        finally:
            self._stop_scenario_runner_module()
            if self._world is not None and self._original_settings is not None:
                try:
                    self._world.apply_settings(self._original_settings)
                except Exception:
                    logger.exception("Failed to restore CARLA world settings")

        self._ego_vehicle = None
        self._world = None
        self._client = None
        logger.info("CARLA simulator stopped.")

    def should_quit(self):
        return self._quit_flag
