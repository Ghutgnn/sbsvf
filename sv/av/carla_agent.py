import logging
import math
import os
import random
import sys
import time
from pathlib import Path
from typing import Any, Optional

from sv.registry import register_av
from sv.utils.control import Ctrl, CtrlMode
from sv.utils.object import ObjectState, RoadObjectType
from sv.utils.sps import ScenarioPack
from sv.utils.util import get_cfg

logger = logging.getLogger(__name__)


class CarlaAgentAV:
    """
    CARLA automatic-control style AV adapter.
    - init(): connect to CARLA, prepare agent classes
    - reset(): find ego by role_name, create agent, set destination/speed
    - step(): run agent.run_step(), convert to Ctrl
    """

    def __init__(self, output_dir: Path, cfg_path: Optional[Path]):
        self._output_dir = output_dir
        self.cfg = get_cfg(Path(cfg_path)) if cfg_path else {}

        self._host = self.cfg.get("host", "localhost")
        self._port = int(self.cfg.get("port", 2000))
        self._timeout = float(self.cfg.get("timeout", 10.0))

        self._carla_root = self.cfg.get("carla_root") or os.environ.get("CARLA_ROOT")
        self._carla_egg = self.cfg.get("carla_egg")
        self._sync = bool(self.cfg.get("sync", True))
        self._no_rendering = bool(self.cfg.get("no_rendering", False))
        self._fixed_delta_seconds = self.cfg.get("fixed_delta_seconds", 0.01)

        self._ego_role_name = self.cfg.get("ego_role_name", "hero")
        self._ego_bp_id = self.cfg.get("ego_bp_id", "vehicle.tesla.model3")
        self._agent_type = str(self.cfg.get("agent_type", "behavior")).lower()
        self._behavior = str(self.cfg.get("behavior", "normal")).lower()
        self._random_destination = bool(self.cfg.get("random_destination", False))
        self._follow_speed_limits = bool(self.cfg.get("follow_speed_limits", False))
        self._ignore_traffic_lights = bool(self.cfg.get("ignore_traffic_lights", False))
        self._ignore_stop_signs = bool(self.cfg.get("ignore_stop_signs", False))
        self._ignore_vehicles = bool(self.cfg.get("ignore_vehicles", False))

        self._target_speed = self.cfg.get("target_speed", None)
        self._target_speed_is_mps = bool(self.cfg.get("target_speed_is_mps", False))

        self._yaw_sign = float(self.cfg.get("yaw_sign", -1.0))
        self._yaw_offset_deg = float(self.cfg.get("yaw_offset_deg", 0.0))
        self._max_wait_sec = float(self.cfg.get("max_wait_sec", 10.0))
        self._spawn_z_offset = float(self.cfg.get("spawn_z_offset", 3.0))

        self._original_settings = None
        self._carla = None
        self._BehaviorAgent = None
        self._BasicAgent = None
        self._ConstantVelocityAgent = None

        self._client = None
        self._world = None
        self._map = None
        self._vehicle = None
        self._agent = None
        self._other_actors: list[Any] = []
        self._other_actor_types: list[RoadObjectType] = []

        self._sps: Optional[ScenarioPack] = None
        self._quit_flag = False

    def _ensure_carla_imports(self) -> None:
        if self._carla is not None:
            return

        entries: list[str] = []
        if self._carla_root:
            root = Path(self._carla_root)
            entries.append(str(root / "PythonAPI"))
            entries.append(str(root / "PythonAPI" / "carla"))
            dist_dir = root / "PythonAPI" / "carla" / "dist"
            if self._carla_egg is None and dist_dir.exists():
                for ext in ("*.whl", "*.egg"):
                    matches = sorted(dist_dir.glob(ext))
                    if matches:
                        self._carla_egg = str(matches[0])
                        break

        if self._carla_egg:
            entries.append(str(self._carla_egg))

        for entry in entries:
            if entry and entry not in sys.path:
                sys.path.insert(0, entry)

        try:
            import carla  # type: ignore
            from agents.navigation.behavior_agent import (  # type: ignore
                BehaviorAgent,
            )
            from agents.navigation.basic_agent import BasicAgent  # type: ignore
            from agents.navigation.constant_velocity_agent import (  # type: ignore
                ConstantVelocityAgent,
            )
        except Exception as e:
            raise RuntimeError("CARLA Python API/agents not available") from e

        self._carla = carla
        self._BehaviorAgent = BehaviorAgent
        self._BasicAgent = BasicAgent
        self._ConstantVelocityAgent = ConstantVelocityAgent

    def _connect(self) -> None:
        self._ensure_carla_imports()
        if self._client is not None:
            return
        client = self._carla.Client(self._host, self._port)
        client.set_timeout(self._timeout)
        self._client = client

    def _find_ego_vehicle_once(self):
        if self._world is None:
            return None
        actors = self._world.get_actors().filter("vehicle.*")
        for actor in actors:
            role = actor.attributes.get("role_name", "")
            if role == self._ego_role_name:
                return actor
        return None

    def _wait_for_ego(self, timeout_s: float):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            logger.info(
                "Searching for ego vehicle with role_name=%r...", self._ego_role_name
            )
            actor = self._find_ego_vehicle_once()
            if actor is not None:
                return actor
            time.sleep(0.05)
        return None

    def _to_carla_location(self, pos) -> Any:
        y = float(pos.y) * self._yaw_sign
        return self._carla.Location(
            x=float(pos.x),
            y=y,
            z=float(pos.z),
        )

    def _get_target_speed_kmh(self, sps: ScenarioPack) -> float:
        speed = self._target_speed
        if speed is None:
            speed = sps.ego.target_speed
        speed = float(speed)
        if self._target_speed_is_mps:
            speed = speed * 3.6
        return speed

    def _build_agent(self, target_speed_kmh: float):
        if self._vehicle is None:
            return None

        if self._agent_type == "behavior":
            agent = self._BehaviorAgent(
                self._vehicle, behavior=self._behavior, map_inst=self._map
            )
        elif self._agent_type == "basic":
            agent = self._BasicAgent(self._vehicle, map_inst=self._map)
        elif self._agent_type in ("constant_velocity", "constant-velocity"):
            agent = self._ConstantVelocityAgent(
                self._vehicle, target_speed=target_speed_kmh, map_inst=self._map
            )
        else:
            raise ValueError(f"Unsupported agent_type: {self._agent_type}")

        agent.set_target_speed(target_speed_kmh)
        if hasattr(agent, "follow_speed_limits"):
            agent.follow_speed_limits(self._follow_speed_limits)
        if hasattr(agent, "ignore_traffic_lights"):
            agent.ignore_traffic_lights(self._ignore_traffic_lights)
        if hasattr(agent, "ignore_stop_signs"):
            agent.ignore_stop_signs(self._ignore_stop_signs)
        if hasattr(agent, "ignore_vehicles"):
            agent.ignore_vehicles(self._ignore_vehicles)
        return agent

    def init(self, runtime_spec: dict, sps: ScenarioPack) -> None:
        self._sps = sps
        self._connect()
        logger.info("CarlaAgentAV initialized.")

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
        self._ensure_world(sps)
        self._vehicle = None
        logger.info("Ego vehicle found: %s", self._vehicle)
        if self._vehicle is None:
            self._vehicle = self._spawn_ego(init_obs, self._sps)

        time.sleep(3.0)
        self._apply_world_settings()
        # self._refresh_world()
        # self._vehicle = self._wait_for_ego(self._max_wait_sec)
        # raise RuntimeError(
        #     f"Ego vehicle with role_name={self._ego_role_name!r} not found."
        # )

        target_speed_kmh = self._get_target_speed_kmh(sps)
        self._agent = self._build_agent(target_speed_kmh)
        if self._agent is None:
            raise RuntimeError("Failed to create CARLA agent")

        if self._random_destination:
            if self._map is None:
                raise RuntimeError("CARLA map not available for destination picking")
            spawn_points = self._map.get_spawn_points()
            if not spawn_points:
                raise RuntimeError("No spawn points available for random destination")
            dest = random.choice(spawn_points).location
        else:
            dest = self._to_carla_location(sps.ego.goal.position)
        dest.z += self._spawn_z_offset  # to avoid underground issues
        import carla

        end_wp = self._map.get_waypoint(
            dest, project_to_road=True, lane_type=carla.LaneType.Driving
        )
        print(f"Setting agent destination to {dest}")
        self._agent.set_destination(end_wp.transform.location)

        return self.step(
            obs=init_obs if init_obs is not None else [],
            time_stamp_ns=0,
        )

    def step(self, obs: list[ObjectState], time_stamp_ns: int) -> Ctrl:
        # if self._agent is None:
        #     return Ctrl(mode=CtrlMode.None_, payload={})
        self._update_and_tick(obs)
        control = self._agent.run_step()
        if hasattr(self._agent, "done") and self._agent.done():
            self._quit_flag = True

        yaw_sign = self._yaw_sign if abs(self._yaw_sign) > 1e-6 else 1.0
        steer_sv = float(control.steer) / yaw_sign

        return Ctrl(
            mode=CtrlMode.THROTTLE_STEER_BREAK,
            payload={
                "throttle": float(control.throttle),
                "brake": float(control.brake),
                "steer": steer_sv,
            },
        )

    def stop(self) -> None:
        if self._world is not None and self._original_settings is not None:
            try:
                self._world.apply_settings(self._original_settings)
                logger.info("Restored original CARLA world settings.")
            except Exception as e:
                logger.warning(f"Failed to restore CARLA world settings: {e}")
        self._agent = None
        self._vehicle = None
        self._world = None
        self._map = None
        self._quit_flag = True

    def should_quit(self) -> bool:
        return self._quit_flag

    def _spawn_ego(self, init_obs: Optional[list[ObjectState]], sps: ScenarioPack):
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

        # pos = sps.ego.spawn.position
        pos = init_obs[0].kinematic if init_obs else sps.ego.spawn.position
        carla_pos = self._to_carla_location(pos)
        carla_pos.z += (
            self._spawn_z_offset
        )  # to avoid spawning underground due to map height issues
        carla_rot = self._carla.Rotation(
            pitch=0.0,
            yaw=self._to_carla_yaw(float(pos.yaw)),
            roll=0.0,
        )
        transform = self._carla.Transform(carla_pos, carla_rot)
        print(f"Spawning ego at {carla_pos} with yaw {pos.yaw}")
        ego = self._world.try_spawn_actor(ego_bp, transform)
        if ego is None:
            logger.warning("Initial spawn failed, trying spawn points...")
            spawn_points = self._world.get_map().get_spawn_points()
            if not spawn_points:
                raise RuntimeError("Failed to spawn ego vehicle (no spawn points)")
            ego = self._world.try_spawn_actor(ego_bp, spawn_points[0])
            if ego is None:
                raise RuntimeError("Failed to spawn ego vehicle")

        try:
            phys = ego.get_physics_control()
            max_steer = max([w.max_steer_angle for w in phys.wheels])
            self._max_steer_rad = math.radians(max_steer)
        except Exception:
            self._max_steer_rad = None

        logger.info(f"Ego vehicle spawned at {carla_pos} with yaw {pos.yaw}")
        return ego

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
        self._map = world.get_map() if world else None
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

    def _update_and_tick(self, obs: list[ObjectState]) -> None:
        if self._world is None:
            return

        def pick_blueprint(obj_type: RoadObjectType):
            if self._world is None:
                return None
            bp_lib = self._world.get_blueprint_library()
            if obj_type == RoadObjectType.PEDESTRIAN:
                candidates = bp_lib.filter("walker.pedestrian.*")
            elif obj_type == RoadObjectType.BUS:
                candidates = bp_lib.filter("vehicle.*bus*")
            elif obj_type == RoadObjectType.TRUCK:
                candidates = bp_lib.filter("vehicle.*truck*")
            elif obj_type == RoadObjectType.TRAILER:
                candidates = bp_lib.filter("vehicle.*trailer*")
            elif obj_type == RoadObjectType.VAN:
                candidates = bp_lib.filter("vehicle.*van*")
            elif obj_type == RoadObjectType.MOTORCYCLE:
                candidates = bp_lib.filter("vehicle.*motorcycle*")
            elif obj_type == RoadObjectType.BICYCLE:
                candidates = bp_lib.filter("vehicle.*bicycle*")
                if not candidates:
                    candidates = bp_lib.filter("vehicle.*bike*")
            else:
                candidates = bp_lib.filter("vehicle.*")
            if not candidates and obj_type != RoadObjectType.PEDESTRIAN:
                candidates = bp_lib.filter("vehicle.*")
            if not candidates:
                return None
            return candidates[0]

        def make_transform(kin, z_offset: float = 0.0):
            loc = self._to_carla_location(kin)
            if z_offset:
                loc.z += z_offset
            rot = self._carla.Rotation(
                pitch=0.0,
                yaw=self._to_carla_yaw(float(kin.yaw)),
                roll=0.0,
            )
            return self._carla.Transform(loc, rot)

        def apply_kinematic(actor, kin) -> None:
            if actor is None:
                return
            try:
                actor.set_transform(make_transform(kin))
            except Exception:
                logger.exception("Failed to set actor transform")

            speed = float(kin.speed)
            yaw_carla_deg = self._to_carla_yaw(float(kin.yaw))
            yaw_carla_rad = math.radians(yaw_carla_deg)
            vx = speed * math.cos(yaw_carla_rad)
            vy = speed * math.sin(yaw_carla_rad)
            vel = self._carla.Vector3D(vx, vy, 0.0)
            try:
                actor.set_target_velocity(vel)
            except Exception:
                try:
                    actor.set_velocity(vel)
                except Exception:
                    pass

            if abs(float(kin.yaw_rate)) > 1e-6:
                ang_z = math.degrees(float(kin.yaw_rate)) * self._yaw_sign
                ang = self._carla.Vector3D(0.0, 0.0, ang_z)
                try:
                    actor.set_target_angular_velocity(ang)
                except Exception:
                    try:
                        actor.set_angular_velocity(ang)
                    except Exception:
                        pass

        if not obs:
            if self._sync:
                self._world.tick()
            else:
                self._world.wait_for_tick()
            return

        if self._vehicle is None:
            self._vehicle = self._spawn_ego(obs, self._sps)

        ego_state = obs[0].kinematic
        apply_kinematic(self._vehicle, ego_state)

        desired_count = max(len(obs) - 1, 0)
        while len(self._other_actors) < desired_count:
            self._other_actors.append(None)
            self._other_actor_types.append(RoadObjectType.UNKNOWN)
        while len(self._other_actors) > desired_count:
            actor = self._other_actors.pop()
            self._other_actor_types.pop()
            if actor is not None:
                try:
                    actor.destroy()
                except Exception:
                    logger.exception("Failed to destroy extra actor")

        for idx, obj in enumerate(obs[1:]):
            actor = self._other_actors[idx]
            obj_type = obj.type
            if (
                actor is None
                or (hasattr(actor, "is_alive") and not actor.is_alive)
                or self._other_actor_types[idx] != obj_type
            ):
                if actor is not None:
                    try:
                        actor.destroy()
                    except Exception:
                        logger.exception("Failed to destroy actor %s", idx)
                bp = pick_blueprint(obj_type)
                if bp is None:
                    logger.warning("No blueprint for object type %s", obj_type)
                    self._other_actors[idx] = None
                    self._other_actor_types[idx] = obj_type
                    continue
                if bp.has_attribute("role_name"):
                    bp.set_attribute("role_name", f"agent_{idx}")
                transform = make_transform(obj.kinematic, z_offset=self._spawn_z_offset)
                actor = self._world.try_spawn_actor(bp, transform)
                if actor is None:
                    logger.warning("Failed to spawn actor for index %s", idx)
                self._other_actors[idx] = actor
                self._other_actor_types[idx] = obj_type

            apply_kinematic(self._other_actors[idx], obj.kinematic)

        if self._sync:
            self._world.tick()
        else:
            self._world.wait_for_tick()
