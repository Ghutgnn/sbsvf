import logging
import os
import random
import sys
import time
from pathlib import Path
from typing import Any, Optional

from sv.registry import register_av
from sv.utils.control import Ctrl, CtrlMode
from sv.utils.object import ObjectState
from sv.utils.sps import ScenarioPack
from sv.utils.util import get_cfg

logger = logging.getLogger(__name__)


@register_av("carla_agent")
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

        self._ego_role_name = self.cfg.get("ego_role_name", "hero")
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

        self._carla = None
        self._BehaviorAgent = None
        self._BasicAgent = None
        self._ConstantVelocityAgent = None

        self._client = None
        self._world = None
        self._map = None
        self._vehicle = None
        self._agent = None

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

    def _refresh_world(self) -> None:
        if self._client is None:
            return
        self._world = self._client.get_world()
        self._map = self._world.get_map() if self._world else None

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
        self._refresh_world()
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
        self._refresh_world()
        self._vehicle = self._wait_for_ego(self._max_wait_sec)
        if self._vehicle is None:
            raise RuntimeError(
                f"Ego vehicle with role_name={self._ego_role_name!r} not found."
            )

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

        self._agent.set_destination(dest)

    def step(self, obs: list[ObjectState], time_stamp_ns: int) -> Ctrl:
        if self._agent is None:
            return Ctrl(mode=CtrlMode.None_, payload={})

        control = self._agent.run_step()
        if hasattr(self._agent, "done") and self._agent.done():
            self._quit_flag = True

        yaw_sign = self._yaw_sign if abs(self._yaw_sign) > 1e-6 else 1.0
        steer_sv = float(control.steer) / yaw_sign

        return Ctrl(
            mode=CtrlMode.THROTTLE_STEER,
            payload={
                "throttle": float(control.throttle),
                "brake": float(control.brake),
                "steer": steer_sv,
            },
        )

    def stop(self) -> None:
        self._agent = None
        self._vehicle = None
        self._world = None
        self._map = None
        self._quit_flag = True

    def should_quit(self) -> bool:
        return self._quit_flag
