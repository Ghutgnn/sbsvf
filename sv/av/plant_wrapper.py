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


class PlantAgent:
    def __init__(self, output_dir: Path, cfg_path: Path):
        self._output_dir = output_dir
        self.cfg = get_cfg(cfg_path)
        self._agent_path = Path(self.cfg.get("agent_file_path")).resolve()
        self._agent_cfg_path = Path(self.cfg.get("cfg_path")).resolve()
        self._agent_class_name = self.cfg.get("agent_class_name", "PlanTAgent")
        self._sps: Optional[ScenarioPack] = None
        self._client = None
        self._world = None
        self._map = None
        self._ego_vehicle = None
        self._agent = None
        self._quit_flag = False

    def init(self, runtime_spec: dict, sps: ScenarioPack) -> None:
        self._sps = sps
        agent_dir = str(self._agent_path.parent)
        if agent_dir not in sys.path:
            sys.path.insert(0, agent_dir)

        module_name = self._agent_path.stem  # "PlanT_agent"

        try:
            if module_name in sys.modules:
                del sys.modules[module_name]

            spec = importlib.util.spec_from_file_location(
                module_name, str(self._agent_path)
            )
            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            spec.loader.exec_module(module)

            # 5. Instantiate the agent class
            self._ego_vehicle = getattr(module, self._agent_class_name)(
                path_to_conf_file=self._agent_cfg_path
            )
            logger.info(
                f"Successfully loaded {self._agent_class_name} from {self._agent_path}"
            )

        except Exception as e:
            logger.error(f"Failed to load {self._agent_class_name}: {e}")
            raise ImportError(
                f"Unable to execute {self._agent_class_name} code: {self._agent_path}"
            ) from e

    def reset(
        self,
        output_dir: Path,
        sps: ScenarioPack,
        init_obs: Optional[list[ObjectState]] = None,
    ) -> Ctrl:
        return Ctrl(mode=CtrlMode.None_)
        # self._output_dir = output_dir
        # self._sps = sps
        # self._quit_flag = False

        # self._connect()
        # self._world = self._client.get_world()
        # self._map = self._world.get_map() if self._world else None

        # self._destroy_sensors()
        # self._destroy_agent()

        # self._ego_vehicle = self._wait_for_ego(self._max_wait_sec)
        # if self._ego_vehicle is None:
        #     raise RuntimeError(
        #         f"Ego vehicle with role_name={self._ego_role_name!r} not found."
        #     )

        # self._agent = self._build_agent()
        # self._prepare_agent_plan()
        # self._setup_sensors()

    def step(self, obs: Any, time_stamp_ns: int) -> Ctrl:

        return Ctrl(mode=CtrlMode.THROTTLE_STEER, payload={"pedal": 1, "wheel": 0.0})
        # if self._agent is None:
        #     return Ctrl(mode=CtrlMode.None_)

        # input_data = self._build_input_data(obs, time_stamp_ns)
        # timestamp_s = float(time_stamp_ns) / 1e9

        # try:
        #     if hasattr(self._agent, "run_step"):
        #         control = self._agent.run_step(input_data, timestamp_s)
        #     else:
        #         control = self._agent(input_data, timestamp_s)
        # except Exception:
        #     logger.exception("Leaderboard agent run_step failed.")
        #     self._quit_flag = True
        #     return Ctrl(mode=CtrlMode.None_)

        # return self._control_to_ctrl(control)

    def stop(self) -> None:
        # self._destroy_sensors()
        # self._destroy_agent()
        pass

    def should_quit(self) -> bool:
        # if self._quit_flag:
        #     return True
        # if self._agent is None:
        #     return False
        # if hasattr(self._agent, "done"):
        #     try:
        #         return bool(self._agent.done())
        #     except Exception:
        #         return False
        # if hasattr(self._agent, "is_done"):
        #     try:
        #         return bool(self._agent.is_done())
        #     except Exception:
        #         return False
        # return False
        return False

    def _ensure_plant_import(self):
        if self._plant_home is None or not Path(self._plant_home).is_dir():
            raise ValueError(f"無效的 plant_home: {self._plant_home}")
