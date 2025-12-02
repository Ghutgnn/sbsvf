import pprint
from typing import Any
import logging
from pathlib import Path
import yaml

# from simulator import Simulator
# from av import AV
# from bridge import Bridge
# from recorder import Recorder
from sv.runner import Runner
from sv.utils.sps import ScenarioPack
from concurrent.futures import ThreadPoolExecutor, as_completed

# import sv.av.acc
# import sv.sim.esmini

# import sv.av.keyboard

# import sv.bridge.none
# import sv.monitor.default

import sv.sim
import sv.av
import sv.bridge
import sv.monitor

logger = logging.getLogger(__name__)


class Manager:
    def __init__(self, config):
        self.config = config
        self.plan_name = config["name"]
        # self._worker_count = config["worker_count"]
        self._sc_cfg_list = config["scenarios"]

    # def _read_scenarios(self, sc_cfg_list):
    # osc_folder = Path(sc_cfg_list) / "xosc"
    # scenarios = [osc_folder / p.name for p in osc_folder.glob("*.xosc")]
    # logger.info(f"Found {len(scenarios)} logical scenarios in {osc_folder}")
    # return scenarios

    def _run_one_scenario(self, scenario_pack: ScenarioPack):
        logger.info(f"Running scenario: {scenario_pack.name}")
        runner = Runner(
            plan_name=self.plan_name,
            sim_cfg=self.config["simulator"],
            av_cfg=self.config["av"],
            bridge_cfg=self.config["bridge"],
            monitor_cfg=self.config["monitor"],
            sps=scenario_pack,
        )
        runner.exec()

    def run_all(self):
        # TODO: implement parallel execution with ThreadPoolExecutor
        for sc_cfg in self._sc_cfg_list:
            cfg = ScenarioPack.from_yaml(sc_cfg)
            self._run_one_scenario(cfg)
            logger.info(f"Scenario {cfg.name} finished.")


def run_plan(plan_path: str) -> None:
    with open(plan_path, "r", encoding="utf-8") as f:
        plan_cfg = yaml.safe_load(f)

    manager = Manager(plan_cfg)
    manager.run_all()
