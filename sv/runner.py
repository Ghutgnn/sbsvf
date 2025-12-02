# sv/runner.py
from time import sleep
from typing import Any
import logging
from sv.interface import Sim, AV, Bridge, Monitor  # Protocols
from sv.utils.control import Ctrl
from sv.utils.sps import ScenarioPack
from pathlib import Path
import yaml
from sv.registry import (
    build_instance_from_registry,
    SIM_REGISTRY,
    AV_REGISTRY,
    BRIDGE_REGISTRY,
    MONITOR_REGISTRY,
)

import importlib

logger = logging.getLogger(__name__)


class Runner:
    def __init__(
        self,
        plan_name: str,
        sim_cfg: dict,
        av_cfg: dict,
        bridge_cfg: dict,
        monitor_cfg: dict,
        sps: ScenarioPack,
    ):
        self.plan_name = plan_name
        self.sps = sps
        self.sim = build_instance_from_registry(
            SIM_REGISTRY,
            name=sim_cfg["name"],
            cfg_path=sim_cfg.get("cfg_path", {}),
            sps=sps,
        )
        self.av = build_instance_from_registry(
            AV_REGISTRY,
            name=av_cfg["name"],
            cfg_path=av_cfg.get("cfg_path", {}),
            sps=sps,
        )
        self.bridge = build_instance_from_registry(
            BRIDGE_REGISTRY,
            name=bridge_cfg["name"],
            cfg_path=bridge_cfg.get("cfg_path", {}),
            sps=sps,
        )
        self.monitor = build_instance_from_registry(
            MONITOR_REGISTRY,
            name=monitor_cfg["name"],
            cfg_path=monitor_cfg.get("cfg_path", {}),
            plan_name=plan_name,
            sps=sps,
        )

    def exec(self) -> None:
        # for sce_name, sce_params in self.scenarios.items():
        #     logger.info(f"Running concrete scenario: {sce_name}")
        #     self.run_concrete(sce_params)
        self.sim.init()
        self.av.init()

        self.run_concrete(self.sps)

    def run_concrete(self, sps: ScenarioPack) -> None:
        self.sim.reset(sps)
        self.av.reset(sps)
        t = 0.0
        dt = 0.00625
        ctrl_for_sim: Ctrl = Ctrl()

        while True:
            if self.sim.should_quit():
                logger.info("Simulator requested to quit.")
                break
            elif self.av.should_quit():
                logger.info("AV requested to quit.")
                break
            raw_obs = self.sim.step(ctrl_for_sim, dt)
            obs_for_av = self.bridge.sim_to_av(raw_obs)
            ctrl_from_av = self.av.step(obs_for_av, dt)
            ctrl_for_sim = self.bridge.av_to_sim(ctrl_from_av)
            # evs = self.sim.events()

            # self.monitor.on_tick(t, obs_for_av, ctrl_from_av, evs)
            # t += dt

        # self.monitor.finalize()
        self.av.stop()
        self.sim.stop()

        logger.info("Scenario finished.")


def load_plan(plan_path: str):
    with open(plan_path, "r", encoding="utf-8") as f:
        plan_cfg = yaml.safe_load(f)

    ret = {}

    # 建立 simulator
    sim_name = plan_cfg["adapters"]["sim"]
    sim_module = f"sv.sim.{sim_name}"
    importlib.import_module(sim_module)
    sim = build_instance_from_registry(SIM_REGISTRY, name=sim_name)
    ret["sim"] = sim

    # 建立 AV
    av_name = plan_cfg["adapters"]["av"]
    av_module = f"sv.av.{av_name}"
    importlib.import_module(av_module)
    av = build_instance_from_registry(AV_REGISTRY, name=av_name)
    ret["av"] = av

    # 建立 Bridge
    bridge_name = plan_cfg["adapters"]["bridge"]
    bridge_module = f"sv.bridge.{bridge_name}"
    importlib.import_module(bridge_module)
    bridge = build_instance_from_registry(BRIDGE_REGISTRY, name=bridge_name)
    ret["bridge"] = bridge

    # 建立 Monitor
    monitor_name = plan_cfg["adapters"]["monitor"]
    monitor_module = f"sv.monitor.{monitor_name}"
    importlib.import_module(monitor_module)
    monitor = build_instance_from_registry(MONITOR_REGISTRY, name=monitor_name)
    ret["monitor"] = monitor

    ret["runtime_cfg"] = plan_cfg.get("runtime", {})
    ret["mode"] = plan_cfg.get("mode", "A")
    ret["artifacts_cfg"] = plan_cfg.get("artifacts", {})
    ret["scenarios"] = plan_cfg.get("scenario", {})
    # scenario_state = plan_cfg["scenario"]["state"]
    # sim_cfg = plan_cfg["scenario"].get("sim_config", {})
    # av_meta = plan_cfg["scenario"].get("av_metadata", {})
    # sim_time_cfg = plan_cfg["simulation_time"]
    # ret["scenario_state"] = scenario_state
    # ret["sim_cfg"] = sim_cfg
    # ret["av_meta"] = av_meta
    # ret["sim_time_cfg"] = sim_time_cfg

    return ret
