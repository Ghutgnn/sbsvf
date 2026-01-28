# sv/runner.py
from time import sleep, time
from typing import Any, Optional
import logging
from pathlib import Path
import importlib
import yaml

from sv.interface import Sim, AV, Bridge, Monitor  # Protocols
from sv.utils.control import Ctrl
from sv.utils.sps import ScenarioPack

from sv.registry import (
    build_instance_from_registry,
    SAMPLER_REGISTRY,
    SIM_REGISTRY,
    AV_REGISTRY,
    BRIDGE_REGISTRY,
    MONITOR_REGISTRY,
)


logger = logging.getLogger(__name__)


class Runner:
    def __init__(
        self,
        runtime_cfg: dict,
        plan_name: str,
        sim_cfg: dict,
        av_cfg: dict,
        bridge_cfg: dict,
        monitor_cfg: dict,
        sampler_cfg: dict,
        sps: ScenarioPack,
    ):
        self.plan_name = plan_name
        self.runtime_cfg = runtime_cfg
        self.sps = sps

        # SIM
        module = importlib.import_module(sim_cfg["module"].split(":")[0])
        sim_class = getattr(module, sim_cfg["module"].split(":")[1])
        self.sim = sim_class(cfg_path=sim_cfg.get("cfg_path", None))

        # AV
        module = importlib.import_module(av_cfg["module"].split(":")[0])
        av_class = getattr(module, av_cfg["module"].split(":")[1])
        self.av = av_class(cfg_path=av_cfg.get("cfg_path", None))

        # Bridge
        module = importlib.import_module(bridge_cfg["module"].split(":")[0])
        bridge_class = getattr(module, bridge_cfg["module"].split(":")[1])
        self.bridge = bridge_class(cfg_path=bridge_cfg.get("cfg_path", None))

        # Monitor
        module = importlib.import_module(monitor_cfg["module"].split(":")[0])
        monitor_class = getattr(module, monitor_cfg["module"].split(":")[1])
        self.monitor = monitor_class(
            cfg_path=monitor_cfg.get("cfg_path", None),
            plan_name=plan_name,
        )

        if self.sps.param_range_file is not None:
            logger.info("Parameter range file provided: %s", self.sps.param_range_file)
            # param_sampler
            module = importlib.import_module(sampler_cfg["module"].split(":")[0])
            sampler_class = getattr(module, sampler_cfg["module"].split(":")[1])
            self.param_sampler = sampler_class(
                param_range_file=self.sps.param_range_file,
                past_results=None,
            )
        else:
            logger.info(
                "No parameter range file provided; seem as testing a concrete scenario; skipping parameter sampler."
            )
            self.param_sampler = None

    def exec(self) -> None:
        sim_ok = False
        av_ok = False

        try:
            # --- init ---
            try:
                self.sim.init(self.sps)
                sim_ok = True
            except Exception:
                logger.exception("Simulator initialization failed")
                return

            try:
                self.av.init(self.sps)
                av_ok = True
            except Exception:
                logger.exception("AV initialization failed")
                return

            # --- run ---
            if self.param_sampler is not None:
                logger.info("Starting parameter sampling execution.")
                total = self.param_sampler.total_permutations()

                logger.info(f"Total parameter combinations: {total}")

                for i in range(total):
                    logger.info(f"Sampling iteration {i+1}/{total}")
                    params = self.param_sampler.next()

                    if params is None:
                        logger.info("Parameter sampling completed.")
                        break

                    logger.info(f"Running scenario with parameters: {params}")

                    try:
                        self.run_concrete(self.runtime_cfg, self.sps, params)
                    except Exception:
                        logger.exception(f"Scenario failed at iteration {i+1}")
                        continue
            else:
                logger.info("Running a single concrete scenario.")
                try:
                    self.run_concrete(self.runtime_cfg, self.sps)
                except Exception:
                    logger.exception("Scenario failed")

            logger.info("Runner execution completed.")

        finally:
            if av_ok:
                try:
                    self.av.stop()
                except Exception:
                    logger.exception("av.stop() failed")
            if sim_ok:
                try:
                    self.sim.stop()
                except Exception:
                    logger.exception("sim.stop() failed")

    def run_concrete(
        self,
        runtime_cfg: dict,
        sps: ScenarioPack,
        params: Optional[dict[str, Any]] = None,
    ) -> None:
        raw_obs = None
        try:
            raw_obs = self.sim.reset(sps, params)
        except Exception as e:
            logger.error(f"Simulator reset failed: {e}")
            return

        try:
            obs_for_av = self.bridge.sim_to_av(raw_obs)
        except Exception as e:
            logger.error(f"Bridge sim_to_av failed: {e}")
            return

        try:
            ctrl_from_av = self.av.reset(sps, obs_for_av)
        except Exception as e:
            logger.error(f"AV reset failed: {e}")
            return
        try:
            ctrl_for_sim = self.bridge.av_to_sim(ctrl_from_av)
        except Exception as e:
            logger.error(f"Bridge av_to_sim failed: {e}")
            return

        dt_s = runtime_cfg.get("dt", -1)
        dt_ns = int(dt_s * 1e9)

        use_real_time = False
        if dt_ns <= 0:  # use real-time stepping
            dt_ns = 0
            use_real_time = True
            prev = time()

        sim_time_ns = 0  # Simulation time in nanoseconds
        # ctrl_for_sim: Ctrl = Ctrl()
        try:
            real_start_time_s = time()
            while True:
                loop_start_time = time()

                if self.sim.should_quit():
                    logger.info("Simulator requested to quit.")
                    break
                elif self.av.should_quit():
                    logger.info("AV requested to quit.")
                    break

                if use_real_time:
                    t = time()
                    dt_ns = int((t - prev) * 1e9)
                    prev = t

                raw_obs = self.sim.step(ctrl_for_sim, sim_time_ns)
                obs_for_av = self.bridge.sim_to_av(raw_obs)
                ctrl_from_av = self.av.step(obs_for_av, sim_time_ns)
                ctrl_for_sim = self.bridge.av_to_sim(ctrl_from_av)
                sim_time_ns += dt_ns

                cur_time_s = time()
                time_use_s = cur_time_s - real_start_time_s

                loop_need_time = time() - loop_start_time
                sleep_time_s = dt_s - loop_need_time
                if sleep_time_s > 0:
                    sleep(sleep_time_s)

                print(
                    f"time use = {time_use_s:.2f} s, sim_time = {sim_time_ns / 1e9:.2f} s",
                    end="  \r",
                )

            sim_time_need = time() - real_start_time_s
            # self.monitor.finalize()
        except Exception as e:
            logger.error(f"Error during scenario execution: {e}")
            return
        logger.info(
            f"Completed {sim_time_ns / 1e9:.2f} seconds scenario, using {sim_time_need:.2f} sec."
        )
        logger.info("Scenario finished.")


# def load_plan(plan_path: str):
#     with open(plan_path, "r", encoding="utf-8") as f:
#         plan_cfg = yaml.safe_load(f)

#     ret = {}

#     # 建立 simulator
#     sim_name = plan_cfg["adapters"]["sim"]
#     sim_module = f"sv.sim.{sim_name}"
#     importlib.import_module(sim_module)
#     sim = build_instance_from_registry(SIM_REGISTRY, name=sim_name)
#     ret["sim"] = sim

#     # 建立 AV
#     av_name = plan_cfg["adapters"]["av"]
#     av_module = f"sv.av.{av_name}"
#     importlib.import_module(av_module)
#     av = build_instance_from_registry(AV_REGISTRY, name=av_name)
#     ret["av"] = av

#     # 建立 Bridge
#     bridge_name = plan_cfg["adapters"]["bridge"]
#     bridge_module = f"sv.bridge.{bridge_name}"
#     importlib.import_module(bridge_module)
#     bridge = build_instance_from_registry(BRIDGE_REGISTRY, name=bridge_name)
#     ret["bridge"] = bridge

#     # 建立 Monitor
#     monitor_name = plan_cfg["adapters"]["monitor"]
#     monitor_module = f"sv.monitor.{monitor_name}"
#     importlib.import_module(monitor_module)
#     monitor = build_instance_from_registry(MONITOR_REGISTRY, name=monitor_name)
#     ret["monitor"] = monitor

#     ret["runtime_cfg"] = plan_cfg.get("runtime", {})
#     ret["mode"] = plan_cfg.get("mode", "A")
#     ret["artifacts_cfg"] = plan_cfg.get("artifacts", {})
#     ret["scenarios"] = plan_cfg.get("scenario", {})
#     # scenario_state = plan_cfg["scenario"]["state"]
#     # sim_cfg = plan_cfg["scenario"].get("sim_config", {})
#     # av_meta = plan_cfg["scenario"].get("av_metadata", {})
#     # sim_time_cfg = plan_cfg["simulation_time"]
#     # ret["scenario_state"] = scenario_state
#     # ret["sim_cfg"] = sim_cfg
#     # ret["av_meta"] = av_meta
#     # ret["sim_time_cfg"] = sim_time_cfg

#     return ret
