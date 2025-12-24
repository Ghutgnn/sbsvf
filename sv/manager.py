from logging import config
import subprocess
import pprint
from typing import Any
import logging
from pathlib import Path
import yaml

from sv.runner import Runner
from sv.utils.sps import ScenarioPack
from concurrent.futures import ThreadPoolExecutor, as_completed

import sv.sim
import sv.av
import sv.bridge
import sv.monitor
import sv.sampler

logger = logging.getLogger(__name__)


class Manager:
    def __init__(self, config):
        self.config = config
        self.plan_name = config["name"]
        # self._worker_count = config["worker_count"]
        self._sc_cfg_list = config["scenarios"]

    def _run_one_scenario(self, scenario_pack: ScenarioPack):
        logger.info(f"Running scenario: {scenario_pack.name}")
        runner = Runner(
            plan_name=self.plan_name,
            sim_cfg=self.config["simulator"],
            av_cfg=self.config["av"],
            bridge_cfg=self.config["bridge"],
            monitor_cfg=self.config["monitor"],
            sampler_cfg=self.config.get("sampler", {}),
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


class HostManager:
    """
    Host 上的 manager：
    - 不直接 new Runner
    - 對每個 logical scenario 開一個 docker container
    - 容器裡的入口會跑 `run-logical --plan ... --scenario ...`
    """

    def __init__(
        self,
        cfg_path: Path,
        docker_image: str,
        data_root: Path,
    ):
        """
        cfg_path: host 上的 plan yaml 路徑
        docker_image: 要使用的 image 名稱（e.g. "hcis/av-esmini-autoware:2025-12-03"）
        data_root: host 上掛給 container 的根目錄（例如 /home/you/data → /data）
        """
        with open(cfg_path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)
        self.config = config
        self.config_path = cfg_path
        self.plan_name = config["name"]
        self._sc_cfg_list: list[str] = config[
            "scenarios"
        ]  # 這裡建議 scenarios 放「scenario yaml 的路徑字串」
        self.docker_image = docker_image
        self.data_root = data_root

        # 在 container 裡，plan & scenarios 會從 /data 底下看見
        # 假設 host 上你的 plan 跟 scenario 也都放在 data_root 裡（你可以自己 mapping）
        # self.plan_path_in_container = f"/data/{Path(config['plan_path']).name}"

    def _run_one_scenario_docker(self, scenario_cfg_path: str):
        """
        scenario_cfg_path: host 上 logical scenario yaml 的絕對路徑或 data_root 底下的相對路徑
        """
        host_scenario_path = Path(scenario_cfg_path)

        # 容器內看到的是 /data/<filename>
        scenario_path_in_container = f"./{str(host_scenario_path)}"

        logger.info(f"Running scenario in docker: {host_scenario_path}")

        # docker run --rm -e DISPLAY=$DISPLAY --gpus all -it -v $PWD:/workspace -v /tmp/.X11-unix:/tmp/.X11-unix -w /workspace esmini bash

        docker_cmd = [
            "docker",
            "run",
            "--rm",
            "-e",
            "DISPLAY=:1",
            "--gpus",
            "all",
            "--net",
            "host",
            # "-v",
            # f"{self.data_root}:/data",  # host -> container 的 volume 映射
            "-v",
            "/tmp/.X11-unix:/tmp/.X11-unix",
            "-v",
            ".:/workspace",
            self.docker_image,
            # ENTRYPOINT 如果是 "python -m src.cli"，後面就是 Typer command / 或我們的 run_logical
            "python",
            "-m",
            "sv.cli",
            "run-logical",
            # "--plan",
            str(self.config_path),
            # "--scenario",
            str(scenario_path_in_container),
        ]
        pprint.pprint(docker_cmd)
        logger.info("Executing: %s", " ".join(docker_cmd))
        ret = subprocess.run(docker_cmd)
        if ret.returncode != 0:
            logger.error(
                f"Scenario {host_scenario_path} failed with code {ret.returncode}"
            )
        else:
            logger.info(f"Scenario {host_scenario_path} finished.")

    def run_all(self):
        for sc_cfg in self._sc_cfg_list:
            # 這裡假設 config["scenarios"] 裡存的是 host 上的 yaml path
            self._run_one_scenario_docker(sc_cfg)


def run_plan_docker(
    plan_path: str,
    docker_image: str = "esmini",
    data_root: str = "/workspace/data",
) -> None:
    # with open(plan_path, "r", encoding="utf-8") as f:
    #     plan_cfg = yaml.safe_load(f)

    manager = HostManager(
        cfg_path=Path(plan_path),
        docker_image=docker_image,
        data_root=Path(data_root),
    )
    manager.run_all()
