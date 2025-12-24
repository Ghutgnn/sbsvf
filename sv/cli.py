from pathlib import Path
import typer
import logging

import yaml

# from sv.lint import lint_scenario
from sv.report import make_report
from sv.manager import run_plan, run_plan_docker

from rich.logging import RichHandler

from sv.runner import Runner
from sv.utils.sps import ScenarioPack

app = typer.Typer(add_completion=False)


@app.callback()
def main(
    log_level: str = typer.Option(
        "INFO",
        "--log-level",
        "-l",
        help="Set log level (DEBUG, INFO, WARNING, ERROR, CRITICAL)",
        case_sensitive=False,
    )
):
    """全域設定，例如 logging。"""
    level = getattr(logging, log_level.upper(), logging.INFO)
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="[%X]",
        handlers=[RichHandler(rich_tracebacks=True)],
    )
    logging.info(f"Logging level set to {log_level.upper()}")


# @app.command()
# def lint(path: str):
#     lint_scenario(path)


# python -m sv.cli run ./plans/docker_test.yaml
@app.command()
def run(plan: str):
    run_plan(plan)


@app.command()
def run_docker(plan_path: str) -> None:
    run_plan_docker(plan_path)


@app.command()
def run_logical(
    plan: str = typer.Argument(..., help="Path to test plan yaml"),
    scenario: str = typer.Argument(..., help="Path to ONE logical scenario yaml"),
):
    """
    只跑一個 logical scenario。

    - 會 new 一個 Runner
    - Runner 裡會用 sampler 不斷產生 params → concrete scenario
    - 跑完所有 sampling 後退出
    """
    with open(plan, "r", encoding="utf-8") as f:
        plan_cfg = yaml.safe_load(f)

    # logical scenario -> ScenarioPack
    sps = ScenarioPack.from_yaml(scenario)

    runner = Runner(
        plan_name=plan_cfg["name"],
        sim_cfg=plan_cfg["simulator"],
        av_cfg=plan_cfg["av"],
        bridge_cfg=plan_cfg["bridge"],
        monitor_cfg=plan_cfg["monitor"],
        sampler_cfg=plan_cfg.get("sampler", {}),
        sps=sps,
    )

    # 這裡直接用你原本的 exec()
    # exec() 裡會：
    # - sim.init()
    # - av.init()
    # - 若有 param_range_file -> 建 sampler & total_permutations()
    # - for params in sampler: run_concrete(...)
    # - 最後 av.stop(), sim.stop()
    runner.exec()


# @app.command()
# def report(run_id: str = "last", out: str = "out"):
#     make_report(run_id, out)


if __name__ == "__main__":
    app()
