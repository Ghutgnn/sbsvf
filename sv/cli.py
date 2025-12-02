import typer
import logging

# from sv.lint import lint_scenario
from sv.report import make_report
from sv.manager import run_plan

from rich.logging import RichHandler

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


@app.command()
def run(plan: str):
    run_plan(plan)


@app.command()
def report(run_id: str = "last", out: str = "out"):
    make_report(run_id, out)


if __name__ == "__main__":
    app()
