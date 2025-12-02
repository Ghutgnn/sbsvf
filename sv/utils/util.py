from pathlib import Path
import logging
from typing import Union

logger = logging.getLogger(__name__)


def get_cfg(cfg_path: Union[Path, str]) -> dict:
    if isinstance(cfg_path, str):
        cfg_path = Path(cfg_path)

    if not cfg_path.exists():
        raise FileNotFoundError(f"Config file not found: {cfg_path}")
    with open(cfg_path, "r") as f:
        if cfg_path.suffix in [".yaml", ".yml"]:
            import yaml

            logger.debug(f"Loading YAML config from {cfg_path}")
            return yaml.safe_load(f)
        elif cfg_path.suffix == ".json":
            import json

            logger.debug(f"Loading JSON config from {cfg_path}")
            return json.load(f)
        else:
            raise ValueError(f"Unsupported config file format: {cfg_path.suffix}")
