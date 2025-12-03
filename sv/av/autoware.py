import logging
from typing import Any, Optional
from pathlib import Path
import ctypes as ct
from pynput import keyboard

from sv.registry import register_av
from sv.utils.util import get_cfg
from sv.utils.sps import ScenarioPack
from sv.utils.control import Ctrl, CtrlMode

logger = logging.getLogger(__name__)


@register_av("autoware")
class AutowareAV:
    def __init__(self, cfg_path: Path, sps: Any):
        self.cfg = get_cfg(Path(cfg_path))

    def init(self):
        """Initialize Autoware AV."""
        logger.info("Autoware AV initialized.")

    def reset(self, sps: ScenarioPack, params: Optional[dict] = None) -> None:
        """Reset AV internal state (e.g. when simulator resets)."""
        logger.info("Autoware AV reset.")

    def step(self, obs: dict[str, Any], dt: float) -> Ctrl:
        """Given observation from simulator, return control commands."""
        logger.debug(f"Autoware AV step with obs: {obs} and dt: {dt}")
        return Ctrl(mode=CtrlMode.AUTO)

    def stop(self) -> None:
        """Clean up resources when AV is no longer needed."""
        logger.info("Autoware AV stopped.")

    def should_quit(self) -> bool:
        """Return True if AV requests to quit the simulation."""
        return False
