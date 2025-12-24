import logging
from typing import Any, Optional
from pathlib import Path
import ctypes as ct

from sv.registry import register_av
from sv.utils.util import get_cfg
from sv.utils.sps import ScenarioPack
from sv.utils.control import Ctrl, CtrlMode

import rclpy

logger = logging.getLogger(__name__)


@register_av("autoware_mode")
class AutowareAV:
    def __init__(self, cfg_path: Path, sps: Any):
        self.cfg = get_cfg(Path(cfg_path))

    def init(self): ...

    def reset(self, sps: ScenarioPack, params: Optional[dict] = None) -> None: ...

    def step(self, obs: dict[str, Any], dt: float) -> Ctrl: ...

    def stop(self) -> None: ...

    def should_quit(self) -> bool:
        return not self.running
