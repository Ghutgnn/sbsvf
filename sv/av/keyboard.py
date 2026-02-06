import logging
from typing import Any, Optional
from pathlib import Path
import ctypes as ct
from pynput import keyboard

from sv.registry import register_av
from sv.utils.object import ObjectState
from sv.utils.util import get_cfg
from sv.utils.sps import ScenarioPack
from sv.utils.control import Ctrl, CtrlMode


logger = logging.getLogger(__name__)


class KeyboardAV:
    def __init__(self, output_dir: Path, cfg_path: Path):
        self.output_dir = output_dir
        # self.cfg = get_cfg(Path(cfg_path))

    def init(self, runtime_spec: dict, sps: ScenarioPack) -> None:
        """Start keyboard listener in background thread."""

        def on_press(key):
            try:
                if key == keyboard.Key.up:
                    self.pedal = 1
                elif key == keyboard.Key.down:
                    self.pedal = -1
                elif key == keyboard.Key.right:
                    self.wheel = -1
                elif key == keyboard.Key.left:
                    self.wheel = 1
            except Exception:
                pass

        def on_release(key):
            try:
                if key in (keyboard.Key.up, keyboard.Key.down):
                    self.pedal = 0
                if key in (keyboard.Key.right, keyboard.Key.left):
                    self.wheel = 0

                # 如果你想用 ESC 結束 AV，可以這樣：
                if key == keyboard.Key.f9:
                    self.running = False
                    return False
            except Exception:
                pass

        self.listener = keyboard.Listener(
            on_press=on_press,
            on_release=on_release,
        )
        self.listener.start()
        self.running = True

    def reset(
        self,
        output_dir: Path,
        sps: ScenarioPack,
        init_obs: Optional[list[ObjectState]] = None,
    ) -> None:
        """Reset AV internal state (e.g. when simulator resets)."""
        self.pedal = 0
        self.wheel = 0
        self.running = True

    def step(self, obs: dict[str, Any], dt: float) -> Ctrl:
        """
        obs: observation dict from simulator.step()
        dt: timestep (float)

        Returns ctrl dict:
            {"pedal": -1|0|1, "wheel": -1|0|1}
        """
        return Ctrl(
            mode=CtrlMode.THROTTLE_STEER,
            payload={"pedal": self.pedal, "wheel": self.wheel},
        )

    def stop(self) -> None:
        """Stop keyboard listener and clean up."""
        if self.listener is not None:
            logger.info("Stopping keyboard listener...")
            self.listener.stop()
            self.listener = None
        self.running = False

    def should_quit(self) -> bool:
        return not self.running
