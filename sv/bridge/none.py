from typing import Any
from sv.registry import register_bridge
from sv.utils.control import Ctrl
from sv.utils.sps import ScenarioPack


@register_bridge("none")
class NoneBridge:
    def __init__(self, cfg_path: dict, sps: ScenarioPack):
        pass

    def sim_to_av(self, raw: dict[str, Any]) -> dict[str, Any]:  # RawObs -> Obs
        return raw

    def av_to_sim(self, ctrl: Ctrl) -> Ctrl:  # Ctrl -> SimCtrl
        return ctrl
