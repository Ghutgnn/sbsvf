from typing import Any
from sv.registry import register_monitor
from sv.utils.sps import ScenarioPack


@register_monitor("default")
class defaultMonitor:
    def __init__(self, cfg_path: dict, plan_name: str, sps: ScenarioPack) -> None:
        self.plan_name = plan_name
        self.scenario_name = sps.name
        self.record_type = ["rosbag", "video", "state"]
        self.output_dir = f"artifacts/{plan_name}/{sps.name}/"

    def on_tick(
        self,
        t: float,
        obs: dict[str, Any],
        ctrl: dict[str, Any],
        events: list[dict[str, Any]],
    ) -> None: ...
    def finalize(self) -> None: ...
