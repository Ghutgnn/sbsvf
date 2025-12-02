from typing import Protocol, Any


class Sim(Protocol):
    def reset(self, sps: dict[str, Any], cfg: dict[str, Any]) -> None: ...
    def step(
        self, ctrl: dict[str, Any], dt: float
    ) -> dict[str, Any]: ...  # returns RawObs
    def events(self) -> list[dict[str, Any]]: ...
    def stop(self) -> None: ...


class AV(Protocol):
    def init(self) -> None: ...
    def reset(self) -> None: ...
    def step(self, obs: dict[str, Any]) -> dict[str, Any]: ...  # returns Ctrl
    def stop(self) -> None: ...
    def should_quit(self) -> bool: ...


class Bridge(Protocol):
    def sim_to_av(self, raw: dict[str, Any]) -> dict[str, Any]: ...  # RawObs -> Obs
    def av_to_sim(self, ctrl: dict[str, Any]) -> dict[str, Any]: ...  # Ctrl -> SimCtrl


class Monitor(Protocol):
    def on_tick(
        self,
        t: float,
        obs: dict[str, Any],
        ctrl: dict[str, Any],
        events: list[dict[str, Any]],
    ) -> None: ...
    def finalize(self) -> None: ...
