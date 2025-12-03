from typing import Protocol, Any, Optional, Iterable


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


TestResult = dict[str, Any]
ParamDict = dict[str, Any]


class Sampler:
    def next(self, past_results=None) -> Optional[ParamDict]:
        raise NotImplementedError

    def total_permutations(self) -> Optional[int]:
        """
        回傳「理論上」總共有幾組 sample。
        - Grid Search 這種離散 sampler 可以回傳具體數字
        - Random / Adaptive 這種可能回傳 None（代表不適用 / 無界）
        """
        return None

    # def remaining_permutations(self) -> Optional[int]:
    #     """
    #     回傳「還剩多少組沒跑」。
    #     預設 None，同 total_permutations 不適用時也可傳 None。
    #     """
    #     return None
