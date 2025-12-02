from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Literal, Tuple, Any, Dict
import yaml

from sv.utils.util import get_cfg
from sv.utils.position import PositionType, Position


@dataclass
class SpawnConfig:
    position: Position
    speed: float

    # def __post_init__(self) -> None:
    #     if self.type == "WorldPosition":
    #         # 這裡你可以依實際需求改成只接受 3 or 6 個數
    #         if len(self.value) not in (3, 6):
    #             raise ValueError(
    #                 f"spawn.type=WorldPosition 時，value 應該是 [x, y, z] 或 [x, y, z, h, p, r]，"
    #                 f"但目前長度是 {len(self.value)}: {self.value}"
    #             )
    #     elif self.type == "LanePosition":
    #         if len(self.value) not in (4, 5):
    #             raise ValueError(
    #                 f"spawn.type=LanePosition 時，value 應該是 [roadId, laneId, s, offset(, orientation)]，"
    #                 f"但目前長度是 {len(self.value)}: {self.value}"
    #             )
    #     else:
    #         raise ValueError(
    #             f"spawn.type 只能是 'WorldPosition' 或 'LanePosition'，目前是 {self.type!r}"
    #         )


# @dataclass
# class CheckPointConfig:
#     type: PositionType
#     value: Tuple[float, ...]
#     tolerance: float
#     speed: float
#     speed_tolerance: Tuple[float, float]
#     time_limit: float
#     time_fatal: bool

#     def __post_init__(self) -> None:
#         if self.type == "WorldPosition":
#             if len(self.value) not in (3, 6):
#                 raise ValueError(
#                     f"check_point(type=WorldPosition) value 應該是 [x, y, z] 或 [x, y, z, h, p, r]，"
#                     f"但目前長度是 {len(self.value)}: {self.value}"
#                 )
#         elif self.type == "LanePosition":
#             if len(self.value) not in (4, 5):
#                 raise ValueError(
#                     f"check_point(type=LanePosition) value 應該是 [roadId, laneId, s, offset(, orientation)]，"
#                     f"但目前長度是 {len(self.value)}: {self.value}"
#                 )
#         else:
#             raise ValueError(
#                 f"check_point.type 只能是 'WorldPosition' 或 'LanePosition'，目前是 {self.type!r}"
#             )

#         if len(self.speed_tolerance) != 2:
#             raise ValueError(
#                 f"speed_tolerance 應為 [a, b] 兩個數值 (speed-a ~ speed+b)，目前是 {self.speed_tolerance}"
#             )


@dataclass
class GoalConfig:
    position: Position
    # speed: float

    # def __post_init__(self) -> None:
    #     if self.type == "WorldPosition":
    #         if len(self.value) != 3:
    #             raise ValueError(
    #                 f"goal(type=WorldPosition) value 應該是 [x, y, z]，目前長度是 {len(self.value)}: {self.value}"
    #             )
    #     elif self.type == "LanePosition":
    #         if len(self.value) != 4:
    #             raise ValueError(
    #                 f"goal(type=LanePosition) value 應該是 [roadId, laneId, s, offset]，"
    #                 f"目前長度是 {len(self.value)}: {self.value}"
    #             )
    #     else:
    #         raise ValueError(
    #             f"goal.type 只能是 'WorldPosition' 或 'LanePosition'，目前是 {self.type!r}"
    #         )


@dataclass
class EgoConfig:
    target_speed: float
    spawn: SpawnConfig
    # check_points: List[CheckPointConfig]
    goal: GoalConfig

    # ---- 解析 YAML 的工廠方法 ----
    @classmethod
    def from_dict(cls, ego: Dict[str, Any]) -> "EgoConfig":

        try:
            target_speed = float(ego["target_speed"])
        except KeyError:
            raise ValueError("ego.target_speed 未設定") from None
        except (TypeError, ValueError):
            raise ValueError(
                f"ego.target_speed 必須是數字，現在是 {ego.get('target_speed')!r}"
            )

        try:
            spawn_raw = ego["spawn"]
        except KeyError:
            raise ValueError("ego.spawn 未設定")

        spawn = SpawnConfig(
            position=Position(
                position_type=spawn_raw["type"],
                values=spawn_raw["value"],
            ),
            speed=float(spawn_raw["speed"]),
        )

        # cps_raw = ego.get("check_points", [])
        # if not isinstance(cps_raw, list):
        #     raise ValueError("ego.check_points 必須是 list")

        # check_points = []
        # for i, cp in enumerate(cps_raw):
        #     try:
        #         cp_obj = CheckPointConfig(
        #             type=cp["type"],
        #             value=tuple(cp["value"]),
        #             tolerance=float(cp["tolerance"]),
        #             speed=float(cp["speed"]),
        #             speed_tolerance=tuple(cp["speed_tolerance"]),
        #             time_limit=float(cp["time_limit"]),
        #             time_fatal=bool(cp["time_fatal"]),
        #         )
        #     except KeyError as e:
        #         raise ValueError(f"check_points[{i}] 缺少必要欄位: {e}") from None
        #     check_points.append(cp_obj)

        try:
            goal_raw = ego["goal"]
        except KeyError:
            raise ValueError("ego.goal 未設定")

        goal = GoalConfig(
            position=Position(
                position_type=goal_raw["type"],
                values=goal_raw["value"],
            ),
        )

        return cls(
            target_speed=target_speed,
            spawn=spawn,
            # check_points=check_points,
            goal=goal,
        )

    @classmethod
    def from_yaml(cls, path: str) -> "EgoConfig":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return cls.from_dict(data)

    # ---- 這裡做你要的 xosc 路徑/物件轉換 ----
    def to_xosc_route(self):
        """
        這裡依你的 sps.scenarios.xosc 實際 API 去改。
        假設它有 xosc.Route 之類的物件可以拿來建路徑。
        """
        # 範例：把 lane-based 的點轉成 route waypoints（超簡化示意）
        lane_points = []
        for cp in self.check_points + [self.goal]:
            if cp.type != "LanePosition":
                # 如果你希望 check_point 一律要寫「道」(LanePosition)，這裡就丟錯
                raise ValueError(
                    f"生成 xosc 路徑時，check_point/goal 必須是 LanePosition，但遇到 {cp.type}"
                )
            road_id, lane_id, s, offset = cp.value[:4]
            lane_points.append(
                {
                    "road_id": road_id,
                    "lane_id": lane_id,
                    "s": s,
                    "offset": offset,
                }
            )

        # TODO: 這邊依你們的 sps.scenarios.xosc 實作實際 route 物件
        # 例：
        # from sps.scenarios import xosc
        # route = xosc.Route(...)
        # return route

        return lane_points  # 先回傳整理好的資料結構給你看


@dataclass
class ScenarioPack:
    name: str
    maps: dict[str, Path]  # map format -> path
    scenarios: dict[str, Path]
    ego: EgoConfig

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ScenarioPack":
        try:
            name = data["name"]
            maps = {
                fmt: Path("scenarios").resolve() / Path(p)
                for fmt, p in data["maps"].items()
            }
            scenarios = {
                fmt: Path("scenarios").resolve() / Path(p)
                for fmt, p in data["scenarios"].items()
            }
            ego = EgoConfig.from_dict(data["ego"])
        except KeyError as e:
            raise ValueError(f"ScenarioPack 缺少必要欄位: {e}") from None

        return cls(
            name=name,
            maps=maps,
            scenarios=scenarios,
            ego=ego,
        )

    @classmethod
    def from_yaml(cls, path: str) -> "ScenarioPack":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return cls.from_dict(data)
