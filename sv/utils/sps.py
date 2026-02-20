from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict
import yaml

from sbsvf_api import path_pb2, scenario_pb2


from sv.utils.util import get_cfg
from sv.utils.position import PositionFactory, Position


@dataclass
class SpawnConfig:
    position: Position
    speed: float

    def to_protobuf(self) -> scenario_pb2.SpawnConfig:
        return scenario_pb2.SpawnConfig(
            position=self.position.to_protobuf(),
            speed=self.speed,
        )


@dataclass
class GoalConfig:
    position: Position
    # speed: float

    def to_protobuf(self) -> scenario_pb2.GoalConfig:
        return scenario_pb2.GoalConfig(
            position=self.position.to_protobuf(),
            # speed=self.speed,
        )


@dataclass
class EgoConfig:
    target_speed: float
    # check_points: List[CheckPointConfig]
    goal: GoalConfig
    spawn: SpawnConfig = field(default=None)  # 如果 spawn 是選填的話

    # ---- 解析 YAML 的工廠方法 ----
    @classmethod
    def from_dict(
        cls, ego: Dict[str, Any], xodr_path: Path, rmlib_path: Path
    ) -> "EgoConfig":
        position_factory = PositionFactory(
            lib_path=rmlib_path.resolve(),
            xodr_path=xodr_path.resolve(),
        )

        try:
            target_speed = float(ego["target_speed"])
        except KeyError:
            raise ValueError("ego.target_speed 未設定") from None
        except (TypeError, ValueError):
            raise ValueError(
                f"ego.target_speed 必須是數字，現在是 {ego.get('target_speed')!r}"
            )

        # try:
        #     spawn_raw = ego["spawn"]
        # except KeyError:
        #     raise ValueError("ego.spawn 未設定")

        # if spawn_raw["type"] == "LanePosition":
        #     spawn_pos = position_factory.from_lane(
        #         road_id=int(spawn_raw["value"][0]),
        #         lane_id=int(spawn_raw["value"][1]),
        #         s=float(spawn_raw["value"][2]),
        #         offset=(
        #             float(spawn_raw["value"][3]) if len(spawn_raw["value"]) > 3 else 0.0
        #         ),
        #     )
        # elif spawn_raw["type"] == "WorldPosition":
        #     spawn_pos = position_factory.from_world(
        #         x=float(spawn_raw["value"][0]),
        #         y=float(spawn_raw["value"][1]),
        #         z=float(spawn_raw["value"][2]),
        #         h=float(spawn_raw["value"][3]) if len(spawn_raw["value"]) > 3 else 0.0,
        #         p=float(spawn_raw["value"][4]) if len(spawn_raw["value"]) > 4 else 0.0,
        #         r=float(spawn_raw["value"][5]) if len(spawn_raw["value"]) > 5 else 0.0,
        #     )

        # spawn = SpawnConfig(
        #     position=spawn_pos,
        #     speed=float(spawn_raw["speed"]),
        # )

        try:
            goal_raw = ego["goal"]
        except KeyError:
            raise ValueError("ego.goal 未設定")

        if goal_raw["type"] == "LanePosition":
            goal_pos = position_factory.from_lane(
                road_id=int(goal_raw["value"][0]),
                lane_id=int(goal_raw["value"][1]),
                s=float(goal_raw["value"][2]),
                offset=(
                    float(goal_raw["value"][3]) if len(goal_raw["value"]) > 3 else 0.0
                ),
            )
        elif goal_raw["type"] == "WorldPosition":
            goal_pos = position_factory.from_world(
                x=float(goal_raw["value"][0]),
                y=float(goal_raw["value"][1]),
                z=float(goal_raw["value"][2]),
                h=float(goal_raw["value"][3]) if len(goal_raw["value"]) > 3 else 0.0,
                p=float(goal_raw["value"][4]) if len(goal_raw["value"]) > 4 else 0.0,
                r=float(goal_raw["value"][5]) if len(goal_raw["value"]) > 5 else 0.0,
            )

        goal = GoalConfig(position=goal_pos)
        position_factory.close()
        return cls(
            target_speed=target_speed,
            # spawn=spawn,
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

        return lane_points  # 先回傳整理好的資料結構給你看

    def to_protobuf(self) -> scenario_pb2.EgoConfig:
        return scenario_pb2.EgoConfig(
            target_speed=self.target_speed,
            # spawn_config=self.spawn.to_protobuf(),
            # check_points=[cp.to_protobuf() for cp in self.check_points],
            goal_config=self.goal.to_protobuf(),
        )


@dataclass
class ScenarioPack:
    name: str
    maps: dict[str, Path]  # map format -> path
    scenarios: dict[str, Path]
    param_range_file: Path | None
    ego: EgoConfig
    timeout_ns: int = field(default=int(3e11))  # default 300 seconds

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
            ego = EgoConfig.from_dict(
                data["ego"],
                xodr_path=Path("scenarios").resolve() / Path(data["maps"]["xodr"]),
                rmlib_path=Path(
                    data.get("rmlib_path", "/opt/esmini/bin/libesminiRMLib.so")
                ).resolve(),
            )

            param_range_file = None
            if "param_range_file" in data:
                param_range_file = Path("scenarios").resolve() / Path(
                    data["param_range_file"]
                )

        except KeyError as e:
            raise ValueError(f"ScenarioPack 缺少必要欄位: {e}") from None

        return cls(
            name=name,
            maps=maps,
            scenarios=scenarios,
            ego=ego,
            param_range_file=param_range_file,
        )

    @classmethod
    def from_dict(
        cls, scenario_spec: Dict[str, Any], map_spec: Dict[str, Any]
    ) -> "ScenarioPack":
        name = scenario_spec["title"]
        scenarios = {"xosc": Path(scenario_spec["scenario_path"]).resolve()}
        # maps = map_spec.copy()
        maps = {}
        for fmt, p in map_spec.items():
            if fmt == "name":
                continue  # 跳過 name 欄位
            maps[fmt] = Path(p).resolve()
        ego = EgoConfig.from_dict(
            scenario_spec["ego"],
            xodr_path=Path(maps["xodr_path"]).resolve(),
            rmlib_path=Path(
                scenario_spec.get("rmlib_path", "/opt/esmini/bin/libesminiRMLib.so")
            ).resolve(),
        )
        param_range_file = scenario_spec.get("param_path", None)
        if param_range_file is not None:
            param_range_file = Path(param_range_file).resolve()

        return cls(
            name=name,
            maps=maps,
            scenarios=scenarios,
            ego=ego,
            param_range_file=param_range_file,
        )

    @classmethod
    def from_yaml(cls, path: str) -> "ScenarioPack":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return cls.from_dict(data)

    def to_protobuf(self):
        return scenario_pb2.ScenarioPack(
            name=self.name,
            maps={fmt: path_pb2.Path(path=str(p)) for fmt, p in self.maps.items()},
            scenarios={
                fmt: path_pb2.Path(path=str(p)) for fmt, p in self.scenarios.items()
            },
            param_range_file=(
                path_pb2.Path(path=str(self.param_range_file))
                if self.param_range_file
                else None
            ),
            ego=self.ego.to_protobuf(),
            timeout_ns=self.timeout_ns,
        )
