from enum import Enum, auto


class PositionType(Enum):
    LANE_POSITION = auto()
    WORLD_POSITION = auto()


class TPType(Enum):
    CAR = auto()
    PEDESTRIAN = auto()
    BIKE = auto()
    TRUCK = auto()
    BUS = auto()

    def __str__(self):
        return self.name.lower()


class LanePosition:
    def __init__(self, road_id: int, lane_id: int, s: float, offset: float):
        self.road_id = road_id
        self.lane_id = lane_id
        self.s = s
        self.offset = offset

    def __eq__(self, other):
        if not isinstance(other, LanePosition):
            return False
        return (
            self.road_id == other.road_id
            and self.lane_id == other.lane_id
            and self.s == other.s
            and self.offset == other.offset
        )


class WorldPosition:
    def __init__(self, x: float, y: float, z: float, h: float, p: float, r: float):
        self.x = x
        self.y = y
        self.z = z
        self.h = h
        self.p = p
        self.r = r

    def __eq__(self, other):
        if not isinstance(other, WorldPosition):
            return False
        return (
            self.x == other.x
            and self.y == other.y
            and self.z == other.z
            and self.h == other.h
            and self.p == other.p
            and self.r == other.r
        )


class Position:
    def __init__(self, position_type: str, values: list):
        self.type = position_type
        self.element = None
        if position_type == "LanePosition":
            if len(values) < 3:
                raise ValueError(
                    f"LanePosition requires at least 3 values: road_id, lane_id, s. Got: {values}"
                )
            # args: road_id, lane_id, s, offset
            self.element = LanePosition(
                road_id=int(values[0]),
                lane_id=int(values[1]),
                s=float(values[2]),
                offset=float(values[3]) if len(values) > 3 else 0,
            )
        elif position_type == "WorldPosition":
            if len(values) < 3:
                raise ValueError(
                    f"WorldPosition requires at least 3 values: x, y, z. Got: {values}"
                )
            # args: x, y, z, h, p, r
            self.element = WorldPosition(
                x=float(values[0]),
                y=float(values[1]),
                z=float(values[2]),
                h=float(values[3]) if len(values) > 3 else 0,
                p=float(values[4]) if len(values) > 4 else 0,
                r=float(values[5]) if len(values) > 5 else 0,
            )
        else:
            raise ValueError(
                f"Invalid position type:{position_type}. Use 'LanePosition' or 'WorldPosition'."
            )

    def __repr__(self):
        if self.type == "LanePosition":
            return f"LanePosition(road_id={self.element.road_id}, lane_id={self.element.lane_id}, s={self.element.s}, offset={self.element.offset})"
        elif self.type == "WorldPosition":
            return f"WorldPosition(x={self.element.x}, y={self.element.y}, z={self.element.z}, h={self.element.h}, p={self.element.p}, r={self.element.r})"
        else:
            return f"Position(type={self.type}, element={self.element})"

    def __str__(self):
        if self.type == "LanePosition":
            return f"LanePosition({self.element.road_id}, {self.element.lane_id}, {self.element.s}, {self.element.offset})"
        elif self.type == "WorldPosition":
            return f"WorldPosition({self.element.x}, {self.element.y}, {self.element.z}, {self.element.h}, {self.element.p}, {self.element.r})"
        else:
            return f"Position(type={self.type}, element={self.element})"

    def __eq__(self, other):
        if not isinstance(other, Position):
            return False
        return self.element == other.element

    def __hash__(self):
        if isinstance(self.element, LanePosition):
            return hash(
                (
                    self.element.road_id,
                    self.element.lane_id,
                    self.element.s,
                    self.element.offset,
                )
            )
        elif isinstance(self.element, WorldPosition):
            return hash(
                (
                    self.element.x,
                    self.element.y,
                    self.element.z,
                    self.element.h,
                    self.element.p,
                    self.element.r,
                )
            )

    def get_info(self):
        if self.type == "LanePosition":
            return (
                self.element.road_id,
                self.element.lane_id,
                self.element.s,
                self.element.offset,
            )
        elif self.type == "WorldPosition":
            return (
                self.element.x,
                self.element.y,
                self.element.z,
                self.element.h,
                self.element.p,
                self.element.r,
            )


class RelativePosition(Position):
    def __init__(
        self,
        tp_type: TPType,
        tag: str,
        relate_to: Position,
        position_type: str,
        **kwargs,
    ):
        super().__init__(position_type, **kwargs)
        self.tp_type = tp_type
        self.tag = tag
        self.relate_to = relate_to

    def __repr__(self):
        return f"RelativePosition(tp_type={self.tp_type}, tag={self.tag}, relate_to={self.relate_to.__repr__()}, position={super().__repr__()})"

    def __str__(self):
        return f"{self.tag}: {super().__str__()}"
