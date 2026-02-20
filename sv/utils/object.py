from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

from sbsvf_api import object_pb2


class RoadObjectType(Enum):
    UNKNOWN = 0
    CAR = auto()
    TRUCK = auto()
    BUS = auto()
    SEMITRAILER = auto()
    TRAILER = auto()
    MOTORCYCLE = auto()
    BICYCLE = auto()
    PEDESTRIAN = auto()
    VAN = auto()
    TRAIN = auto()
    TRAM = auto()
    WHEELCHAIR = auto()
    ANIMAL = auto()


class ShapeType(Enum):
    BOUNDING_BOX = 0
    CYLINDER = auto()
    POLYGON = auto()


DEFAULT_SHAPES: dict[RoadObjectType, tuple[float, float, float]] = {
    RoadObjectType.CAR: (4.5, 1.8, 1.5),
    RoadObjectType.TRUCK: (8.0, 2.5, 3.5),
    RoadObjectType.BICYCLE: (2.0, 0.6, 1.2),
}


@dataclass
class ObjectKinematic:
    time_ns: int = 0  # timestamp in nanoseconds

    # position
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    # longitudinal motion (vehicle frame)
    speed: float = 0.0  # forward speed [m/s]
    acceleration: float = 0.0  # forward acceleration [m/s^2]

    # angular motion
    yaw_rate: float = 0.0
    yaw_acceleration: float = 0.0

    @classmethod
    def from_dict(cls, data: dict) -> "ObjectKinematic":
        return cls(
            time_ns=int(data.get("time_ns", 0)),
            x=float(data.get("x", 0.0)),
            y=float(data.get("y", 0.0)),
            z=float(data.get("z", 0.0)),
            yaw=float(data.get("yaw", 0.0)),
            speed=float(data.get("speed", 0.0)),
            acceleration=float(data.get("acceleration", 0.0)),
            yaw_rate=float(data.get("yaw_rate", 0.0)),
            yaw_acceleration=float(data.get("yaw_acceleration", 0.0)),
        )

    @classmethod
    def from_pb(cls, pb: object_pb2.ObjectKinematic) -> "ObjectKinematic":
        return cls(
            time_ns=pb.time_ns,
            x=pb.x,
            y=pb.y,
            z=pb.z,
            yaw=pb.yaw,
            speed=pb.speed,
            acceleration=pb.acceleration,
            yaw_rate=pb.yaw_rate,
            yaw_acceleration=pb.yaw_acceleration,
        )


from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class Shape:
    type: ShapeType = ShapeType.BOUNDING_BOX
    dimensions: tuple[float, float, float] = (0.0, 0.0, 0.0)
    footprint: Optional[tuple[tuple[float, float], ...]] = None

    @classmethod
    def from_pb(cls, pb: object_pb2.Shape) -> "Shape":
        shape_type = (
            ShapeType(pb.type)
            if pb.type in ShapeType._value2member_map_
            else ShapeType.BOUNDING_BOX
        )
        dimensions = (pb.dimensions.x, pb.dimensions.y, pb.dimensions.z)
        footprint = None
        if shape_type == ShapeType.POLYGON:
            footprint = tuple((point.x, point.y) for point in pb.footprint.points)
        return cls(type=shape_type, dimensions=dimensions, footprint=footprint)


@dataclass
class ObjectState:
    _type: RoadObjectType
    kinematic: ObjectKinematic
    _shape: Optional[Shape] = None

    @property
    def type(self) -> RoadObjectType:
        return self._type

    @property
    def shape(self) -> Optional[Shape]:
        return self._shape

    @classmethod
    def create(
        cls,
        *,
        type: RoadObjectType,
        kinematic: ObjectKinematic,
        shape: Shape | None = None,
    ) -> "ObjectState":
        if shape is None:
            shape = default_shape_for_vehicle(type)
        return cls(_type=type, kinematic=kinematic, _shape=shape)

    @classmethod
    def from_pb(cls, pb: object_pb2.ObjectState) -> "ObjectState":
        obj_type = (
            RoadObjectType(pb.type)
            if pb.type in RoadObjectType._value2member_map_
            else RoadObjectType.UNKNOWN
        )
        kinematic = ObjectKinematic.from_pb(pb.kinematic)
        shape = None
        if pb.HasField("shape"):
            shape = Shape.from_pb(pb.shape)
        return cls.create(type=obj_type, kinematic=kinematic, shape=shape)

    def to_pb(self) -> object_pb2.ObjectState:
        obj_type_value = self._type.value if self._type in RoadObjectType else 0
        kinematic_pb = object_pb2.ObjectKinematic(
            time_ns=self.kinematic.time_ns,
            x=self.kinematic.x,
            y=self.kinematic.y,
            z=self.kinematic.z,
            yaw=self.kinematic.yaw,
            speed=self.kinematic.speed,
            acceleration=self.kinematic.acceleration,
            yaw_rate=self.kinematic.yaw_rate,
            yaw_acceleration=self.kinematic.yaw_acceleration,
        )
        shape_pb = None
        if self._shape is not None:
            shape_pb = object_pb2.Shape(
                type=self._shape.type.value if self._shape.type in ShapeType else 0,
                dimensions=object_pb2.Shape.Dimension(
                    x=self._shape.dimensions[0],
                    y=self._shape.dimensions[1],
                    z=self._shape.dimensions[2],
                ),
            )
            # if (
            #     self._shape.type == ShapeType.POLYGON
            #     and self._shape.footprint is not None
            # ):
            #     shape_pb.Shape.Vertex(
            #         object_pb2.Vector2(x=pt[0], y=pt[1]) for pt in self._shape.footprint
            #     )
        return object_pb2.ObjectState(
            type=obj_type_value, kinematic=kinematic_pb, shape=shape_pb
        )

    def update(self, kinematic: ObjectKinematic) -> None:
        self.kinematic = kinematic


def default_shape_for_vehicle(vehicle_type: RoadObjectType) -> Shape:
    if vehicle_type in DEFAULT_SHAPES:
        dims = DEFAULT_SHAPES[vehicle_type]
        return Shape(type=ShapeType.BOUNDING_BOX, dimensions=dims)
    else:
        return Shape(type=ShapeType.BOUNDING_BOX, dimensions=(0.0, 0.0, 0.0))
