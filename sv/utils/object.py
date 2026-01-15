from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


class RoadObjectType(Enum):
    UNKNOWN = auto()
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
    BOUNDING_BOX = 1
    CYLINDER = 2
    POLYGON = 3


DEFAULT_SHAPES: dict[RoadObjectType, tuple[float, float, float]] = {
    RoadObjectType.CAR: (4.5, 1.8, 1.5),
    RoadObjectType.TRUCK: (8.0, 2.5, 3.5),
    RoadObjectType.BICYCLE: (2.0, 0.6, 1.2),
}


@dataclass
class ObjectKinematic:
    time: float = 0.0

    # position
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    # longitudinal motion (vehicle frame)
    speed: float = 0.0  # forward speed [m/s]
    accel: float = 0.0  # forward acceleration [m/s^2]

    # angular motion
    yaw_rate: float = 0.0
    yaw_acc: float = 0.0

    @classmethod
    def from_dict(cls, data: dict) -> "ObjectKinematic":
        return cls(
            time=float(data.get("time", 0.0)),
            x=float(data.get("x", 0.0)),
            y=float(data.get("y", 0.0)),
            z=float(data.get("z", 0.0)),
            yaw=float(data.get("yaw", 0.0)),
            speed=float(data.get("speed", 0.0)),
            accel=float(data.get("accel", 0.0)),
            yaw_rate=float(data.get("yaw_rate", 0.0)),
            yaw_acc=float(data.get("yaw_acc", 0.0)),
        )


from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class Shape:
    type: ShapeType = ShapeType.BOUNDING_BOX
    dimensions: tuple[float, float, float] = (0.0, 0.0, 0.0)
    footprint: Optional[tuple[tuple[float, float], ...]] = None


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

    def update(self, kinematic: ObjectKinematic) -> None:
        self.kinematic = kinematic


def default_shape_for_vehicle(vehicle_type: RoadObjectType) -> Shape:
    if vehicle_type in DEFAULT_SHAPES:
        dims = DEFAULT_SHAPES[vehicle_type]
        return Shape(type=ShapeType.BOUNDING_BOX, dimensions=dims)
    else:
        return Shape(type=ShapeType.BOUNDING_BOX, dimensions=(0.0, 0.0, 0.0))
