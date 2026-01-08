from dataclasses import dataclass
import math


@dataclass
class VehicleKinematic:
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
    def from_dict(cls, data: dict) -> "VehicleKinematic":
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
