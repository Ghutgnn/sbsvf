from enum import Enum
from dataclasses import dataclass
from typing import Dict, Any, List, Protocol
from carla_api import control_pb2
from google.protobuf.struct_pb2 import Struct


class CtrlMode(str, Enum):
    None_ = "NONE"
    TRAJ = "TRAJ"
    THROTTLE_STEER = "THROTTLE_STEER"
    WAYPOINTS = "WAYPOINTS"
    POSITION = "POSITION"
    ACKERMANN = "ACKERMANN"
    THROTTLE_STEER_BREAK = "THROTTLE_STEER_BREAK"


@dataclass
class Ctrl:
    mode: CtrlMode = CtrlMode.None_
    payload: Dict[str, Any] = None

    def to_pb(self):
        payload_struct = Struct()
        if self.payload is not None:
            payload_struct.update(self.payload)

        return control_pb2.CtrlCmd(
            mode=control_pb2.CtrlMode.Value(self.mode.value),
            payload=payload_struct,
        )
