from enum import Enum, auto
from dataclasses import dataclass
from typing import Dict, Any
from sbsvf_api import control_pb2
from google.protobuf.struct_pb2 import Struct


class CtrlMode(Enum):
    None_ = 0
    TRAJ = auto()
    THROTTLE_STEER = auto()
    WAYPOINTS = auto()
    POSITION = auto()
    ACKERMANN = auto()
    THROTTLE_STEER_BREAK = auto()


@dataclass
class Ctrl:
    mode: CtrlMode = CtrlMode.None_
    payload: Dict[str, Any] = None

    def to_pb(self):
        payload_struct = Struct()
        if self.payload is not None:
            payload_struct.update(self.payload)
        return control_pb2.CtrlCmd(
            mode=self.mode.value,
            payload=payload_struct,
        )

    @classmethod
    def from_pb(cls, pb: control_pb2.CtrlCmd) -> "Ctrl":
        mode = CtrlMode(pb.mode)
        payload = {k: v for k, v in pb.payload.items()}
        return cls(mode=mode, payload=payload)


def main():
    for mode in CtrlMode:
        print(mode.name, mode.value)


if __name__ == "__main__":
    main()
