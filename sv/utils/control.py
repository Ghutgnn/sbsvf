from enum import Enum
from dataclasses import dataclass
from typing import Dict, Any, List, Protocol


class CtrlMode(str, Enum):
    None_ = "NONE"
    TRAJ = "TRAJ"
    THROTTLE_STEER = "THROTTLE_STEER"
    VEL_STEER = "VEL_STEER"
    WAYPOINTS = "WAYPOINTS"
    POSITION = "POSITION"


@dataclass
class Ctrl:
    mode: CtrlMode = CtrlMode.None_
    payload: Dict[str, Any] = None
