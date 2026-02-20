from __future__ import annotations

import ctypes as ct
from dataclasses import dataclass
from pathlib import Path
import logging

from sbsvf_api import position_pb2


logger = logging.getLogger("esmini.rm")


# ---------- ctypes struct ----------
class RM_PositionData(ct.Structure):
    _fields_ = [
        ("x", ct.c_float),
        ("y", ct.c_float),
        ("z", ct.c_float),
        ("h", ct.c_float),
        ("p", ct.c_float),
        ("r", ct.c_float),
        ("hRelative", ct.c_float),
        ("roadId", ct.c_int),
        ("junctionId", ct.c_int),
        ("laneId", ct.c_int),
        ("laneOffset", ct.c_float),
        ("s", ct.c_float),
    ]


# ---------- pure data ----------
@dataclass(frozen=True, slots=True)
class LanePosition:
    road_id: int
    lane_id: int
    s: float
    offset: float
    junction_id: int = -1  # -1 if not in a junction

    def to_protobuf(self) -> position_pb2.LanePosition:
        return position_pb2.LanePosition(
            road_id=self.road_id,
            junction_id=self.junction_id,
            lane_id=self.lane_id,
            offset=self.offset,
            s=self.s,
        )


@dataclass(frozen=True, slots=True)
class WorldPosition:
    x: float
    y: float
    z: float
    h: float
    p: float
    r: float
    h_relative: float

    def to_protobuf(self) -> position_pb2.WorldPosition:
        return position_pb2.WorldPosition(
            x=self.x,
            y=self.y,
            z=self.z,
            h=self.h,
            p=self.p,
            r=self.r,
            h_relative=self.h_relative,
        )


@dataclass(frozen=True, slots=True)
class Position:
    """
    Immutable snapshot: contains BOTH representations.
    Factory fills everything at creation time.
    """

    lane: LanePosition
    world: WorldPosition

    # convenience properties (optional)
    @property
    def road_id(self) -> int:
        return self.lane.road_id

    @property
    def lane_id(self) -> int:
        return self.lane.lane_id

    @property
    def s(self) -> float:
        return self.lane.s

    @property
    def offset(self) -> float:
        return self.lane.offset

    @property
    def x(self) -> float:
        return self.world.x

    @property
    def y(self) -> float:
        return self.world.y

    @property
    def z(self) -> float:
        return self.world.z

    @property
    def h(self) -> float:
        return self.world.h

    @property
    def p(self) -> float:
        return self.world.p

    @property
    def r(self) -> float:
        return self.world.r

    # dict representation
    def to_dict(self) -> dict:
        return {
            "road_id": self.road_id,
            "lane_id": self.lane_id,
            "s": self.s,
            "offset": self.offset,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "h": self.h,
            "p": self.p,
            "r": self.r,
        }

    def to_protobuf(self) -> position_pb2.Position:
        return position_pb2.Position(
            lane=self.lane.to_protobuf(),
            world=self.world.to_protobuf(),
        )


# ---------- factory ----------
class PositionFactory:
    """
    RM is initialized once. Each Position creation uses:
      create handle -> set lane/world -> get data -> delete handle
    Returned Position is pure data (no handle, no factory ref).
    """

    def __init__(self, lib_path: Path, xodr_path: Path):
        self._rm = ct.CDLL(str(lib_path))
        self._setup_functions()

        # Disable RM log file output
        self._rm.RM_SetLogFilePath("".encode())
        self._rm.RM_SetOptionPersistent("disable_stdout".encode())

        ret = int(self._rm.RM_Init(str(xodr_path).encode()))
        if ret != 0:
            raise RuntimeError(f"RM_Init failed ret={ret} xodr={xodr_path}")
        logger.info("RM_Init OK: %s", xodr_path)

        self._closed = False

    def _setup_functions(self) -> None:
        rm = self._rm

        rm.RM_SetLogFilePath.argtypes = [ct.c_char_p]
        rm.RM_SetLogFilePath.restype = None

        rm.RM_SetOptionPersistent.argtypes = [ct.c_char_p]
        rm.RM_SetOptionPersistent.restype = int

        rm.RM_Init.argtypes = [ct.c_char_p]
        rm.RM_Init.restype = ct.c_int

        rm.RM_Close.argtypes = []
        rm.RM_Close.restype = ct.c_int

        rm.RM_CreatePosition.argtypes = []
        rm.RM_CreatePosition.restype = ct.c_int

        rm.RM_DeletePosition.argtypes = [ct.c_int]
        rm.RM_DeletePosition.restype = ct.c_int

        rm.RM_SetLanePosition.argtypes = [
            ct.c_int,
            ct.c_int,
            ct.c_int,
            ct.c_float,
            ct.c_float,
            ct.c_bool,
        ]
        rm.RM_SetLanePosition.restype = ct.c_int

        rm.RM_SetWorldPosition.argtypes = [
            ct.c_int,
            ct.c_float,
            ct.c_float,
            ct.c_float,
            ct.c_float,
            ct.c_float,
            ct.c_float,
        ]
        rm.RM_SetWorldPosition.restype = ct.c_int

        rm.RM_GetPositionData.argtypes = [ct.c_int, ct.POINTER(RM_PositionData)]
        rm.RM_GetPositionData.restype = ct.c_int

    def close(self) -> None:
        if self._closed:
            return
        ret = int(self._rm.RM_Close())
        self._closed = True
        logger.info("RM_Close ret=%d", ret)

    def __enter__(self) -> "PositionFactory":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError(
                "PositionFactory is closed. Create a new factory to generate more positions."
            )

    def _make_snapshot_from_handle(self, handle: int) -> Position:
        out = RM_PositionData()
        ret = int(self._rm.RM_GetPositionData(handle, ct.byref(out)))
        if ret != 0:
            raise RuntimeError(f"RM_GetPositionData failed ret={ret}")

        lane = LanePosition(
            road_id=int(out.roadId),
            lane_id=int(out.laneId),
            s=float(out.s),
            offset=float(out.laneOffset),
            junction_id=int(out.junctionId),
        )
        world = WorldPosition(
            x=float(out.x),
            y=float(out.y),
            z=float(out.z),
            h=float(out.h),
            p=float(out.p),
            r=float(out.r),
            h_relative=float(out.hRelative),
        )
        return Position(lane=lane, world=world)

    def from_lane(
        self,
        road_id: int,
        lane_id: int,
        s: float,
        offset: float = 0.0,
        align: bool = True,
    ) -> Position:
        self._ensure_open()
        handle = int(self._rm.RM_CreatePosition())
        if handle < 0:
            raise RuntimeError("RM_CreatePosition failed")

        logger.debug("Create handle=%d (from_lane)", handle)

        try:
            ret = int(
                self._rm.RM_SetLanePosition(
                    handle, road_id, lane_id, float(offset), float(s), bool(align)
                )
            )
            logger.debug("RM_SetLanePosition(handle=%d) ret=%d", handle, ret)
            if ret != 0:
                raise RuntimeError(f"RM_SetLanePosition failed ret={ret}")

            pos = self._make_snapshot_from_handle(handle)
            logger.info(
                "Position snapshot from lane: road=%d lane=%d s=%.3f offset=%.3f -> x=%.3f y=%.3f z=%.3f h=%.3f p=%.3f r=%.3f",
                road_id,
                lane_id,
                s,
                offset,
                pos.x,
                pos.y,
                pos.z,
                pos.h,
                pos.p,
                pos.r,
            )
            return pos
        finally:
            self._rm.RM_DeletePosition(handle)
            logger.debug("RM_DeletePosition(%d)", handle)

    def from_world(
        self,
        x: float,
        y: float,
        z: float,
        h: float = 0.0,
        p: float = 0.0,
        r: float = 0.0,
    ) -> Position:
        self._ensure_open()
        handle = int(self._rm.RM_CreatePosition())
        if handle < 0:
            raise RuntimeError("RM_CreatePosition failed")

        logger.debug("Create handle=%d (from_world)", handle)

        try:
            ret = int(
                self._rm.RM_SetWorldPosition(
                    handle, float(x), float(y), float(z), float(h), float(p), float(r)
                )
            )
            logger.debug("RM_SetWorldPosition(handle=%d) ret=%d", handle, ret)
            if ret != 0:
                raise RuntimeError(f"RM_SetWorldPosition failed ret={ret}")

            pos = self._make_snapshot_from_handle(handle)
            logger.info(
                "Position snapshot from world: x=%.3f y=%.3f z=%.3f h=%.3f p=%.3f r=%.3f -> road=%d lane=%d s=%.3f offset=%.3f",
                x,
                y,
                z,
                pos.h,
                pos.p,
                pos.r,
                pos.road_id,
                pos.lane_id,
                pos.s,
                pos.offset,
            )
            return pos
        finally:
            self._rm.RM_DeletePosition(handle)
            logger.debug("RM_DeletePosition(%d)", handle)
