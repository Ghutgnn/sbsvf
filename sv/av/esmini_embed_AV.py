import logging
from typing import Any, Optional
import ctypes as ct
from sv.registry import register_av
from sv.utils.control import Ctrl, CtrlMode
from sv.utils.sps import ScenarioPack
from sv.utils.util import get_cfg
from sv.sim.esmini import SESimpleVehicleState, SEScenarioObjectState

logger = logging.getLogger(__name__)


class Vehicle:
    """Internal helper class, only used inside Simulator."""

    def __init__(self, se, x, y, h, length, speed):
        self._se = se
        self.x = 0
        self.y = 0
        self.sv_handle = self._se.SE_SimpleVehicleCreate(x, y, h, length, speed)
        self.vh_state = SESimpleVehicleState()

    def apply_control(self, obs: dict[str, Any], dt: float) -> None:
        x = obs.get("x", 0.0)
        y = obs.get("y", 0.0)
        h = obs.get("h", 0.0)
        logger.debug(f"AV Applying control: x={x}, y={y}")
        self._se.SE_ReportObjectPosXYH(self._se.SE_GetId(1), dt, x, y, h)


@register_av("esmini_embed")
class EsminiEmbedAV:
    def __init__(self, cfg_path: dict, sps: ScenarioPack):
        self.cfg = get_cfg(cfg_path)
        self.esmini_home = self.cfg.get("esmini_home", "/opt/esmini/")
        self.extra_paths = self.cfg.get("extra_paths", [])
        self.obj_states = SEScenarioObjectState()
        self.log_file_path = self.cfg.get("log_file_path", "./esmini_log.txt")
        self.se = ct.CDLL(self.esmini_home + "bin/libesminiLib.so")
        self.agent: Optional[Vehicle] = None
        self._setup_esmini_opts()
        self._setup_function_signatures()

    def _setup_esmini_opts(self):
        self.se.SE_SetLogFilePath(self.log_file_path.encode())
        for extra_path in self.extra_paths:
            self.se.SE_AddPath(extra_path.encode())
        self.se.SE_SetWindowPosAndSize(1920, 60, 1920, 1080)
        self.se.SE_LogToConsole(0)

    def _setup_function_signatures(self):
        se = self.se

        se.SE_SimpleVehicleCreate.argtypes = [
            ct.c_float,
            ct.c_float,
            ct.c_float,
            ct.c_float,
            ct.c_float,
        ]
        se.SE_SimpleVehicleCreate.restype = ct.c_void_p

        se.SE_SimpleVehicleGetState.argtypes = [ct.c_void_p, ct.c_void_p]
        se.SE_SimpleVehicleControlBinary.argtypes = [
            ct.c_void_p,
            ct.c_double,
            ct.c_int,
            ct.c_int,
        ]

        se.SE_ReportObjectWheelStatus.argtypes = [ct.c_int, ct.c_float, ct.c_float]
        se.SE_ReportObjectPosXYH.argtypes = [
            ct.c_int,
            ct.c_float,
            ct.c_float,
            ct.c_float,
            ct.c_float,
        ]

        se.SE_GetSimTimeStep.restype = ct.c_float
        se.SE_StepDT.argtypes = [ct.c_float]

        se.SE_GetQuitFlag.restype = ct.c_int

        # 其他 API 可視需要補 argtypes / restype

    def init(self) -> None:
        pass

    def reset(self, sps: ScenarioPack, params: Optional[dict] = None) -> None:
        self.stop()
        ret = self.se.SE_Init(str(sps.maps["dummy"]).encode(), 0, 8, 0, 0)
        if ret != 0:
            raise RuntimeError(f"esmini SE_Init failed with code {ret}")
        obj_state = SEScenarioObjectState()
        self.se.SE_GetObjectState(self.se.SE_GetId(1), ct.byref(obj_state))
        self.agent = Vehicle(
            self.se,
            obj_state.x,
            obj_state.y,
            obj_state.h,
            obj_state.length,
            obj_state.speed,
        )

    def step(self, obs: dict[str, Any], dt) -> dict[str, Any]:
        self.agent.apply_control(obs, dt)

        obj_state = self.obj_states
        self.se.SE_GetObjectState(self.se.SE_GetId(0), ct.byref(obj_state))

        if dt <= 0:
            self.se.SE_Step()
        else:
            self.se.SE_StepDT(dt)

        return Ctrl(
            mode=CtrlMode.POSITION,
            payload={
                "x": float(obj_state.x),
                "y": float(obj_state.y),
                "h": float(obj_state.h),
            },
        )

    def stop(self) -> None:
        self.se.SE_Close()
        if self.agent is not None:
            self.se.SE_SimpleVehicleDelete(self.agent.sv_handle)

    def should_quit(self) -> bool:
        return self.se.SE_GetQuitFlag() != 0
