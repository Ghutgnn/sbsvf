import logging
import subprocess
import json
import tempfile
import pathlib
import time
from enum import Enum, auto
from typing import Any, Optional, Union
from sv.registry import register_sim
from pathlib import Path
from math import pi
import ctypes as ct
import yaml

from sv.utils.util import get_cfg
from sv.utils.sps import ScenarioPack
from sv.utils.control import Ctrl, CtrlMode

logger = logging.getLogger(__name__)


class SEScenarioObjectState(ct.Structure):
    _fields_ = [
        ("id", ct.c_int),
        ("model_id", ct.c_int),
        ("control", ct.c_int),
        ("timestamp", ct.c_float),
        ("x", ct.c_float),
        ("y", ct.c_float),
        ("z", ct.c_float),
        ("h", ct.c_float),
        ("p", ct.c_float),
        ("r", ct.c_float),
        ("roadId", ct.c_int),
        ("junctionId", ct.c_int),
        ("t", ct.c_float),
        ("laneId", ct.c_int),
        ("laneOffset", ct.c_float),
        ("s", ct.c_float),
        ("speed", ct.c_float),
        ("centerOffsetX", ct.c_float),
        ("centerOffsetY", ct.c_float),
        ("centerOffsetZ", ct.c_float),
        ("width", ct.c_float),
        ("length", ct.c_float),
        ("height", ct.c_float),
        ("objectType", ct.c_int),
        ("objectCategory", ct.c_int),
        ("wheelAngle", ct.c_float),
        ("wheelRot", ct.c_float),
    ]


class SESimpleVehicleState(ct.Structure):
    _fields_ = [
        ("x", ct.c_float),
        ("y", ct.c_float),
        ("z", ct.c_float),
        ("h", ct.c_float),
        ("p", ct.c_float),
        ("speed", ct.c_float),
        ("wheel_rotation", ct.c_float),
        ("wheel_angle", ct.c_float),
    ]


class SimulatorState(Enum):
    INIT = auto()
    AV_CONNECTING = auto()
    WAITING_FOR_PLANNING = auto()
    ENGAGING = auto()
    RUNNING = auto()
    STOPPED = auto()


class Vehicle:
    """Internal helper class, only used inside Simulator."""

    def __init__(self, se, x, y, h, length, speed):
        self._se = se

        self.sv_handle = self._se.SE_SimpleVehicleCreate(x, y, h, length, speed)
        self.vh_state = SESimpleVehicleState()
        self._se.SE_SimpleVehicleGetState(self.sv_handle, ct.byref(self.vh_state))

    def apply_control(self, ctrl: Ctrl, dt):
        if ctrl.mode == CtrlMode.None_:
            return
        elif ctrl.mode == CtrlMode.THROTTLE_STEER:
            logger.debug(
                f"Applying control: pedal={ctrl.payload.get('pedal', 0)}, wheel={ctrl.payload.get('wheel', 0)}, dt={dt}"
            )
            pedal = int(ctrl.payload.get("pedal", 0))
            wheel = int(ctrl.payload.get("wheel", 0))
            self._se.SE_SimpleVehicleControlBinary(self.sv_handle, dt, pedal, wheel)
            # Update vehicle state
            self._se.SE_SimpleVehicleGetState(self.sv_handle, ct.byref(self.vh_state))

        # elif ctrl.mode == CtrlMode.VEL_STEER:
        #     speed = ctrl.payload.get("speed", self.vh_state.speed)
        #     h = ctrl.payload.get("h", self.vh_state.h)
        #     logger.info(f"Applying control: speed={speed}, h={h}, dt={dt}")
        #     # self._se.SE_SimpleVehicleControlTarget(self.sv_handle, dt, 100, 0)
        #     # self._se.SE_SimpleVehicleControlBinary(self.sv_handle, dt, -1, -1)
        #     self._se.SE_SimpleVehicleControlAnalog(self.sv_handle, dt, 0, h)

        #     self._se.SE_SimpleVehicleSetSpeed(self.sv_handle, speed)
        elif ctrl.mode == CtrlMode.POSITION:
            x = ctrl.payload.get("x", self.vh_state.x)
            y = ctrl.payload.get("y", self.vh_state.y)
            h = ctrl.payload.get("h", self.vh_state.h)
            logger.debug(f"Applying control: x={x}, y={y}, h={h}, dt={dt}")
            # Directly set position
            self.vh_state.x = x
            self.vh_state.y = y
            self.vh_state.h = h

        else:
            logger.warning(f"Unsupported control mode: {ctrl.mode}")


@register_sim("esmini")
class EsminiAdapter:
    def __init__(self, cfg_path: Union[str, Path], sps: ScenarioPack):
        self.sim_state = SimulatorState.INIT
        self.cfg = get_cfg(cfg_path)
        self.esmini_home = self.cfg.get("esmini_home", "/opt/esmini/")
        self.extra_paths = self.cfg.get("extra_paths", [])
        self.obj_states = SEScenarioObjectState()
        self.log_file_path = self.cfg.get("log_file_path", "./esmini_log.txt")
        self.se = ct.CDLL(self.esmini_home + "bin/libesminiLib.so")  # Linux
        self._c_param_cb = None
        self._params_obj = None
        self._params_ptr = None
        self.vehicle = None
        self._setup_esmini_opts()
        self._setup_function_signatures()

    def _setup_esmini_opts(self):
        self.se.SE_SetLogFilePath(self.log_file_path.encode())
        for extra_path in self.extra_paths:
            self.se.SE_AddPath(extra_path.encode())
        self.se.SE_SetWindowPosAndSize(60, 60, 1920, 1080)
        self.se.SE_LogToConsole(0)

    def _setup_function_signatures(self):
        se = self.se

        # SE_DLL_API void *SE_SimpleVehicleCreate(float x, float y, float h, float length, float speed);
        se.SE_SimpleVehicleCreate.argtypes = [
            ct.c_float,
            ct.c_float,
            ct.c_float,
            ct.c_float,
            ct.c_float,
        ]
        se.SE_SimpleVehicleCreate.restype = ct.c_void_p

        # SE_DLL_API void SE_SimpleVehicleGetState(void *handleSimpleVehicle, SE_SimpleVehicleState *state);
        se.SE_SimpleVehicleGetState.argtypes = [ct.c_void_p, ct.c_void_p]
        se.SE_SimpleVehicleGetState.restype = None

        # SE_DLL_API void SE_SimpleVehicleControlBinary(void *handleSimpleVehicle, double dt, int throttle, int steering);
        se.SE_SimpleVehicleControlBinary.argtypes = [
            ct.c_void_p,
            ct.c_double,
            ct.c_int,
            ct.c_int,
        ]
        se.SE_SimpleVehicleControlBinary.restype = None

        # SE_DLL_API void SE_SimpleVehicleControlAnalog(void  *handleSimpleVehicle, double dt, double throttle, double steering);
        se.SE_SimpleVehicleControlAnalog.argtypes = [
            ct.c_void_p,
            ct.c_double,
            ct.c_double,
            ct.c_double,
        ]
        se.SE_SimpleVehicleControlAnalog.restype = None

        se.SE_SimpleVehicleControlTarget.argtypes = [
            ct.c_void_p,
            ct.c_double,
            ct.c_float,
            ct.c_float,
        ]
        se.SE_SimpleVehicleSetSpeed.argtypes = [ct.c_void_p, ct.c_float]
        se.SE_ReportObjectWheelStatus.argtypes = [ct.c_int, ct.c_float, ct.c_float]
        se.SE_ReportObjectPosXYH.argtypes = [
            ct.c_int,
            ct.c_float,
            ct.c_float,
            ct.c_float,
            ct.c_float,
        ]
        # SE_DLL_API void SE_RegisterParameterDeclarationCallback(void (*fnPtr)(void *), void *user_data);
        self._PARAM_CB_TYPE = ct.CFUNCTYPE(None, ct.c_void_p)
        self.se.SE_RegisterParameterDeclarationCallback.argtypes = [
            self._PARAM_CB_TYPE,
            ct.c_void_p,
        ]
        self.se.SE_RegisterParameterDeclarationCallback.restype = None

        # SE_DLL_API const char *SE_GetVariableName(int index, int *type);
        self.se.SE_GetVariableName.argtypes = [ct.c_int, ct.c_char_p]
        self.se.SE_GetVariableName.restype = ct.c_char_p

        # SE_DLL_API int SE_SetParameterBool(const char *parameterName, bool value);
        self.se.SE_SetParameterBool.argtypes = [ct.c_char_p, ct.c_bool]
        self.se.SE_SetParameterBool.restype = None

        # SE_DLL_API int SE_GetVariableInt(const char *variableName, int *value);
        self.se.SE_SetParameterInt.argtypes = [ct.c_char_p, ct.c_int]
        self.se.SE_SetParameterInt.restype = None

        # SE_DLL_API int SE_GetVariableDouble(const char *variableName, double *value);
        self.se.SE_SetParameterDouble.argtypes = [ct.c_char_p, ct.c_double]
        self.se.SE_SetParameterDouble.restype = None

        # SE_DLL_API int SE_GetVariableString(const char *variableName, const char **value);
        self.se.SE_SetParameterString.argtypes = [ct.c_char_p, ct.c_char_p]
        self.se.SE_SetParameterString.restype = None

        # SE_DLL_API const char *SE_GetParameterName(int index, int *type);
        se.SE_GetParameterName.argtypes = [ct.c_int, ct.POINTER(ct.c_int)]
        se.SE_GetParameterName.restype = ct.c_char_p

        se.SE_GetSimTimeStep.restype = ct.c_float
        se.SE_StepDT.argtypes = [ct.c_float]

        se.SE_GetQuitFlag.restype = ct.c_int

        # 其他 API 可視需要補 argtypes / restype

    def init(self):
        self.sim_state = SimulatorState.AV_CONNECTING

    def start(self, cfg: dict):
        pass

    def step(self, ctrl: Ctrl, dt: float):
        # if self.sim_state == SimulatorState.AV_CONNECTING:
        #     if ctrl.payload.get("pedal", 0) != 0 or ctrl.payload.get("wheel", 0) != 0:
        #         self.sim_state = SimulatorState.RUNNING
        #         logger.info("AV engaged.")
        #     return None

        se = self.se

        # Update vehicle control
        self.vehicle.apply_control(ctrl, dt)

        obj_id = se.SE_GetId(0)
        se.SE_ReportObjectPosXYH(
            obj_id,
            0.0,
            self.vehicle.vh_state.x,
            ct.c_float(self.vehicle.vh_state.y),
            self.vehicle.vh_state.h,
        )
        se.SE_ReportObjectWheelStatus(
            obj_id,
            self.vehicle.vh_state.wheel_rotation,
            self.vehicle.vh_state.wheel_angle,
        )
        # lane_type = se.SE_GetObjectInLaneType(obj_id)
        # obs = {
        #     "x": float(self.vehicle.vh_state.x),
        #     "y": float(self.vehicle.vh_state.y),
        #     "h": float(self.vehicle.vh_state.h),
        #     "speed": float(self.vehicle.vh_state.speed),
        #     "lane_type": int(lane_type),
        #     "in_driving_lane": (lane_type & 1966594) != 0,
        #     "on_road": (lane_type & 1966726) != 0,
        #     "on_defined_area": lane_type != 1,
        # }

        # other agents' states
        # obj_id = se.SE_GetId(1)
        # obj_state = self.obj_states
        obj_state = self.obj_states
        se.SE_GetObjectState(se.SE_GetId(1), ct.byref(obj_state))
        obs = {
            "x": float(obj_state.x),
            "y": float(obj_state.y),
            "h": float(obj_state.h),
            "speed": float(obj_state.speed),
        }
        if dt <= 0:
            se.SE_Step()
        else:
            se.SE_StepDT(ct.c_float(dt))
        return obs

    def stop(self):
        self.se.SE_Close()
        if self.vehicle is not None:
            self.se.SE_SimpleVehicleDelete(self.vehicle.sv_handle)

    def parameter_declaration_callback(self, params: dict[str, Any]) -> int:
        """
        這個是你真的想寫的邏輯：用 dict 設定 parameter。
        C 那邊看不到這個，只會呼叫底下包好的 _c_param_cb。
        """
        n = self.se.SE_GetNumberOfParameters()
        param_type = {}
        for i in range(n):
            ptype = ct.c_int()
            param_name = self.se.SE_GetParameterName(i, ct.byref(ptype)).decode("utf-8")
            param_type[param_name] = ptype.value
            # print(f"esmini parameter {i}: {param_name} (type {ptype.value})")

        for name, value in params.items():
            if name not in param_type:
                logger.warning(
                    f"Parameter {name} not found in esmini parameters. Skip."
                )
                continue

            ptype = param_type[name]
            if ptype == 1:  # int
                try:
                    v = int(value)
                except (TypeError, ValueError):
                    # print(f"  skip {name} = {value} (not an int)")
                    logger.warning(
                        f"Parameter {name} value {value} is not an int. Skip."
                    )
                    continue

                self.se.SE_SetParameterInt(name.encode("utf-8"), v)
                logger.info(f"  set {name} = {v}")
            elif ptype == 2:  # double
                try:
                    v = float(value)
                except (TypeError, ValueError):
                    logger.warning(
                        f"Parameter {name} value {value} is not a float. Skip."
                    )
                    continue

                self.se.SE_SetParameterDouble(name.encode("utf-8"), v)
                logger.info(f"  set {name} = {v}")
            elif ptype == 3:  # string
                v = str(value)
                self.se.SE_SetParameterString(name.encode("utf-8"), v.encode("utf-8"))
                logger.info(f"  set {name} = {v}")
            elif ptype == 4:  # bool
                try:
                    v = bool(value)
                except (TypeError, ValueError):
                    logger.warning(
                        f"Parameter {name} value {value} is not a bool. Skip."
                    )
                    continue

                self.se.SE_SetParameterBool(name.encode("utf-8"), v)
                logger.info(f"  set {name} = {v}")
            else:
                # print(f"  skip {name} = {value} (unknown parameter type {ptype})")
                logger.warning(f"Parameter {name} has unknown type {ptype}. Skip.")
                continue

        # 這個 return 給自己用就好，C callback 是 void，不會用到
        return 0

    def _get_parameter(self, sps: ScenarioPack) -> dict[str, Any]:
        self

    def reset(self, sps: ScenarioPack, params: Optional[dict] = None):
        self.stop()
        if params is None:
            params = {}

        # 1) 把 params 包成 py_object，並轉成 void* 當 user_data
        self._params_obj = ct.py_object(params)
        self._params_ptr = ct.cast(ct.pointer(self._params_obj), ct.c_void_p)

        # 2) 建立一次 C 用的 callback（void (*)(void*)）
        if self._c_param_cb is None:

            @self._PARAM_CB_TYPE
            def _c_param_cb(user_data):

                py_obj_ptr = ct.cast(user_data, ct.POINTER(ct.py_object))
                params_dict = py_obj_ptr.contents.value

                # 呼叫你自己的高階 callback
                self.parameter_declaration_callback(params_dict)

            self._c_param_cb = _c_param_cb  # hold reference

        # 3) 在 SE_Init 前註冊 callback
        self.se.SE_RegisterParameterDeclarationCallback(
            self._c_param_cb,
            self._params_ptr,
        )
        self._setup_esmini_opts()
        ret = self.se.SE_Init(str(sps.scenarios["xosc"]).encode(), 1, 1, 0, 0)
        if ret != 0:
            raise RuntimeError(f"esmini SE_Init failed with code {ret}")
        obj_state = SEScenarioObjectState()
        self.se.SE_GetObjectState(self.se.SE_GetId(0), ct.byref(obj_state))
        self.vehicle = Vehicle(
            self.se,
            obj_state.x,
            obj_state.y,
            obj_state.h,
            obj_state.length,
            obj_state.speed,
        )

    # define a function returning if the simulator need to stop
    def should_quit(self):
        return self.se.SE_GetQuitFlag()

    # def _get_ctrl_input(ctrl: Ctrl):
    #     # Control type I: binary control
    #     pedal = ctrl.payload.get("pedal", 0) if ctrl is not None else 0
    #     wheel = ctrl.payload.get("wheel", 0) if ctrl is not None else 0

    #     return pedal, wheel
