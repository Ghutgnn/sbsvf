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
from sv.utils.object import (
    ObjectState,
    ObjectKinematic,
    RoadObjectType,
    Shape,
    ShapeType,
)

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
        ("visibilityMask", ct.c_int),
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


class Vehicle:
    """Internal helper class, only used inside Simulator."""

    def __init__(self, se, x, y, h, length, speed):
        self._se = se
        self.sv_handle = self._se.SE_SimpleVehicleCreate(x, y, h, length, speed)
        self.vh_state = SESimpleVehicleState()
        self._se.SE_SimpleVehicleGetState(self.sv_handle, ct.byref(self.vh_state))

    def apply_control(self, ctrl: Ctrl, dt_s: float):
        if ctrl.mode == CtrlMode.None_:
            return
        elif ctrl.mode == CtrlMode.THROTTLE_STEER:
            pedal = int(ctrl.payload.get("pedal", 0))
            wheel = int(ctrl.payload.get("wheel", 0))
            self._se.SE_SimpleVehicleControlBinary(self.sv_handle, dt_s, pedal, wheel)
            # Update vehicle state
            self._se.SE_SimpleVehicleGetState(self.sv_handle, ct.byref(self.vh_state))

        elif ctrl.mode == CtrlMode.THROTTLE_STEER_BREAK:
            throttle = float(ctrl.payload.get("throttle", 0.0))
            steer = float(ctrl.payload.get("steer", 0.0))
            brake = float(ctrl.payload.get("brake", 0.0))

            final_throttle = (
                throttle - brake
            )  # Simple way to combine throttle and brake

            self._se.SE_SimpleVehicleControlAnalog(
                self.sv_handle, dt_s, final_throttle, steer
            )
            # Update vehicle state
            self._se.SE_SimpleVehicleGetState(self.sv_handle, ct.byref(self.vh_state))

        elif ctrl.mode == CtrlMode.ACKERMANN:
            target_speed = ctrl.payload.get("speed", self.vh_state.speed)
            heading_to_target = ctrl.payload.get("steer", self.vh_state.h)
            self._se.SE_SimpleVehicleControlTarget(
                self.sv_handle, dt_s, target_speed, heading_to_target
            )
            # Update vehicle state
            self._se.SE_SimpleVehicleGetState(self.sv_handle, ct.byref(self.vh_state))

        elif ctrl.mode == CtrlMode.POSITION:
            x = ctrl.payload.get("x", self.vh_state.x)
            y = ctrl.payload.get("y", self.vh_state.y)
            h = ctrl.payload.get("h", self.vh_state.h)
            speed = ctrl.payload.get("speed", self.vh_state.speed)
            # Directly set position
            self.vh_state.x = x
            self.vh_state.y = y
            self.vh_state.h = h
            self.vh_state.speed = speed

        else:
            logger.warning(f"Unsupported control mode: {ctrl.mode}")


TYPE_MAP = {
    0: RoadObjectType.CAR,
    1: RoadObjectType.VAN,
    2: RoadObjectType.TRUCK,
    3: RoadObjectType.BUS,
    4: RoadObjectType.TRAILER,
    5: RoadObjectType.BUS,
    6: RoadObjectType.MOTORCYCLE,
    7: RoadObjectType.BICYCLE,
    8: RoadObjectType.TRAIN,
    9: RoadObjectType.TRAM,
}


class EsminiAdapter:
    def __init__(self, output_dir: Path, cfg_path: Union[str, Path]):
        self._output_dir = output_dir
        self._time_ns = 0
        self.cfg = get_cfg(cfg_path)
        self.esmini_home = self.cfg.get("esmini_home", "/opt/esmini/")
        self.se = ct.CDLL(self.esmini_home + "bin/libesminiLib.so")  # Linux
        self._c_param_cb = None
        self._params_obj = None
        self._params_ptr = None
        self.ego_car = None
        self.objects: list[ObjectState] = []
        self._setup_function_signatures()

    def _setup_esmini_opts(self):
        self.se.SE_SetSeed(1234)

        use_viewer = self.cfg.get("use_viewer", True)
        threads = self.cfg.get("threads", 0)
        record = self.cfg.get("record", False)

        if "log_file_path" in self.cfg:
            log_file_path = self._output_dir / self.cfg["log_file_path"]
            self.se.SE_SetLogFilePath(str(log_file_path).encode())
        else:
            logger.info("No log_file_path specified; using default esmini_log.txt")
            self.se.SE_SetLogFilePath(b"./esmini_log.txt")

        if "path" in self.cfg:
            for extra_path in self.cfg["path"]:
                logger.info(f"Adding esmini path: {extra_path}")
                self.se.SE_AddPath(extra_path.encode())

        if "window" in self.cfg:
            win_cfg = self.cfg["window"]  # ["x", "y", "width", "height"]
            logger.info(f"Setting esmini window position and size: {win_cfg}")
            self.se.SE_SetWindowPosAndSize(
                win_cfg[0], win_cfg[1], win_cfg[2], win_cfg[3]
            )

        if self.cfg.get("disable_stdout", True):
            logger.info("Disable stdout in esmini")
            self.se.SE_SetOptionPersistent(b"disable_stdout")

        if self.cfg.get("dat_file_path", None) is not None:
            dat_file_path = self._output_dir / self.cfg["dat_file_path"]
            logger.info(f"Setting esmini dat file path: {dat_file_path}")
            self.se.SE_SetDatFilePath(str(dat_file_path).encode())

        return use_viewer, threads, record

    def _setup_function_signatures(self):
        se = self.se

        # SE_DLL_API int SE_GetObjectState(int object_id, SE_ScenarioObjectState *state);
        se.SE_GetObjectState.argtypes = [ct.c_int, ct.POINTER(SEScenarioObjectState)]
        se.SE_GetObjectState.restype = ct.c_int

        # SE_DLL_API float SE_GetObjectAcceleration(int object_id);
        se.SE_GetObjectAcceleration.argtypes = [ct.c_int]
        se.SE_GetObjectAcceleration.restype = ct.c_float

        # SE_DLL_API int SE_GetObjectAngularAcceleration(int object_id, float *h_acc, float *p_acc, float *r_acc);
        se.SE_GetObjectAngularAcceleration.argtypes = [
            ct.c_int,
            ct.POINTER(ct.c_float),
            ct.POINTER(ct.c_float),
            ct.POINTER(ct.c_float),
        ]
        se.SE_GetObjectAngularAcceleration.restype = ct.c_int

        # SE_DLL_API int SE_GetObjectAngularVelocity(int object_id, float *h_rate, float *p_rate, float *r_rate);
        se.SE_GetObjectAngularVelocity.argtypes = [
            ct.c_int,
            ct.POINTER(ct.c_float),
            ct.POINTER(ct.c_float),
            ct.POINTER(ct.c_float),
        ]
        se.SE_GetObjectAngularVelocity.restype = ct.c_int

        # SE_DLL_API const char *SE_GetObjectTypeName(int object_id)
        # se.SE_GetObjectTypeName.argtypes = [ct.c_int]
        # se.SE_GetObjectTypeName.restype = ct.c_char_p

        # SE_DLL_API void *SE_SimpleVehicleCreate(float x, float y, float h, float length, float speed);
        se.SE_SimpleVehicleCreate.argtypes = [
            ct.c_float,
            ct.c_float,
            ct.c_float,
            ct.c_float,
            ct.c_float,
        ]
        se.SE_SimpleVehicleCreate.restype = ct.c_void_p

        # SE_DLL_API void SE_SimpleVehicleDelete(void *handleSimpleVehicle);
        se.SE_SimpleVehicleDelete.argtypes = [ct.c_void_p]
        se.SE_SimpleVehicleDelete.restype = None

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

        # SE_DLL_API void SE_SimpleVehicleControlTarget(void *handleSimpleVehicle, double dt, double target_speed, double heading_to_target);
        se.SE_SimpleVehicleControlTarget.argtypes = [
            ct.c_void_p,
            ct.c_double,
            ct.c_double,
            ct.c_double,
        ]
        se.SE_SimpleVehicleControlTarget.restype = None

        se.SE_SimpleVehicleSetSpeed.argtypes = [ct.c_void_p, ct.c_float]

        se.SE_ReportObjectWheelStatus.argtypes = [ct.c_int, ct.c_float, ct.c_float]

        # SE_DLL_API int SE_ReportObjectSpeed(int object_id, float speed);
        se.SE_ReportObjectSpeed.argtypes = [ct.c_int, ct.c_float]
        se.SE_ReportObjectSpeed.restype = ct.c_int

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

        # SE_DLL_API void SE_ClearPaths();
        self.se.SE_ClearPaths.argtypes = []
        self.se.SE_ClearPaths.restype = None

        # SE_DLL_API const char *SE_GetVariableName(int index, int *type);
        self.se.SE_GetVariableName.argtypes = [ct.c_int, ct.c_char_p]
        self.se.SE_GetVariableName.restype = ct.c_char_p

        # SE_DLL_API void SE_SetSeed(unsigned int seed);
        self.se.SE_SetSeed.argtypes = [ct.c_uint]
        self.se.SE_SetSeed.restype = None

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

        # SE_DLL_API int SE_GetNumberOfObjects()
        se.SE_GetNumberOfObjects.argtypes = []
        se.SE_GetNumberOfObjects.restype = ct.c_int

        se.SE_GetSimTimeStep.restype = ct.c_float

        # SE_DLL_API float SE_GetSimulationTime();
        se.SE_GetSimulationTime.restype = ct.c_float

        se.SE_StepDT.argtypes = [ct.c_float]

        se.SE_GetQuitFlag.restype = ct.c_int

        se.SE_SetOptionPersistent.argtypes = [ct.c_char_p]
        se.SE_SetOptionPersistent.restype = ct.c_int

        # SE_DLL_API void SE_SetDatFilePath(const char *datFilePath);
        se.SE_SetDatFilePath.argtypes = [ct.c_char_p]
        se.SE_SetDatFilePath.restype = None

    def init(self, runtime_spec: dict, sps: ScenarioPack) -> None:
        pass

    def step(self, ctrl: Ctrl, time_stamp_ns: int):
        # if self.sim_state == SimulatorState.AV_CONNECTING:
        #     if ctrl.payload.get("pedal", 0) != 0 or ctrl.payload.get("wheel", 0) != 0:
        #         self.sim_state = SimulatorState.RUNNING
        #         logger.info("AV engaged.")
        #     return None

        dt_s = (time_stamp_ns - self._time_ns) / 1e9
        self._time_ns = time_stamp_ns

        se = self.se

        # Update vehicle control
        self.ego_car.apply_control(ctrl, dt_s)

        obj_id = se.SE_GetId(0)
        se.SE_ReportObjectPosXYH(
            obj_id,
            0.0,
            self.ego_car.vh_state.x,
            self.ego_car.vh_state.y,
            self.ego_car.vh_state.h,
        )
        se.SE_ReportObjectWheelStatus(
            obj_id,
            self.ego_car.vh_state.wheel_rotation,
            self.ego_car.vh_state.wheel_angle,
        )
        se.SE_ReportObjectSpeed(
            obj_id,
            self.ego_car.vh_state.speed,
        )

        se.SE_StepDT(dt_s)

        # Update object state
        for i in range(0, self.obj_count):
            # Get object state
            obj_state = SEScenarioObjectState()
            ret_state = se.SE_GetObjectState(se.SE_GetId(i), ct.byref(obj_state))

            # Get object acceleration
            obj_accel = se.SE_GetObjectAcceleration(se.SE_GetId(i))

            # Get object angular velocity
            h_rate = ct.c_float()
            p_rate = ct.c_float()
            r_rate = ct.c_float()
            ret_rate = se.SE_GetObjectAngularVelocity(
                se.SE_GetId(i), ct.byref(h_rate), ct.byref(p_rate), ct.byref(r_rate)
            )

            # Get object angular acceleration
            h_acc = ct.c_float()
            p_acc = ct.c_float()
            r_acc = ct.c_float()
            ret_acc = se.SE_GetObjectAngularAcceleration(
                se.SE_GetId(i), ct.byref(h_acc), ct.byref(p_acc), ct.byref(r_acc)
            )

            if ret_state != 0:
                logger.warning(f"SE_GetObjectState failed for object id {i}")
                print(f"ret = {ret_state}, id = {se.SE_GetId(i)}")
                continue

            kinematic = ObjectKinematic(
                time_ns=int(obj_state.timestamp * 1e9),
                x=float(obj_state.x),
                y=float(obj_state.y),
                z=float(obj_state.z),
                yaw=float(obj_state.h),
                speed=float(obj_state.speed),
                acceleration=float(obj_accel),
                yaw_rate=float(h_rate.value) if ret_rate == 0 else 0.0,
                yaw_acceleration=float(h_acc.value) if ret_acc == 0 else 0.0,
            )
            self.objects[i].update(kinematic)
        return self.objects

    def stop(self):
        self.se.SE_Close()
        if self.ego_car is not None:
            self.se.SE_SimpleVehicleDelete(self.ego_car.sv_handle)
            self.ego_car = None
        logger.info("Esmini simulator stopped.")

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
                logger.warning(f"Parameter {name} has unknown type {ptype}. Skip.")
                continue

        # 這個 return 給自己用就好，C callback 是 void，不會用到
        return 0

    def _get_parameter(self, sps: ScenarioPack) -> dict[str, Any]:
        self

    def reset(self, output_dir: Path, sps: ScenarioPack, params: Optional[dict] = None):
        self._output_dir = output_dir

        self.stop()

        # Reset time
        self._time_ns = 0

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
        use_viewer, threads, record = self._setup_esmini_opts()

        map_path = Path(sps.maps["xodr_path"])
        self.se.SE_AddPath(str(map_path.parent).encode())
        disable_controller = 1  # 0 to enable built-in controllers, 1 to disable
        ret = self.se.SE_Init(
            str(sps.scenarios["xosc"]).encode(),
            disable_controller,
            use_viewer,
            threads,
            record,
        )
        if ret != 0:
            raise RuntimeError(f"esmini SE_Init failed with code {ret}")

        self.obj_count = self.se.SE_GetNumberOfObjects()
        self.objects = []
        for i in range(0, self.obj_count):
            obj_state = SEScenarioObjectState()
            self.se.SE_GetObjectState(self.se.SE_GetId(i), ct.byref(obj_state))

            esmini_obj_type = int(obj_state.objectType)
            obj_category = int(obj_state.objectCategory)
            obj_type = RoadObjectType.UNKNOWN
            if esmini_obj_type == 2:  # Pedestrian type
                if obj_category == 0:  # Pedestrian
                    obj_type = RoadObjectType.PEDESTRIAN
                elif obj_category == 1:  # Wheelchair
                    obj_type = RoadObjectType.WHEELCHAIR
                elif obj_category == 2:  # Animal
                    obj_type = RoadObjectType.ANIMAL
                else:
                    obj_type = RoadObjectType.UNKNOWN
            else:  # Vehicle type
                obj_type = TYPE_MAP.get(obj_category, RoadObjectType.UNKNOWN)

            obj_kinematic = ObjectKinematic(
                time_ns=int(obj_state.timestamp * 1e9),
                x=float(obj_state.x),
                y=float(obj_state.y),
                z=float(obj_state.z),
                yaw=float(obj_state.h),
                speed=float(obj_state.speed),
            )

            obj_shape = Shape(
                type=ShapeType.BOUNDING_BOX,
                dimensions=(
                    float(obj_state.length),
                    float(obj_state.width),
                    float(obj_state.height),
                ),
            )
            obj = ObjectState.create(
                type=obj_type,
                kinematic=obj_kinematic,
                shape=obj_shape,
            )

            self.objects.append(obj)

        # Create ego vehicle helper
        self.ego_car = Vehicle(
            self.se,
            x=float(self.objects[0].kinematic.x),
            y=float(self.objects[0].kinematic.y),
            h=float(self.objects[0].kinematic.yaw),
            length=float(self.objects[0].shape.dimensions[0]),
            speed=float(self.objects[0].kinematic.speed),
        )
        return self.objects

    # define a function returning if the simulator need to stop
    def should_quit(self):
        return self.se.SE_GetQuitFlag()
