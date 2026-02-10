import logging
from typing import Any, Optional
import ctypes as ct
from pathlib import Path

from sv.registry import register_av
from sv.utils.control import Ctrl, CtrlMode
from sv.utils.object import ObjectState
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

    def apply_control(self, obs: dict[str, Any], dt_s: float) -> None:
        x = obs.get("x", 0.0)
        y = obs.get("y", 0.0)
        h = obs.get("h", 0.0)
        logger.debug(f"AV Applying control: x={x}, y={y}")
        self._se.SE_ReportObjectPosXYH(self._se.SE_GetId(1), dt_s, x, y, h)


@register_av("esmini_embed")
class EsminiEmbedAV:
    def __init__(self, output_dir: Path, cfg_path: Path):
        self._output_dir = output_dir
        self.cfg = get_cfg(cfg_path)
        self.esmini_home = self.cfg.get("esmini_home", "/opt/esmini/")
        self.extra_paths = self.cfg.get("path", [])
        self.log_file_path = self.cfg.get("log_file_path", "./esmini_log.txt")
        self.se = ct.CDLL(self.esmini_home + "bin/libesminiLib.so")
        self._agents: list[Vehicle] = []
        self._time_ns = 0
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

    def reset(
        self,
        output_dir: Path,
        sps: ScenarioPack,
        init_obs: Optional[list[ObjectState]] = None,
    ):
        self._output_dir = output_dir
        self.stop()

        use_viewer, threads, record = self._setup_esmini_opts()
        disable_controller = 0  # 0 to enable built-in controllers, 1 to disable
        map_path = Path(sps.maps["xodr_path"])
        self.se.SE_AddPath(str(map_path.parent).encode())
        ret = self.se.SE_Init(
            str(sps.maps["dummy"]).encode(),
            disable_controller,
            use_viewer,
            threads,
            record,
        )
        if ret != 0:
            raise RuntimeError(f"esmini SE_Init failed with code {ret}")
        # obj_state = SEScenarioObjectState()
        # self.se.SE_GetObjectState(self.se.SE_GetId(1), ct.byref(obj_state))

        ego_kinematic = init_obs[0].kinematic
        ego_shape = init_obs[0].shape

        self._agent = init_obs[1:] if len(init_obs) > 1 else []
        # for i in range(len(init_obs) - 1):
        # obj_kinematic = init_obs[i + 1].kinematic
        # obj_shape = init_obs[i + 1].shape
        # self._agents.append(
        #     Vehicle(
        #         self.se,
        #         obj_kinematic.x,
        #         obj_kinematic.y,
        #         obj_kinematic.yaw,
        #         obj_shape.dimensions[0],
        #         obj_kinematic.speed,
        #     )
        # )
        logger.info(
            "AV reset complete, ego initialized at x=%.2f, y=%.2f, h=%.2f, speed=%.2f",
            ego_kinematic.x,
            ego_kinematic.y,
            ego_kinematic.yaw,
            ego_kinematic.speed,
        )
        return Ctrl(
            mode=CtrlMode.POSITION,
            payload={
                "x": float(ego_kinematic.x),
                "y": float(ego_kinematic.y),
                "h": float(ego_kinematic.yaw),
            },
        )

    def step(self, obs: list[ObjectState], time_stamp_ns: int) -> Ctrl:
        dt_s = (time_stamp_ns - self._time_ns) / 1e9
        self._time_ns = time_stamp_ns
        se = self.se
        self._agents = obs[1:]  # Skip the ego vehicle (index 0)
        for id, agent in enumerate(self._agents):
            # agent.apply_control(obs[id], dt_s)

            # Update the agent's state after applying control
            obj_id = se.SE_GetId(id + 1)
            se.SE_ReportObjectPosXYH(
                obj_id,
                0.0,
                self._agents[id].kinematic.x,
                self._agents[id].kinematic.y,
                self._agents[id].kinematic.yaw,
            )
            # se.SE_ReportObjectWheelStatus(
            #     obj_id,
            #     self._agents[id].vh_state.wheel_rotation,
            #     self._agents[id].vh_state.wheel_angle,
            # )
            se.SE_ReportObjectSpeed(
                obj_id,
                self._agents[id].kinematic.speed,
            )

        obj_state = SEScenarioObjectState()
        self.se.SE_GetObjectState(self.se.SE_GetId(0), ct.byref(obj_state))

        self.se.SE_StepDT(dt_s)

        return Ctrl(
            mode=CtrlMode.POSITION,
            payload={
                "x": float(obj_state.x),
                "y": float(obj_state.y),
                "h": float(obj_state.h),
                "speed": float(obj_state.speed),
            },
        )

    def stop(self) -> None:
        self.se.SE_Close()
        # for agent in self._agents:
        #     self.se.SE_SimpleVehicleDelete(agent.sv_handle)
        self._agents.clear()

    def should_quit(self) -> bool:
        return self.se.SE_GetQuitFlag() != 0
