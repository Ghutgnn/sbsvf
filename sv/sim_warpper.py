from pathlib import Path
from typing import Any, Optional
import logging

import grpc
from google.protobuf.struct_pb2 import Struct
from carla_api import (
    carla_pb2,
    carla_pb2_grpc,
    config_pb2,
    control_pb2,
)

from sv.utils.control import Ctrl
from sv.utils.sps import ScenarioPack
from sv.utils.util import get_cfg

logger = logging.getLogger(__name__)


class SimWrapper:
    def __init__(self, sim_spec: dict, dt_ns: int = None):
        self._sim_spec = sim_spec
        if dt_ns is None:
            logger.warning("dt not specified for SimWrapper, defaulting to 0.01s")
            self._dt_s = 0.01
        else:
            self._dt_s = dt_ns / 1e9

        self._url = self._sim_spec.get("url", "llt.hcislab.org:50051")
        self._timeout = float(self._sim_spec.get("timeout", 10.0))
        self._sim_cfg_path = self._sim_spec.get("config_path", None)

        if self._sim_cfg_path is not None:
            self._sim_cfg = get_cfg(self._sim_cfg_path)
        else:
            self._sim_cfg = None

        self._channel = None
        self._stub = None
        self._connected = False

        self.init()

    # ---------------------------
    # Public API
    # ---------------------------
    def init(self):
        # long-lived channel
        self._channel = grpc.insecure_channel(self._url)
        self._stub = carla_pb2_grpc.CarlaSimStub(self._channel)

        # Ping
        # try:
        #     pong = self._stub.Ping(carla_pb2.Empty(), timeout=self._timeout)
        #     print(f"Ping response: {pong.msg}")
        # except grpc.RpcError as e:
        #     raise RuntimeError(f"Ping failed: {e.code().name} - {e.details()}") from e

        cfg_struct = Struct()
        cfg_struct.update(self._sim_cfg if self._sim_cfg is not None else {})
        config = config_pb2.Config(config=cfg_struct)
        request = carla_pb2.InitRequest(
            config=config,
            dt=self._dt_s,
        )
        response = self._stub.Init(request, timeout=self._timeout)
        logger.info(f"Init response: {response.msg}")
        if not response.success:
            raise RuntimeError(f"Server Init returned success=false: {response.msg}")

        self._connected = True

    def reset(
        self,
        output_dir: Path,
        scenario_pack: ScenarioPack,
        params: Optional[dict[str, Any]] = {},
    ):
        self._ensure_ready()

        req = carla_pb2.ResetRequest(
            output_dir=str(output_dir),
            scenario_pack=scenario_pack.to_protobuf(),
            params=params,
        )
        try:
            resp = self._stub.Reset(req, timeout=self._timeout)
            return resp.objects
        except grpc.RpcError as e:
            raise RuntimeError(f"Reset failed: {e.code().name} - {e.details()}") from e

    def step(self, ctrl_cmd: Ctrl, time_stamp_ns: int):
        self._ensure_ready()

        if ctrl_cmd == None:
            return control_pb2.CtrlCmd(mode=control_pb2.CtrlMode.NONE)  # 空的 CtrlCmd

        # payload = Struct()
        # payload.update(ctrl_cmd.payload)

        # ctrl_pb = control_pb2.CtrlCmd(
        #     mode=control_pb2.CtrlMode.ACKERMANN,  # 根據 Ctrl.mode 決定 CtrlMode
        #     payload=payload,
        # )

        req = carla_pb2.StepRequest(ctrl_cmd=ctrl_cmd.to_pb(), timestamp_ns=int(time_stamp_ns))
        try:
            resp = self._stub.Step(req, timeout=self._timeout)
            # StepResponse { repeated ObjectState objects }
            return resp.objects
        except grpc.RpcError as e:
            raise RuntimeError(f"Step failed: {e.code().name} - {e.details()}") from e

    def stop(self):
        """
        rpc Stop(Empty) returns (Empty)
        """
        if self._stub is None:
            return
        try:
            self._stub.Stop(carla_pb2.Empty(), timeout=min(self._timeout, 5.0))
        except grpc.RpcError as e:
            logger.warning(f"[WARN] Stop failed: {e.code().name} - {e.details()}")
        finally:
            self._connected = False
            self._close()

    def should_quit(self) -> bool:
        """
        rpc ShouldQuit(Empty) returns (ShouldQuitResponse)
        """
        if self._stub is None or not self._connected:
            return True
        try:
            resp = self._stub.ShouldQuit(
                carla_pb2.Empty(), timeout=min(self._timeout, 2.0)
            )
            return bool(resp.should_quit)
        except grpc.RpcError:
            # server 抖一下不要直接判 quit
            return False

    # ---------------------------
    # Helpers
    # ---------------------------

    # ---------------------------
    # Internal
    # ---------------------------
    def _ensure_ready(self):
        if self._stub is None or self._channel is None or not self._connected:
            raise RuntimeError("SimWrapper not initialized. Call init() first.")

    def _close(self):
        if self._channel is not None:
            try:
                self._channel.close()
            except Exception:
                pass
        self._channel = None
        self._stub = None
