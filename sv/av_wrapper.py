from pathlib import Path
from typing import Any, Optional
import logging

import grpc
from google.protobuf.struct_pb2 import Struct
from sbsvf_api import (
    av_server_pb2,
    av_server_pb2_grpc,
    config_pb2,
    control_pb2,
    empty_pb2,
    path_pb2,
)

from sv.utils.control import Ctrl
from sv.utils.object import ObjectState
from sv.utils.sps import ScenarioPack
from sv.utils.util import get_cfg

logger = logging.getLogger(__name__)


class AVWrapper:
    def __init__(
        self,
        output_dir: Path,
        av_spec: dict,
        dt_ns: int = None,
        sps: ScenarioPack = None,
    ):
        self._output_dir = output_dir
        self._av_spec = av_spec
        self._sps = sps

        if dt_ns is None:
            logger.warning("dt not specified for AVWrapper, defaulting to 0.01s")
            self._dt_s = 0.01
        else:
            self._dt_s = dt_ns / 1e9

        self._url = self._av_spec.get("url", "llt.hcislab.org:50052")
        self._timeout = float(self._av_spec.get("timeout", 100.0))
        self._av_cfg_path = self._av_spec.get("config_path", None)

        if self._av_cfg_path is not None:
            self._av_cfg = get_cfg(self._av_cfg_path)
        else:
            self._av_cfg = None

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
        self._stub = av_server_pb2_grpc.AvServerStub(self._channel)

        # Ping
        try:
            pong = self._stub.Ping(empty_pb2.Empty(), timeout=self._timeout)
            print(f"Ping response: {pong.msg}")
        except grpc.RpcError as e:
            raise RuntimeError(f"Ping failed: {e.code().name} - {e.details()}") from e

        cfg_struct = Struct()
        cfg_struct.update(self._av_cfg if self._av_cfg is not None else {})
        config = config_pb2.Config(config=cfg_struct)
        request = av_server_pb2.AvServerMessages.InitRequest(
            config=config,
            output_dir=path_pb2.Path(path=str(self._output_dir)),
            scenario_pack=self._sps.to_protobuf() if self._sps is not None else None,
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
        sps: ScenarioPack,
        init_obs: Optional[dict[str, Any]] = {},
    ):
        self._output_dir = output_dir
        self._sps = sps
        self._ensure_ready()
        req = av_server_pb2.AvServerMessages.ResetRequest(
            output_dir=path_pb2.Path(path=str(self._output_dir)),
            scenario_pack=self._sps.to_protobuf(),
            initial_observation=init_obs,
        )
        try:
            resp = self._stub.Reset(req, timeout=self._timeout)
            return resp.ctrl_cmd
        except grpc.RpcError as e:
            raise RuntimeError(f"Reset failed: {e.code().name} - {e.details()}") from e

    def step(self, obs, time_stamp_ns: int):
        self._ensure_ready()

        req = av_server_pb2.AvServerMessages.StepRequest(
            observation=obs, timestamp_ns=int(time_stamp_ns)
        )
        try:
            resp = self._stub.Step(req, timeout=self._timeout)
            # StepResponse { repeated ObjectState objects }
            return resp.ctrl_cmd
        except grpc.RpcError as e:
            raise RuntimeError(f"Step failed: {e.code().name} - {e.details()}") from e

    def stop(self):
        """
        rpc Stop(Empty) returns (Empty)
        """
        if self._stub is None:
            return
        try:
            self._stub.Stop(empty_pb2.Empty(), timeout=min(self._timeout, 5.0))
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
                empty_pb2.Empty(), timeout=min(self._timeout, 2.0)
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
            raise RuntimeError("AvWrapper not initialized. Call init() first.")

    def _close(self):
        if self._channel is not None:
            try:
                self._channel.close()
            except Exception:
                pass
        self._channel = None
        self._stub = None
