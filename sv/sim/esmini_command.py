import subprocess, pathlib, json, tempfile, pathlib, time
from sv.registry import register_sim


@register_sim("esmini_command")
class EsminiAdapter:
    def prepare(self, sps: dict):
        self.osc = pathlib.Path("./scenarios") / sps["artifacts"]["osc"][0]
        assert self.osc.exists(), f"OSC not found: {self.osc}"

    def start(self, cfg: dict):
        self.dt = cfg.get("dt", 0.01)
        self.tmpdir = pathlib.Path(tempfile.mkdtemp())
        # 啟動 esmini（headless）輸出軌跡/事件（參考 esmini CLI）
        self.proc = subprocess.Popen(
            [
                "xvfb-run",
                "-a",
                "esmini",
                "--osc",
                str(self.osc),
                "--headless",
                "--fixed_timestep",
                str(self.dt),
                "--path",
                "/opt/esmini/resources/xosc/",
                "--path",
                "/opt/esmini/resources/Catalogs/",
                "--record",
                cfg.get("record", str(self.tmpdir / "sim.dat")),
                #   "--stdout_log", str(self.tmpdir/"esmini.log"),
                #   "--write_osi","--write_osi_file", str(self.tmpdir/"osi.txt")
            ]
        )

    def step(self, dt: float):
        # 真實情況：不逐 tick 拉；此教學返回空事件，重點在整體跑完
        return {"t": 0, "ego": {}, "agents": []}, []

    def stop(self):
        self.proc.wait(timeout=60)
        # TODO: 解析 /tmp/osi.txt → 事件/軌跡，寫入 artifacts
