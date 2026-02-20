import tkinter as tk
from tkinter import ttk, messagebox
import yaml
import subprocess
import os
import signal


class SVProConfigGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SV Simulation Pro (Full Options)")
        self.process = None

        # --- 1. 選單選項設定 (在此輕鬆增加新選項) ---
        self.options = {
            "av_modules": [
                "sv.av.autoware_pure:AutowarePureAV",
                "sv.av.autoware_pure_pb:AutowarePureAV",
            ],
            "av_configs": ["configs/av/autoware_pure.yaml", "configs/av/test_av.yaml"],
            "sim_modules": [
                "sv.sim.esmini:EsminiAdapter",
                "sv.sim.esmini_pb:EsminiAdapter",
                "sv.sim.carla:CarlaAdapter",
            ],
            "sim_configs": [
                "configs/sim/simcfg_esmini_docker.yaml",
                "configs/sim/simcfg_carla.yaml",
            ],
            "samplers": [
                "sv.sampler.grid_search_sampler:GridSearchSampler",
                "sv.sampler.random_sampler:RandomSampler",
            ],
            "goal_types": ["LanePosition", "WorldPosition"],
        }

        # --- 2. 場景預設值 ---
        self.scenario_presets = {
            "DEMO_CI_e6mini": {
                "scenario_path": "./scenarios/mvp/scenarios/DEMO_CI_e6mini.xosc",
                "xodr_path": "./scenarios/mvp_distribution/maps/e6mini.xodr",
                "osm_path": "./scenarios/mvp/maps/e6mini_same.osm",
                "target_speed": "50",
                "goal_type": "LanePosition",
                "goal_val": "0, -4, 700, 0",
            },
            "01FOLLOW_02LEAD_196_209_f2701": {
                "scenario_path": "./scenarios/ego_196_KEEP/01FOLLOW_02LEAD_196_209_f2701.xosc",
                "xodr_path": "./scenarios/maps/taoyuan_minsheng_testing.xodr",
                "osm_path": "./scenarios/maps/location18_fixed.osm",
                "target_speed": "60",
                "goal_type": "LanePosition",
                "goal_val": "11, 2, 13.0, -1.5595416434466642",
            },
            "01KEEP_02CUTIN_L_196_211_f2965": {
                "scenario_path": "./scenarios/ego_196_KEEP/01KEEP_02CUTIN_L_196_211_f2965.xosc",
                "xodr_path": "./scenarios/maps/taoyuan_minsheng_testing.xodr",
                "osm_path": "./scenarios/maps/location18_fixed.osm",
                "target_speed": "60",
                "goal_type": "LanePosition",
                "goal_val": "11, 2, 13.0, -1.5595416434466642",
            },
            "01KEEP_02CUTIN_R_196_179_f2713": {
                "scenario_path": "./scenarios/ego_196_KEEP/01KEEP_02CUTIN_R_196_179_f2713.xosc",
                "xodr_path": "./scenarios/maps/taoyuan_minsheng_testing.xodr",
                "osm_path": "./scenarios/maps/location18_fixed.osm",
                "target_speed": "60",
                "goal_type": "LanePosition",
                "goal_val": "11, 2, 13.0, -1.5595416434466642",
            },
            "01KEEP_02TL_196_199_f2708": {
                "scenario_path": "./scenarios/ego_196_KEEP/01KEEP_02TL_196_199_f2708.xosc",
                "xodr_path": "./scenarios/maps/taoyuan_minsheng_testing.xodr",
                "osm_path": "./scenarios/maps/location18_fixed.osm",
                "target_speed": "60",
                "goal_type": "LanePosition",
                "goal_val": "11, 2, 13.0, -1.5595416434466642",
            },
            "01KEEP_02TW_UNDERPASS_196_194_f3201": {
                "scenario_path": "./scenarios/ego_196_KEEP/01KEEP_02TW_UNDERPASS_196_194_f3201.xosc",
                "xodr_path": "./scenarios/maps/taoyuan_minsheng_testing.xodr",
                "osm_path": "./scenarios/maps/location18_fixed.osm",
                "target_speed": "60",
                "goal_type": "LanePosition",
                "goal_val": "11, 2, 13.0, -1.5595416434466642",
            },
            "av_wrapper_test": {
                "scenario_path": "./scenarios/ego_196_KEEP/01FOLLOW_02LEAD_196_209_f2701.xosc",
                "xodr_path": "./scenarios/maps/taoyuan_minsheng_testing.xodr",
                "osm_path": "/mnt/maps/location18_fixed.osm",
                "target_speed": "60",
                "goal_type": "LanePosition",
                "goal_val": "11, 2, 13.0, -1.5595416434466642",
            },
        }

        self.create_widgets()

    def create_widgets(self):
        canvas = tk.Canvas(self.root)
        scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=canvas.yview)
        self.scroll_frame = ttk.Frame(canvas, padding="15")

        self.scroll_frame.bind(
            "<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        canvas.create_window((0, 0), window=self.scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        curr_row = 0

        # --- Section: Task ---
        self.add_section_header("1. Task Settings", curr_row)
        curr_row += 1
        self.worker_id = self.add_entry("Worker ID:", "test_worker_01", curr_row)
        curr_row += 1
        self.dt = self.add_entry("Runtime dt:", "0.01", curr_row)
        curr_row += 1

        # --- Section: Scenario Preset ---
        self.add_section_header("2. Scenario (Quick Select)", curr_row)
        curr_row += 1
        self.preset_var = self.add_option_menu(
            "Preset:",
            list(self.scenario_presets.keys()),
            curr_row,
            self.on_preset_change,
        )
        curr_row += 1
        self.scen_path = self.add_entry("Scenario Path:", "", curr_row)
        curr_row += 1
        self.xodr_path = self.add_entry("XODR Path:", "", curr_row)
        curr_row += 1
        self.osm_path = self.add_entry("OSM Path:", "", curr_row)
        curr_row += 1
        self.target_speed = self.add_entry("Target Speed:", "", curr_row)
        curr_row += 1
        self.goal_type = self.add_option_menu(
            "Goal Type:", self.options["goal_types"], curr_row
        )
        curr_row += 1
        self.goal_val = self.add_entry("Goal Value:", "", curr_row)
        curr_row += 1

        # --- Section: Modules (All Menus) ---
        self.add_section_header("3. Module Configurations", curr_row)
        curr_row += 1
        self.av_mod = self.add_option_menu(
            "AV Module:", self.options["av_modules"], curr_row
        )
        curr_row += 1
        self.av_cfg = self.add_option_menu(
            "AV Config:", self.options["av_configs"], curr_row
        )
        curr_row += 1
        self.sim_mod = self.add_option_menu(
            "Sim Module:", self.options["sim_modules"], curr_row
        )
        curr_row += 1
        self.sim_cfg = self.add_option_menu(
            "Sim Config:", self.options["sim_configs"], curr_row
        )
        curr_row += 1
        self.sampler_mod = self.add_option_menu(
            "Sampler:", self.options["samplers"], curr_row
        )
        curr_row += 1

        # 初始化預設
        self.on_preset_change(list(self.scenario_presets.keys())[0])

        # --- Buttons ---
        btn_frame = ttk.Frame(self.scroll_frame, padding="20")
        btn_frame.grid(row=curr_row, column=0, columnspan=2)
        self.run_btn = ttk.Button(
            btn_frame, text="RUN (run-one)", command=self.run_task
        )
        self.run_btn.pack(side=tk.LEFT, padx=10)
        self.stop_btn = ttk.Button(
            btn_frame, text="STOP & CLEAN", command=self.stop_task, state=tk.DISABLED
        )
        self.stop_btn.pack(side=tk.LEFT, padx=10)

    # --- UI Helpers ---
    def add_section_header(self, text, row):
        lbl = ttk.Label(
            self.scroll_frame,
            text=text,
            font=("Helvetica", 12, "bold"),
            foreground="#2c3e50",
        )
        lbl.grid(row=row, column=0, columnspan=2, sticky=tk.W, pady=(15, 5))

    def add_entry(self, label, default, row):
        ttk.Label(self.scroll_frame, text=label).grid(row=row, column=0, sticky=tk.W)
        var = tk.StringVar(value=default)
        ttk.Entry(self.scroll_frame, textvariable=var, width=55).grid(
            row=row, column=1, sticky=tk.W, pady=2
        )
        return var

    def add_option_menu(self, label, options, row, command=None):
        ttk.Label(self.scroll_frame, text=label).grid(row=row, column=0, sticky=tk.W)
        var = tk.StringVar(value=options[0])
        # 如果有傳入 command，則選單變動時會觸發
        menu = ttk.OptionMenu(
            self.scroll_frame, var, options[0], *options, command=command
        )
        menu.grid(row=row, column=1, sticky=tk.W, pady=2)
        return var

    def on_preset_change(self, choice):
        d = self.scenario_presets[choice]
        self.scen_path.set(d["scenario_path"])
        self.xodr_path.set(d["xodr_path"])
        self.osm_path.set(d["osm_path"])
        self.target_speed.set(d["target_speed"])
        self.goal_type.set(d["goal_type"])
        self.goal_val.set(d["goal_val"])

    # --- Logic ---
    def run_task(self):
        def parse_list(s):
            return [float(x.strip()) for x in s.split(",")] if s.strip() else None

        speed = float(self.target_speed.get() or 0)
        config = {
            "task": {"worker_id": self.worker_id.get(), "output_dir": "./artifacts"},
            "av": {"module_path": self.av_mod.get(), "config_path": self.av_cfg.get()},
            "map": {
                "name": self.preset_var.get(),
                "xodr_path": self.xodr_path.get(),
                "osm_path": self.osm_path.get(),
            },
            "scenario": {
                "title": self.preset_var.get(),
                "scenario_path": self.scen_path.get(),
                "ego": {
                    "target_speed": speed,
                    "spawn": {
                        "type": "LanePosition",
                        "value": [0, -2, 300, 0],
                        "speed": speed,
                    },
                    "goal": {
                        "type": self.goal_type.get(),
                        "value": parse_list(self.goal_val.get()),
                    },
                },
            },
            "simulator": {
                "module_path": self.sim_mod.get(),
                "config_path": self.sim_cfg.get(),
            },
            "sampler": {"module_path": self.sampler_mod.get()},
            "runtime": {"dt": float(self.dt.get() or 0.01)},
        }

        with open("pro_config.yaml", "w") as f:
            yaml.dump(config, f, sort_keys=False)

        self.process = subprocess.Popen(
            ["python3", "-m", "sv.cli", "run-one", "pro_config.yaml"]
        )
        self.run_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)

    def stop_task(self):
        if self.process:
            os.kill(self.process.pid, signal.SIGTERM)
            self.process = None
        subprocess.run(["just", "clean"])
        self.run_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        messagebox.showinfo("Done", "Stopped and Cleaned.")


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1920x1080")
    app = SVProConfigGUI(root)
    root.mainloop()
