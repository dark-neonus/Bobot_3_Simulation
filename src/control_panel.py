from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
import re
from typing import Any, Dict

import tkinter as tk
from tkinter import ttk, messagebox


@dataclass(frozen=True)
class SliderSpec:
    key: str
    label: str
    min_value: float
    max_value: float
    default_value: float
    resolution: float


class TkControlPanel:
    """External control panel with readable labels and PID sliders."""

    def __init__(self, defaults: Dict[str, float]) -> None:
        self.closed = False
        self._pending_action: Dict[str, Any] | None = None
        self._preset_dir = Path(__file__).resolve().parent.parent / "data" / "presets"
        self._last_session_path = self._preset_dir / "__last_session__.json"
        self._preset_dir.mkdir(parents=True, exist_ok=True)

        merged_defaults = dict(defaults)
        merged_defaults.update(self._load_last_session())

        self.root = tk.Tk()
        self.root.title("Bobot-3 Control Panel")
        self.root.geometry("680x960")
        self.root.minsize(620, 860)
        self.root.configure(bg="#eef2f7")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure("Panel.TFrame", background="#eef2f7")
        style.configure("Header.TLabel", background="#eef2f7", font=("DejaVu Sans", 11, "bold"))
        style.configure("Body.TLabel", background="#eef2f7", font=("DejaVu Sans", 10))

        self._vars: Dict[str, tk.DoubleVar] = {}
        self._value_labels: Dict[str, ttk.Label] = {}
        self._metric_vars: Dict[str, tk.StringVar] = {}
        self._stable_pid_defaults: Dict[str, float] = {
            "pid_kp": 55.0,
            "pid_ki": 2.0,
            "pid_kd": 4.0,
            "max_torque_nm": 4.5,
            "control_polarity": -1.0,
            "target_pitch_deg": 0.0,
        }
        self._status_var = tk.StringVar(value="Ready")
        self._preset_name_var = tk.StringVar(value="")
        self._preset_listbox: tk.Listbox | None = None

        container = ttk.Frame(self.root, style="Panel.TFrame", padding=(14, 12))
        container.pack(fill="both", expand=True)

        title = ttk.Label(container, text="Runtime Control Panel", style="Header.TLabel")
        title.pack(anchor="w")
        subtitle = ttk.Label(
            container,
            text="Readable controls for PID tuning, motor constants, and drivetrain parameters",
            style="Body.TLabel",
        )
        subtitle.pack(anchor="w", pady=(0, 10))

        notebook = ttk.Notebook(container)
        notebook.pack(fill="both", expand=True)

        tab_pid = ttk.Frame(notebook, style="Panel.TFrame")
        tab_drive = ttk.Frame(notebook, style="Panel.TFrame")
        tab_power = ttk.Frame(notebook, style="Panel.TFrame")
        tab_tests = ttk.Frame(notebook, style="Panel.TFrame")
        tab_presets = ttk.Frame(notebook, style="Panel.TFrame")
        notebook.add(tab_pid, text="PID")
        notebook.add(tab_drive, text="Drive")
        notebook.add(tab_power, text="Power")
        notebook.add(tab_tests, text="Tests/Reset")
        notebook.add(tab_presets, text="Presets")

        pid_specs = [
            SliderSpec("pid_kp", "PID Kp", 0.0, 250.0, merged_defaults.get("pid_kp", 80.0), 0.1),
            SliderSpec("pid_ki", "PID Ki", 0.0, 60.0, merged_defaults.get("pid_ki", 5.0), 0.1),
            SliderSpec("pid_kd", "PID Kd", 0.0, 30.0, merged_defaults.get("pid_kd", 2.0), 0.1),
            SliderSpec("control_polarity", "Control Polarity", -1.0, 1.0, merged_defaults.get("control_polarity", -1.0), 1.0),
            SliderSpec("target_pitch_deg", "Target Pitch [deg]", -10.0, 10.0, merged_defaults.get("target_pitch_deg", 0.0), 0.1),
            SliderSpec("max_torque_nm", "Max Torque [Nm]", 0.2, 12.0, merged_defaults.get("max_torque_nm", 6.0), 0.1),
        ]

        drive_specs = [
            SliderSpec("drive_mode", "Drive Mode (0 direct, 1 gear, 2 belt, 3 combo)", 0.0, 3.0, merged_defaults.get("drive_mode", 0.0), 1.0),
            SliderSpec("reducer_ratio", "Reducer Ratio", 1.0, 10.0, merged_defaults.get("reducer_ratio", 4.0), 0.1),
            SliderSpec("belt_ratio", "Belt Ratio", 1.0, 4.0, merged_defaults.get("belt_ratio", 1.5), 0.1),
            SliderSpec("backlash_deg", "Backlash [deg]", 0.0, 12.0, merged_defaults.get("backlash_deg", 1.5), 0.1),
            SliderSpec("backlash_impact_gain", "Backlash Impact Gain", 0.0, 4.0, merged_defaults.get("backlash_impact_gain", 2.2), 0.05),
            SliderSpec("battery_z_m", "Battery Slider Z [m]", -0.20, 0.12, merged_defaults.get("battery_z_m", 0.0), 0.005),
        ]

        power_specs = [
            SliderSpec("battery_nominal_v", "Battery Nominal [V]", 15.0, 21.0, merged_defaults.get("battery_nominal_v", 18.5), 0.05),
            SliderSpec("battery_internal_r", "Battery Internal R [ohm]", 0.01, 0.50, merged_defaults.get("battery_internal_r", 0.12), 0.005),
            SliderSpec("battery_max_current_a", "Battery Max Current [A]", 1.0, 30.0, merged_defaults.get("battery_max_current_a", 15.0), 0.1),
            SliderSpec("motor_kt_nm_per_a", "Motor Kt [Nm/A]", 0.01, 0.30, merged_defaults.get("motor_kt_nm_per_a", 0.08), 0.005),
            SliderSpec("motor_ke_v_per_rad_s", "Motor Ke [V/(rad/s)]", 0.01, 0.30, merged_defaults.get("motor_ke_v_per_rad_s", 0.08), 0.005),
            SliderSpec("motor_phase_r_ohm", "Motor Phase R [ohm]", 0.05, 2.0, merged_defaults.get("motor_phase_r_ohm", 0.35), 0.01),
        ]

        test_specs = [
            SliderSpec("disturbance_torque_nm", "Disturbance Torque [Nm]", -2.0, 2.0, merged_defaults.get("disturbance_torque_nm", 0.0), 0.05),
            SliderSpec("reset_tilt_deg", "Reset Tilt Magnitude [deg]", 0.0, 20.0, merged_defaults.get("reset_tilt_deg", 6.0), 0.1),
        ]

        self._build_tab(tab_pid, pid_specs)
        self._build_pid_actions(tab_pid)
        self._build_tab(tab_drive, drive_specs)
        self._build_tab(tab_power, power_specs)
        self._build_tab(tab_tests, test_specs)
        self._build_reset_buttons(tab_tests)
        self._build_preset_tab(tab_presets)
        self._build_live_metrics(container)

        status_row = ttk.Frame(container, style="Panel.TFrame")
        status_row.pack(fill="x", pady=(8, 0))
        status_label = ttk.Label(status_row, textvariable=self._status_var, style="Body.TLabel")
        status_label.pack(anchor="w")

        self._refresh_preset_list()
        self._save_last_session()

    def _build_tab(self, parent: ttk.Frame, specs: list[SliderSpec]) -> None:
        parent.columnconfigure(0, weight=1)
        for row_index, spec in enumerate(specs):
            row = ttk.Frame(parent, style="Panel.TFrame")
            row.grid(row=row_index, column=0, sticky="ew", padx=8, pady=8)
            row.columnconfigure(0, weight=1)

            label = ttk.Label(row, text=spec.label, style="Body.TLabel")
            label.grid(row=0, column=0, sticky="w")

            value_label = ttk.Label(row, text=f"{spec.default_value:.3f}", style="Body.TLabel")
            value_label.grid(row=0, column=1, sticky="e")
            self._value_labels[spec.key] = value_label

            variable = tk.DoubleVar(value=spec.default_value)
            self._vars[spec.key] = variable

            scale = tk.Scale(
                row,
                from_=spec.min_value,
                to=spec.max_value,
                orient=tk.HORIZONTAL,
                resolution=spec.resolution,
                showvalue=False,
                length=420,
                variable=variable,
                command=lambda value, key=spec.key: self._on_value_change(key, value),
                highlightthickness=0,
                troughcolor="#d7deeb",
                bg="#eef2f7",
                activebackground="#8aa2d6",
                fg="#1d2b44",
                font=("DejaVu Sans", 9),
            )
            scale.grid(row=1, column=0, columnspan=2, sticky="ew")

    def _build_pid_actions(self, parent: ttk.Frame) -> None:
        row = ttk.Frame(parent, style="Panel.TFrame")
        row.grid(row=30, column=0, sticky="ew", padx=8, pady=(12, 6))
        row.columnconfigure(0, weight=1)

        button = ttk.Button(row, text="Apply Stable PID Defaults", command=self._apply_stable_pid_defaults)
        button.grid(row=0, column=0, sticky="ew")

    def _apply_stable_pid_defaults(self) -> None:
        for key, value in self._stable_pid_defaults.items():
            variable = self._vars.get(key)
            if variable is None:
                continue
            variable.set(value)
            if key in self._value_labels:
                self._value_labels[key].configure(text=f"{value:.3f}")

        self._status_var.set("Applied stable PID defaults")
        self._save_last_session()

    def _build_reset_buttons(self, parent: ttk.Frame) -> None:
        row = ttk.Frame(parent, style="Panel.TFrame")
        row.grid(row=40, column=0, sticky="ew", padx=8, pady=(16, 8))

        left_button = ttk.Button(row, text="Reset Tilt Left", command=lambda: self._queue_reset_tilt(+1.0))
        right_button = ttk.Button(row, text="Reset Tilt Right", command=lambda: self._queue_reset_tilt(-1.0))
        upright_button = ttk.Button(row, text="Reset Upright", command=self._queue_reset_upright)

        left_button.grid(row=0, column=0, padx=(0, 6), pady=2, sticky="ew")
        right_button.grid(row=0, column=1, padx=6, pady=2, sticky="ew")
        upright_button.grid(row=0, column=2, padx=(6, 0), pady=2, sticky="ew")

        row.columnconfigure(0, weight=1)
        row.columnconfigure(1, weight=1)
        row.columnconfigure(2, weight=1)

    def _build_preset_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=1)

        name_row = ttk.Frame(parent, style="Panel.TFrame")
        name_row.grid(row=0, column=0, sticky="ew", padx=8, pady=(10, 6))
        name_row.columnconfigure(1, weight=1)

        ttk.Label(name_row, text="Preset name", style="Body.TLabel").grid(row=0, column=0, sticky="w", padx=(0, 8))
        name_entry = ttk.Entry(name_row, textvariable=self._preset_name_var)
        name_entry.grid(row=0, column=1, sticky="ew")

        button_row = ttk.Frame(parent, style="Panel.TFrame")
        button_row.grid(row=1, column=0, sticky="ew", padx=8, pady=6)
        button_row.columnconfigure(0, weight=1)
        button_row.columnconfigure(1, weight=1)
        button_row.columnconfigure(2, weight=1)

        ttk.Button(button_row, text="Save Current", command=self._save_named_preset).grid(row=0, column=0, padx=(0, 6), sticky="ew")
        ttk.Button(button_row, text="Load Selected", command=self._load_selected_preset).grid(row=0, column=1, padx=6, sticky="ew")
        ttk.Button(button_row, text="Refresh List", command=self._refresh_preset_list).grid(row=0, column=2, padx=(6, 0), sticky="ew")

        list_frame = ttk.Frame(parent, style="Panel.TFrame")
        list_frame.grid(row=2, column=0, sticky="nsew", padx=8, pady=(8, 8))
        parent.rowconfigure(2, weight=1)
        list_frame.columnconfigure(0, weight=1)
        list_frame.rowconfigure(0, weight=1)

        self._preset_listbox = tk.Listbox(
            list_frame,
            height=18,
            font=("DejaVu Sans", 10),
            selectmode=tk.SINGLE,
            exportselection=False,
        )
        self._preset_listbox.grid(row=0, column=0, sticky="nsew")

        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self._preset_listbox.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self._preset_listbox.configure(yscrollcommand=scrollbar.set)

    def _build_live_metrics(self, parent: ttk.Frame) -> None:
        panel = ttk.Frame(parent, style="Panel.TFrame")
        panel.pack(fill="x", pady=(10, 2))

        title = ttk.Label(panel, text="Live Derived Motor Values", style="Header.TLabel")
        title.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 4))

        metric_specs = [
            ("Drive Mode", "drive_mode"),
            ("Drive Ratio", "drive_ratio"),
            ("Battery V [V]", "battery_voltage_v"),
            ("Left Motor RPM", "left_motor_rpm"),
            ("Right Motor RPM", "right_motor_rpm"),
            ("Left Motor Current [A]", "left_motor_current_a"),
            ("Right Motor Current [A]", "right_motor_current_a"),
            ("Left Motor Torque [Nm]", "left_output_torque_nm"),
            ("Right Motor Torque [Nm]", "right_output_torque_nm"),
            ("Left Torque After Backlash [Nm]", "left_torque_after_backlash_nm"),
            ("Right Torque After Backlash [Nm]", "right_torque_after_backlash_nm"),
        ]

        for index, (label_text, key) in enumerate(metric_specs):
            row = 1 + index // 2
            col_block = index % 2
            label_col = col_block * 2
            value_col = label_col + 1

            ttk.Label(panel, text=label_text, style="Body.TLabel").grid(
                row=row,
                column=label_col,
                sticky="w",
                padx=(0, 8),
                pady=1,
            )
            var = tk.StringVar(value="--")
            self._metric_vars[key] = var
            ttk.Label(panel, textvariable=var, style="Body.TLabel").grid(
                row=row,
                column=value_col,
                sticky="w",
                padx=(0, 16),
                pady=1,
            )

        panel.columnconfigure(0, weight=0)
        panel.columnconfigure(1, weight=1)
        panel.columnconfigure(2, weight=0)
        panel.columnconfigure(3, weight=1)

    def update_metrics(self, metrics: Dict[str, Any]) -> None:
        if self.closed:
            return

        try:
            for key, value in metrics.items():
                var = self._metric_vars.get(key)
                if var is None:
                    continue

                if isinstance(value, (int, float)):
                    if key.endswith("_rpm"):
                        text = f"{value:.1f}"
                    elif key == "drive_ratio":
                        text = f"{value:.3f}"
                    else:
                        text = f"{value:.3f}"
                else:
                    text = str(value)

                var.set(text)
        except tk.TclError:
            self.closed = True

    def _queue_reset_tilt(self, direction: float) -> None:
        magnitude = abs(self._vars.get("reset_tilt_deg", tk.DoubleVar(value=6.0)).get())
        tilt = direction * magnitude
        self._pending_action = {"type": "reset_pose", "tilt_deg": float(tilt)}
        side = "left" if direction > 0 else "right"
        self._status_var.set(f"Queued reset: {side} tilt {tilt:.2f} deg")

    def _queue_reset_upright(self) -> None:
        self._pending_action = {"type": "reset_pose", "tilt_deg": 0.0}
        self._status_var.set("Queued reset: upright")

    def consume_action(self) -> Dict[str, Any] | None:
        action = self._pending_action
        self._pending_action = None
        return action

    def _on_value_change(self, key: str, value: str) -> None:
        if key not in self._value_labels:
            return
        self._value_labels[key].configure(text=f"{float(value):.3f}")
        self._save_last_session()

    def _save_last_session(self) -> None:
        payload = self.get_values()
        if not payload:
            return
        payload["saved_at"] = datetime.now().isoformat(timespec="seconds")
        try:
            with self._last_session_path.open("w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, sort_keys=True)
        except OSError:
            pass

    def _load_last_session(self) -> Dict[str, float]:
        if not self._last_session_path.exists():
            return {}
        try:
            with self._last_session_path.open("r", encoding="utf-8") as handle:
                payload = json.load(handle)
            if not isinstance(payload, dict):
                return {}
            return {key: value for key, value in payload.items() if isinstance(value, (int, float))}
        except (OSError, json.JSONDecodeError):
            return {}

    def _preset_file_path(self, preset_name: str) -> Path:
        clean = re.sub(r"[^A-Za-z0-9_.-]+", "_", preset_name.strip())
        if not clean:
            clean = datetime.now().strftime("preset_%Y%m%d_%H%M%S")
        return self._preset_dir / f"{clean}.json"

    def _save_named_preset(self) -> None:
        target = self._preset_file_path(self._preset_name_var.get())
        payload = self.get_values()
        payload["saved_at"] = datetime.now().isoformat(timespec="seconds")
        try:
            with target.open("w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, sort_keys=True)
            self._preset_name_var.set(target.stem)
            self._status_var.set(f"Saved preset: {target.stem}")
            self._refresh_preset_list()
        except OSError as exc:
            messagebox.showerror("Preset Save Error", f"Could not save preset.\n{exc}")

    def _selected_preset_path(self) -> Path | None:
        if self._preset_listbox is None:
            return None
        selection = self._preset_listbox.curselection()
        if not selection:
            return None
        name = self._preset_listbox.get(selection[0])
        return self._preset_dir / f"{name}.json"

    def _load_selected_preset(self) -> None:
        selected = self._selected_preset_path()
        if selected is None:
            messagebox.showinfo("Load Preset", "Select a preset first.")
            return
        if not selected.exists():
            messagebox.showerror("Load Preset", f"Preset not found: {selected.name}")
            self._refresh_preset_list()
            return

        try:
            with selected.open("r", encoding="utf-8") as handle:
                payload = json.load(handle)
        except (OSError, json.JSONDecodeError) as exc:
            messagebox.showerror("Load Preset", f"Could not read preset.\n{exc}")
            return

        if not isinstance(payload, dict):
            messagebox.showerror("Load Preset", "Preset format is invalid.")
            return

        for key, variable in self._vars.items():
            value = payload.get(key)
            if isinstance(value, (int, float)):
                variable.set(float(value))
                if key in self._value_labels:
                    self._value_labels[key].configure(text=f"{float(value):.3f}")

        self._status_var.set(f"Loaded preset: {selected.stem}")
        self._save_last_session()

    def _refresh_preset_list(self) -> None:
        if self._preset_listbox is None:
            return

        current = None
        selection = self._preset_listbox.curselection()
        if selection:
            current = self._preset_listbox.get(selection[0])

        self._preset_listbox.delete(0, tk.END)
        preset_names = sorted(
            path.stem
            for path in self._preset_dir.glob("*.json")
            if path.name != self._last_session_path.name
        )
        for name in preset_names:
            self._preset_listbox.insert(tk.END, name)

        if current and current in preset_names:
            idx = preset_names.index(current)
            self._preset_listbox.selection_set(idx)

    def _on_close(self) -> None:
        self._save_last_session()
        self.closed = True
        try:
            self.root.withdraw()
            self.root.destroy()
        except tk.TclError:
            pass

    def poll(self) -> None:
        if self.closed:
            return
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            self.closed = True

    def get_values(self) -> Dict[str, float]:
        values: Dict[str, float] = {}
        for key, variable in self._vars.items():
            try:
                values[key] = float(variable.get())
            except tk.TclError:
                continue
        return values

    def close(self) -> None:
        self._on_close()
