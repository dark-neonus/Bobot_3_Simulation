from __future__ import annotations

import argparse
from pathlib import Path
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze Bobot-3 simulation telemetry")
    parser.add_argument("--input", type=str, default="data/sim_log.csv", help="Input telemetry CSV")
    parser.add_argument("--output", type=str, default="data/step_response.png", help="Output plot image path")
    parser.add_argument("--disturbance-threshold-deg", type=float, default=5.0, help="Tilt threshold for disturbance detection")
    parser.add_argument("--settling-band-deg", type=float, default=0.5, help="Settled band around target tilt")
    parser.add_argument("--settle-window-s", type=float, default=0.25, help="Time that tilt must stay in band")
    return parser.parse_args()


def detect_recovery(
    timestamps: np.ndarray,
    tilt_deg: np.ndarray,
    disturbance_threshold_deg: float,
    settling_band_deg: float,
    settle_window_s: float,
) -> Tuple[float | None, int | None, int | None]:
    if len(timestamps) < 4:
        return None, None, None

    abs_tilt = np.abs(tilt_deg)
    disturbance_indices = np.where(abs_tilt >= disturbance_threshold_deg)[0]
    if disturbance_indices.size == 0:
        return None, None, None

    start_idx = int(disturbance_indices[0])
    dt = float(np.median(np.diff(timestamps)))
    hold_count = max(1, int(settle_window_s / max(dt, 1e-9)))

    for idx in range(start_idx + 1, len(tilt_deg) - hold_count):
        segment = abs_tilt[idx : idx + hold_count]
        if np.all(segment <= settling_band_deg):
            return timestamps[idx] - timestamps[start_idx], start_idx, idx

    return None, start_idx, None


def smooth_series(values: np.ndarray) -> np.ndarray:
    if len(values) < 9:
        return values

    window = min(101, len(values) // 2 * 2 - 1)
    if window < 5:
        return values
    return savgol_filter(values, window_length=window, polyorder=3)


def main() -> None:
    args = parse_args()

    root_dir = Path(__file__).resolve().parent.parent
    input_path = Path(args.input)
    output_path = Path(args.output)
    if not input_path.is_absolute():
        input_path = root_dir / input_path
    if not output_path.is_absolute():
        output_path = root_dir / output_path

    if not input_path.exists():
        raise FileNotFoundError(f"Telemetry file not found: {input_path}")

    df = pd.read_csv(input_path)
    if "timestamp_s" not in df.columns:
        raise ValueError("CSV must include timestamp_s column")

    tilt_col = "pitch_true_deg" if "pitch_true_deg" in df.columns else "base_pitch_deg"
    if tilt_col not in df.columns:
        raise ValueError("CSV must include either pitch_true_deg or base_pitch_deg")

    torque_col = "left_torque_after_backlash_nm"
    if torque_col not in df.columns:
        raise ValueError("CSV must include left_torque_after_backlash_nm")

    timestamps = df["timestamp_s"].to_numpy(dtype=float)
    tilt = df[tilt_col].to_numpy(dtype=float)
    tilt_smooth = smooth_series(tilt)

    recovery_time_s, disturbance_idx, recovered_idx = detect_recovery(
        timestamps,
        tilt_smooth,
        args.disturbance_threshold_deg,
        args.settling_band_deg,
        args.settle_window_s,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    axes[0].plot(timestamps, tilt, color="#4f79d8", alpha=0.35, label="Tilt raw")
    axes[0].plot(timestamps, tilt_smooth, color="#1f3f91", linewidth=2.0, label="Tilt smoothed")
    axes[0].axhline(args.settling_band_deg, color="#228b22", linestyle="--", linewidth=1)
    axes[0].axhline(-args.settling_band_deg, color="#228b22", linestyle="--", linewidth=1)
    axes[0].set_ylabel("Tilt [deg]")
    axes[0].legend(loc="upper right")
    axes[0].grid(alpha=0.3)

    axes[1].plot(timestamps, df[torque_col], color="#c7502e", linewidth=1.4)
    axes[1].set_ylabel("Wheel Torque [Nm]")
    axes[1].grid(alpha=0.3)

    if "battery_voltage_v" in df.columns:
        axes[2].plot(timestamps, df["battery_voltage_v"], color="#6d3fb3", linewidth=1.4)
        axes[2].set_ylabel("Battery V [V]")
    else:
        axes[2].plot(timestamps, np.zeros_like(timestamps), color="#666666", linewidth=1.0)
        axes[2].set_ylabel("Battery V [V]")
    axes[2].set_xlabel("Time [s]")
    axes[2].grid(alpha=0.3)

    if disturbance_idx is not None:
        t_disturb = timestamps[disturbance_idx]
        for ax in axes:
            ax.axvline(t_disturb, color="#333333", linestyle=":", linewidth=1.2)

    if recovered_idx is not None:
        t_recover = timestamps[recovered_idx]
        for ax in axes:
            ax.axvline(t_recover, color="#2b9e2b", linestyle=":", linewidth=1.2)

    title = "Bobot-3 Step Response"
    if recovery_time_s is not None:
        title += f" | Recovery time: {recovery_time_s:.3f}s"
    else:
        title += " | Recovery not detected"
    fig.suptitle(title)

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)

    if recovery_time_s is None:
        print("Recovery not detected in selected telemetry window.")
    else:
        print(f"Estimated recovery time: {recovery_time_s:.3f}s")
    print(f"Saved analysis plot to: {output_path}")


if __name__ == "__main__":
    main()
