from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Any

import pybullet as p
import pybullet_data

from controller import PIDController, PIDGains
from logger import SimulationLogger
from physics import (
    BacklashTorqueGate,
    DRIVE_MODE_NAMES,
    EncoderModel,
    IMUSensorModel,
    MotorElectricalParams,
    SimpleMotorModel,
    cylinder_inertia,
    drive_ratio_from_mode,
)
from robot import Bobot3Robot, create_gui_controls, read_gui_controls

try:
    from control_panel import TkControlPanel
except Exception:
    TkControlPanel = None


@dataclass
class RuntimeTuning:
    pid_kp: float = 55.0
    pid_ki: float = 2.0
    pid_kd: float = 4.0
    control_polarity: float = -1.0
    target_pitch_deg: float = 0.0
    max_torque_nm: float = 4.5
    backlash_deg: float = 1.5
    backlash_impact_gain: float = 2.2
    drive_mode: int = 0
    reducer_ratio: float = 4.0
    belt_ratio: float = 1.5
    battery_z_m: float = 0.0
    battery_nominal_v: float = 18.5
    battery_internal_r: float = 0.12
    battery_max_current_a: float = 15.0
    motor_kt_nm_per_a: float = 0.08
    motor_ke_v_per_rad_s: float = 0.08
    motor_phase_r_ohm: float = 0.35
    disturbance_torque_nm: float = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bobot-3 dynamics and control simulation")
    parser.add_argument("--headless", action="store_true", help="Run in DIRECT mode without GUI")
    parser.add_argument("--duration", type=float, default=20.0, help="Simulation duration in seconds")
    parser.add_argument("--physics-freq", type=int, default=2000, help="Physics step frequency in Hz")
    parser.add_argument("--control-freq", type=int, default=500, help="Controller update frequency in Hz")
    parser.add_argument("--start-tilt-deg", type=float, default=5.0, help="Initial body pitch in degrees")
    parser.add_argument("--log-file", type=str, default="data/sim_log.csv", help="CSV output path")
    parser.add_argument("--realtime", action="store_true", help="Sleep to approximate real-time in GUI mode")
    parser.add_argument(
        "--legacy-gui-controls",
        action="store_true",
        help="Use PyBullet slider controls instead of the external Tk control panel.",
    )
    return parser.parse_args()


def tuning_from_values(values: dict[str, Any]) -> RuntimeTuning:
    defaults = RuntimeTuning()
    return RuntimeTuning(
        pid_kp=float(values.get("pid_kp", defaults.pid_kp)),
        pid_ki=float(values.get("pid_ki", defaults.pid_ki)),
        pid_kd=float(values.get("pid_kd", defaults.pid_kd)),
        control_polarity=float(values.get("control_polarity", defaults.control_polarity)),
        target_pitch_deg=float(values.get("target_pitch_deg", defaults.target_pitch_deg)),
        max_torque_nm=float(values.get("max_torque_nm", defaults.max_torque_nm)),
        backlash_deg=float(values.get("backlash_deg", defaults.backlash_deg)),
        backlash_impact_gain=float(values.get("backlash_impact_gain", defaults.backlash_impact_gain)),
        drive_mode=int(round(float(values.get("drive_mode", defaults.drive_mode)))),
        reducer_ratio=float(values.get("reducer_ratio", defaults.reducer_ratio)),
        belt_ratio=float(values.get("belt_ratio", defaults.belt_ratio)),
        battery_z_m=float(values.get("battery_z_m", defaults.battery_z_m)),
        battery_nominal_v=float(values.get("battery_nominal_v", defaults.battery_nominal_v)),
        battery_internal_r=float(values.get("battery_internal_r", defaults.battery_internal_r)),
        battery_max_current_a=float(values.get("battery_max_current_a", defaults.battery_max_current_a)),
        motor_kt_nm_per_a=float(values.get("motor_kt_nm_per_a", defaults.motor_kt_nm_per_a)),
        motor_ke_v_per_rad_s=float(values.get("motor_ke_v_per_rad_s", defaults.motor_ke_v_per_rad_s)),
        motor_phase_r_ohm=float(values.get("motor_phase_r_ohm", defaults.motor_phase_r_ohm)),
        disturbance_torque_nm=float(values.get("disturbance_torque_nm", defaults.disturbance_torque_nm)),
    )


def load_runtime_tuning(
    gui_enabled: bool,
    control_ids: dict[str, int] | None,
    external_values: dict[str, float] | None = None,
) -> RuntimeTuning:
    if external_values is not None:
        return tuning_from_values(external_values)
    if not gui_enabled or control_ids is None:
        return RuntimeTuning()
    values = read_gui_controls(control_ids)
    return tuning_from_values(values)


def main() -> None:
    args = parse_args()
    if args.physics_freq <= 0 or args.control_freq <= 0:
        raise ValueError("Frequencies must be positive")
    if args.physics_freq % args.control_freq != 0:
        raise ValueError("physics_freq must be divisible by control_freq to preserve exact sample-and-hold timing")

    physics_dt = 1.0 / args.physics_freq
    control_dt = 1.0 / args.control_freq
    control_interval_steps = args.physics_freq // args.control_freq
    total_steps = int(args.duration * args.physics_freq)

    connection_mode = p.DIRECT if args.headless else p.GUI
    client_id = p.connect(connection_mode)
    if client_id < 0:
        raise RuntimeError("Failed to connect to PyBullet")

    root_dir = Path(__file__).resolve().parent.parent
    urdf_path = root_dir / "urdf" / "bobot3.urdf"
    log_file = Path(args.log_file)
    if not log_file.is_absolute():
        log_file = root_dir / log_file

    control_panel = None
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
        p.resetSimulation(physicsClientId=client_id)
        p.setGravity(0.0, 0.0, -9.81, physicsClientId=client_id)
        p.setTimeStep(physics_dt, physicsClientId=client_id)
        p.setPhysicsEngineParameter(
            fixedTimeStep=physics_dt,
            numSubSteps=1,
            numSolverIterations=120,
            physicsClientId=client_id,
        )

        p.loadURDF("plane.urdf", physicsClientId=client_id)

        robot = Bobot3Robot(client_id=client_id, urdf_path=str(urdf_path), start_tilt_deg=args.start_tilt_deg)
        robot.load()

        if not args.headless:
            p.resetDebugVisualizerCamera(
                cameraDistance=1.1,
                cameraYaw=40,
                cameraPitch=-20,
                cameraTargetPosition=[0.0, 0.0, 0.2],
                physicsClientId=client_id,
            )

        gui_ids = None
        panel_poll_interval_steps = max(1, args.physics_freq // 30)
        panel_values: dict[str, float] | None = None

        if not args.headless and not args.legacy_gui_controls and TkControlPanel is not None:
            try:
                control_panel = TkControlPanel(RuntimeTuning().__dict__)
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=client_id)
                print("Using external Tk control panel for readable controls.")
            except Exception as exc:
                print(f"Tk control panel unavailable ({exc}); using PyBullet sliders.")
                gui_ids = create_gui_controls()
        elif not args.headless:
            gui_ids = create_gui_controls()

        wheel_mass_kg = 0.28
        wheel_radius_m = 0.055
        wheel_inertia = cylinder_inertia(wheel_mass_kg, wheel_radius_m)

        left_motor = SimpleMotorModel(
            electrical=MotorElectricalParams(),
            rotor_inertia_kg_m2=1.5e-4,
            wheel_inertia_kg_m2=wheel_inertia,
            gearbox_inertia_kg_m2=2.0e-4,
        )
        right_motor = SimpleMotorModel(
            electrical=MotorElectricalParams(),
            rotor_inertia_kg_m2=1.5e-4,
            wheel_inertia_kg_m2=wheel_inertia,
            gearbox_inertia_kg_m2=2.0e-4,
        )

        left_backlash = BacklashTorqueGate(deadzone_deg=1.5)
        right_backlash = BacklashTorqueGate(deadzone_deg=1.5)
        imu_model = IMUSensorModel()
        encoder = EncoderModel(bits=12)

        default_tuning = RuntimeTuning()
        controller = PIDController(
            gains=PIDGains(kp=default_tuning.pid_kp, ki=default_tuning.pid_ki, kd=default_tuning.pid_kd),
            output_limit_nm=default_tuning.max_torque_nm,
        )
        logger = SimulationLogger()

        held_torque_command_nm = 0.0
        reset_torque_hold_steps = max(1, int(0.12 * args.physics_freq))
        reset_hold_steps_remaining = 0

        for step in range(total_steps):
            sim_time_s = step * physics_dt

            if control_panel is not None and step % panel_poll_interval_steps == 0:
                control_panel.poll()
                action = control_panel.consume_action()
                if action is not None and action.get("type") == "reset_pose":
                    tilt_deg = float(action.get("tilt_deg", 0.0))
                    robot.reset_pose(tilt_deg=tilt_deg)
                    controller.reset()
                    left_motor.reset_state()
                    right_motor.reset_state()
                    left_backlash.reset_state()
                    right_backlash.reset_state()
                    held_torque_command_nm = 0.0
                    reset_hold_steps_remaining = reset_torque_hold_steps

                latest_values = control_panel.get_values()
                if latest_values:
                    panel_values = latest_values

            tuning = load_runtime_tuning(not args.headless, gui_ids, panel_values)

            controller.set_gains(tuning.pid_kp, tuning.pid_ki, tuning.pid_kd)
            controller.set_output_limit(tuning.max_torque_nm)
            robot.set_battery_position(tuning.battery_z_m)
            left_backlash.set_deadzone_deg(tuning.backlash_deg)
            right_backlash.set_deadzone_deg(tuning.backlash_deg)
            left_backlash.set_impact_gain(tuning.backlash_impact_gain)
            right_backlash.set_impact_gain(tuning.backlash_impact_gain)

            left_motor.electrical.battery_nominal_voltage_v = tuning.battery_nominal_v
            right_motor.electrical.battery_nominal_voltage_v = tuning.battery_nominal_v
            left_motor.electrical.battery_internal_resistance_ohm = tuning.battery_internal_r
            right_motor.electrical.battery_internal_resistance_ohm = tuning.battery_internal_r
            left_motor.electrical.battery_max_current_a = tuning.battery_max_current_a
            right_motor.electrical.battery_max_current_a = tuning.battery_max_current_a
            left_motor.electrical.kt_nm_per_a = tuning.motor_kt_nm_per_a
            right_motor.electrical.kt_nm_per_a = tuning.motor_kt_nm_per_a
            left_motor.electrical.ke_v_per_rad_s = tuning.motor_ke_v_per_rad_s
            right_motor.electrical.ke_v_per_rad_s = tuning.motor_ke_v_per_rad_s
            left_motor.electrical.phase_resistance_ohm = tuning.motor_phase_r_ohm
            right_motor.electrical.phase_resistance_ohm = tuning.motor_phase_r_ohm

            pitch_true_rad, pitch_rate_true_rad_s = robot.get_pitch_state()
            wheel_state = robot.get_wheel_state()
            pose = robot.get_base_pose()

            imu = imu_model.step(pitch_true_rad, pitch_rate_true_rad_s, physics_dt)
            left_encoder_tick, left_encoder_angle_q = encoder.quantize(wheel_state["left_angle_rad"])
            right_encoder_tick, right_encoder_angle_q = encoder.quantize(wheel_state["right_angle_rad"])

            if step % control_interval_steps == 0:
                if reset_hold_steps_remaining > 0:
                    held_torque_command_nm = 0.0
                else:
                    pid_output_nm = controller.update(
                        setpoint_rad=math.radians(tuning.target_pitch_deg),
                        measured_pitch_rad=imu["pitch_estimate_rad"],
                        measured_pitch_rate_rad_s=imu["gyro_pitch_rate_rad_s"],
                        dt=control_dt,
                    )
                    held_torque_command_nm = tuning.control_polarity * pid_output_nm

            applied_command_nm = held_torque_command_nm + tuning.disturbance_torque_nm
            drive_ratio = drive_ratio_from_mode(tuning.drive_mode, tuning.reducer_ratio, tuning.belt_ratio)

            left_result = left_motor.step(
                commanded_torque_nm=applied_command_nm,
                wheel_speed_rad_s=wheel_state["left_velocity_rad_s"],
                ratio=drive_ratio,
                dt=physics_dt,
            )
            right_result = right_motor.step(
                commanded_torque_nm=applied_command_nm,
                wheel_speed_rad_s=wheel_state["right_velocity_rad_s"],
                ratio=drive_ratio,
                dt=physics_dt,
            )

            left_torque_nm, left_deadzone_rad = left_backlash.step(
                left_result.output_torque_nm,
                left_result.motor_speed_rad_s,
                physics_dt,
            )
            right_torque_nm, right_deadzone_rad = right_backlash.step(
                right_result.output_torque_nm,
                right_result.motor_speed_rad_s,
                physics_dt,
            )

            robot.apply_wheel_torques(left_torque_nm, right_torque_nm)
            p.stepSimulation(physicsClientId=client_id)

            if reset_hold_steps_remaining > 0:
                reset_hold_steps_remaining -= 1

            if control_panel is not None and step % panel_poll_interval_steps == 0:
                control_panel.update_metrics(
                    {
                        "drive_mode": DRIVE_MODE_NAMES.get(tuning.drive_mode, "unknown"),
                        "drive_ratio": drive_ratio,
                        "battery_voltage_v": 0.5 * (left_result.battery_voltage_v + right_result.battery_voltage_v),
                        "left_motor_rpm": left_result.motor_rpm,
                        "right_motor_rpm": right_result.motor_rpm,
                        "left_motor_current_a": left_result.current_a,
                        "right_motor_current_a": right_result.current_a,
                        "left_output_torque_nm": left_result.output_torque_nm,
                        "right_output_torque_nm": right_result.output_torque_nm,
                        "left_torque_after_backlash_nm": left_torque_nm,
                        "right_torque_after_backlash_nm": right_torque_nm,
                    }
                )

            logger.log(
                {
                    "timestamp_s": sim_time_s,
                    "pitch_true_deg": math.degrees(pitch_true_rad),
                    "pitch_rate_true_deg_s": math.degrees(pitch_rate_true_rad_s),
                    "pitch_est_deg": math.degrees(imu["pitch_estimate_rad"]),
                    "gyro_pitch_rate_deg_s": math.degrees(imu["gyro_pitch_rate_rad_s"]),
                    "accel_x_m_s2": imu["accel_x_m_s2"],
                    "accel_z_m_s2": imu["accel_z_m_s2"],
                    "left_encoder_tick": left_encoder_tick,
                    "right_encoder_tick": right_encoder_tick,
                    "left_encoder_angle_q_rad": left_encoder_angle_q,
                    "right_encoder_angle_q_rad": right_encoder_angle_q,
                    "left_wheel_angle_rad": wheel_state["left_angle_rad"],
                    "right_wheel_angle_rad": wheel_state["right_angle_rad"],
                    "left_wheel_speed_rad_s": wheel_state["left_velocity_rad_s"],
                    "right_wheel_speed_rad_s": wheel_state["right_velocity_rad_s"],
                    "left_motor_rpm": left_result.motor_rpm,
                    "right_motor_rpm": right_result.motor_rpm,
                    "left_motor_current_a": left_result.current_a,
                    "right_motor_current_a": right_result.current_a,
                    "battery_voltage_v": 0.5 * (left_result.battery_voltage_v + right_result.battery_voltage_v),
                    "motor_kt_nm_per_a": tuning.motor_kt_nm_per_a,
                    "motor_ke_v_per_rad_s": tuning.motor_ke_v_per_rad_s,
                    "motor_phase_r_ohm": tuning.motor_phase_r_ohm,
                    "commanded_torque_nm": held_torque_command_nm,
                    "applied_command_with_disturbance_nm": applied_command_nm,
                    "left_torque_after_backlash_nm": left_torque_nm,
                    "right_torque_after_backlash_nm": right_torque_nm,
                    "left_backlash_remaining_deg": math.degrees(left_deadzone_rad),
                    "right_backlash_remaining_deg": math.degrees(right_deadzone_rad),
                    "reflected_inertia_kg_m2": 0.5
                    * (left_result.reflected_inertia_kg_m2 + right_result.reflected_inertia_kg_m2),
                    "drive_mode": DRIVE_MODE_NAMES.get(tuning.drive_mode, "unknown"),
                    "drive_ratio": drive_ratio,
                    "battery_slider_z_m": tuning.battery_z_m,
                    "base_x_m": pose["x_m"],
                    "base_y_m": pose["y_m"],
                    "base_z_m": pose["z_m"],
                    "base_roll_deg": math.degrees(pose["roll_rad"]),
                    "base_pitch_deg": math.degrees(pose["pitch_rad"]),
                    "base_yaw_deg": math.degrees(pose["yaw_rad"]),
                }
            )

            if args.realtime and not args.headless:
                time.sleep(physics_dt)

        saved = logger.save_csv(log_file)
        print(f"Simulation complete. Wrote {len(logger.rows)} samples to {saved}")

    finally:
        if control_panel is not None:
            control_panel.close()
        p.disconnect(client_id)


if __name__ == "__main__":
    main()
