from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict

import numpy as np


GRAVITY = 9.80665


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def cylinder_inertia(mass_kg: float, radius_m: float) -> float:
    """Return inertia around cylinder center axis: I = 0.5 * m * r^2."""
    return 0.5 * mass_kg * radius_m * radius_m


def reflected_inertia(j_rotor: float, j_wheel: float, j_gear: float, ratio: float) -> float:
    """J_total = J_rotor + (J_wheel + J_gear) / N^2."""
    ratio_safe = max(abs(ratio), 1e-6)
    return j_rotor + (j_wheel + j_gear) / (ratio_safe * ratio_safe)


DRIVE_MODE_NAMES: Dict[int, str] = {
    0: "direct",
    1: "gear_reducer",
    2: "belt_reduction",
    3: "gear_plus_belt",
}


def drive_ratio_from_mode(mode: int, reducer_ratio: float, belt_ratio: float) -> float:
    reducer_ratio = max(1.0, reducer_ratio)
    belt_ratio = max(1.0, belt_ratio)
    if mode == 0:
        return 1.0
    if mode == 1:
        return reducer_ratio
    if mode == 2:
        return belt_ratio
    return reducer_ratio * belt_ratio


@dataclass
class MotorElectricalParams:
    kt_nm_per_a: float = 0.08
    ke_v_per_rad_s: float = 0.08
    phase_resistance_ohm: float = 0.35
    battery_nominal_voltage_v: float = 18.5
    battery_voltage_limit_v: float = 21.0
    battery_internal_resistance_ohm: float = 0.12
    battery_max_current_a: float = 15.0


@dataclass
class MotorStepResult:
    output_torque_nm: float
    current_a: float
    battery_voltage_v: float
    required_voltage_v: float
    reflected_inertia_kg_m2: float
    motor_speed_rad_s: float
    motor_rpm: float


class SimpleMotorModel:
    """Simple electrical + rotational model for BLDC drive stage."""

    def __init__(
        self,
        electrical: MotorElectricalParams,
        rotor_inertia_kg_m2: float,
        wheel_inertia_kg_m2: float,
        gearbox_inertia_kg_m2: float,
    ) -> None:
        self.electrical = electrical
        self.rotor_inertia_kg_m2 = rotor_inertia_kg_m2
        self.wheel_inertia_kg_m2 = wheel_inertia_kg_m2
        self.gearbox_inertia_kg_m2 = gearbox_inertia_kg_m2
        self.rotor_speed_rad_s = 0.0

    def reset_state(self) -> None:
        self.rotor_speed_rad_s = 0.0

    def step(self, commanded_torque_nm: float, wheel_speed_rad_s: float, ratio: float, dt: float) -> MotorStepResult:
        ratio_safe = max(abs(ratio), 1.0)
        reflected = reflected_inertia(
            self.rotor_inertia_kg_m2,
            self.wheel_inertia_kg_m2,
            self.gearbox_inertia_kg_m2,
            ratio_safe,
        )

        # Keep rotor speed loosely coupled to wheel speed so backlash dead-zone can be traversed.
        target_rotor_speed = wheel_speed_rad_s * ratio_safe
        coupling_gain = 40.0
        self.rotor_speed_rad_s += coupling_gain * (target_rotor_speed - self.rotor_speed_rad_s) * dt

        sign = 0.0
        if commanded_torque_nm > 0.0:
            sign = 1.0
        elif commanded_torque_nm < 0.0:
            sign = -1.0

        if sign == 0.0:
            current_a = 0.0
            battery_voltage_v = min(
                self.electrical.battery_nominal_voltage_v,
                self.electrical.battery_voltage_limit_v,
            )
            required_voltage_v = abs(self.electrical.ke_v_per_rad_s * self.rotor_speed_rad_s)
            output_torque_nm = 0.0
        else:
            requested_current_a = abs(commanded_torque_nm) / max(self.electrical.kt_nm_per_a, 1e-9)
            bemf_v = abs(self.electrical.ke_v_per_rad_s * self.rotor_speed_rad_s)
            voltage_cap_v = min(
                self.electrical.battery_nominal_voltage_v,
                self.electrical.battery_voltage_limit_v,
            )

            max_current_from_voltage_a = max(
                (voltage_cap_v - bemf_v) / max(self.electrical.phase_resistance_ohm, 1e-9),
                0.0,
            )
            current_a = min(
                requested_current_a,
                max_current_from_voltage_a,
                self.electrical.battery_max_current_a,
            )

            battery_voltage_v = clamp(
                self.electrical.battery_nominal_voltage_v
                - current_a * self.electrical.battery_internal_resistance_ohm,
                0.0,
                self.electrical.battery_voltage_limit_v,
            )
            max_current_from_sagged_voltage_a = max(
                (battery_voltage_v - bemf_v) / max(self.electrical.phase_resistance_ohm, 1e-9),
                0.0,
            )
            current_a = min(current_a, max_current_from_sagged_voltage_a)
            output_torque_nm = sign * self.electrical.kt_nm_per_a * current_a
            required_voltage_v = bemf_v + current_a * self.electrical.phase_resistance_ohm

        alpha = 0.0
        if reflected > 1e-9:
            alpha = output_torque_nm / reflected
        self.rotor_speed_rad_s += alpha * dt
        motor_rpm = self.rotor_speed_rad_s * 60.0 / (2.0 * math.pi)

        return MotorStepResult(
            output_torque_nm=output_torque_nm,
            current_a=current_a,
            battery_voltage_v=battery_voltage_v,
            required_voltage_v=required_voltage_v,
            reflected_inertia_kg_m2=reflected,
            motor_speed_rad_s=self.rotor_speed_rad_s,
            motor_rpm=motor_rpm,
        )


class BacklashTorqueGate:
    """Threshold-based backlash model.

    When command direction reverses, torque is blocked until equivalent motor-side
    travel consumes the configured dead-zone.
    """

    def __init__(
        self,
        deadzone_deg: float = 1.5,
        min_travel_speed_rad_s: float = 0.0,
        impact_gain: float = 2.2,
        max_impact_torque_nm: float = 2.0,
    ) -> None:
        self.deadzone_rad = math.radians(max(0.0, deadzone_deg))
        self.min_travel_speed_rad_s = max(0.0, min_travel_speed_rad_s)
        self.impact_gain = max(0.0, impact_gain)
        self.max_impact_torque_nm = max(0.0, max_impact_torque_nm)
        self.remaining_deadzone_rad = 0.0
        self.last_direction = 0

    def set_deadzone_deg(self, deadzone_deg: float) -> None:
        self.deadzone_rad = math.radians(max(0.0, deadzone_deg))

    def set_impact_gain(self, impact_gain: float) -> None:
        self.impact_gain = max(0.0, impact_gain)

    def reset_state(self) -> None:
        self.remaining_deadzone_rad = 0.0
        self.last_direction = 0

    def step(self, commanded_torque_nm: float, motor_speed_rad_s: float, dt: float) -> tuple[float, float]:
        if abs(commanded_torque_nm) < 1e-10:
            return 0.0, self.remaining_deadzone_rad

        direction = 1 if commanded_torque_nm > 0.0 else -1
        if self.last_direction != 0 and direction != self.last_direction and self.remaining_deadzone_rad <= 0.0:
            self.remaining_deadzone_rad = self.deadzone_rad

        if self.remaining_deadzone_rad > 0.0:
            traversal = max(abs(motor_speed_rad_s), self.min_travel_speed_rad_s) * dt
            self.remaining_deadzone_rad = max(0.0, self.remaining_deadzone_rad - traversal)
            if self.remaining_deadzone_rad <= 0.0:
                # Engagement creates a short torque kick that produces the expected "flick"
                # when large backlash is configured.
                impact_torque = min(
                    abs(commanded_torque_nm) * self.impact_gain,
                    self.max_impact_torque_nm,
                )
                transmitted_torque = commanded_torque_nm + direction * impact_torque
            else:
                transmitted_torque = 0.0
        else:
            transmitted_torque = commanded_torque_nm

        self.last_direction = direction
        return transmitted_torque, self.remaining_deadzone_rad


class IMUSensorModel:
    def __init__(
        self,
        gyro_noise_std_rad_s: float = 0.01,
        accel_noise_std_m_s2: float = 0.05,
        complementary_alpha: float = 0.98,
        seed: int = 7,
    ) -> None:
        self.gyro_noise_std_rad_s = gyro_noise_std_rad_s
        self.accel_noise_std_m_s2 = accel_noise_std_m_s2
        self.alpha = clamp(complementary_alpha, 0.0, 1.0)
        self.rng = np.random.default_rng(seed)
        self.pitch_estimate_rad = 0.0

    def step(self, pitch_rad: float, pitch_rate_rad_s: float, dt: float) -> Dict[str, float]:
        gyro_y = pitch_rate_rad_s + float(self.rng.normal(0.0, self.gyro_noise_std_rad_s))
        accel_x = GRAVITY * math.sin(pitch_rad) + float(self.rng.normal(0.0, self.accel_noise_std_m_s2))
        accel_z = GRAVITY * math.cos(pitch_rad) + float(self.rng.normal(0.0, self.accel_noise_std_m_s2))

        pitch_from_accel = math.atan2(accel_x, accel_z)
        self.pitch_estimate_rad = self.alpha * (self.pitch_estimate_rad + gyro_y * dt) + (1.0 - self.alpha) * pitch_from_accel

        return {
            "gyro_pitch_rate_rad_s": gyro_y,
            "accel_x_m_s2": accel_x,
            "accel_z_m_s2": accel_z,
            "pitch_from_accel_rad": pitch_from_accel,
            "pitch_estimate_rad": self.pitch_estimate_rad,
        }


class EncoderModel:
    def __init__(self, bits: int = 12) -> None:
        if bits <= 1:
            raise ValueError("Encoder bits must be greater than 1")
        self.bits = bits
        self.ticks = 2**bits

    def quantize(self, angle_rad: float) -> tuple[int, float]:
        angle_norm = angle_rad % (2.0 * math.pi)
        tick = int(round((angle_norm / (2.0 * math.pi)) * (self.ticks - 1)))
        quantized = (tick / (self.ticks - 1)) * 2.0 * math.pi
        return tick, quantized
