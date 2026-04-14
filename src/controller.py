from __future__ import annotations

from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class PIDGains:
    kp: float = 80.0
    ki: float = 5.0
    kd: float = 2.0


class PIDController:
    """Balance PID with measured angular-rate derivative term."""

    def __init__(self, gains: PIDGains, output_limit_nm: float) -> None:
        self.gains = gains
        self.output_limit_nm = max(0.0, output_limit_nm)
        self.integral = 0.0

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        self.gains = PIDGains(kp=kp, ki=ki, kd=kd)

    def set_output_limit(self, limit_nm: float) -> None:
        self.output_limit_nm = max(0.0, limit_nm)

    def reset(self) -> None:
        self.integral = 0.0

    def update(self, setpoint_rad: float, measured_pitch_rad: float, measured_pitch_rate_rad_s: float, dt: float) -> float:
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        error = setpoint_rad - measured_pitch_rad
        self.integral += error * dt

        # Derivative on measurement avoids a large derivative kick on setpoint changes.
        derivative_term = -measured_pitch_rate_rad_s

        raw = (
            self.gains.kp * error
            + self.gains.ki * self.integral
            + self.gains.kd * derivative_term
        )

        saturated = clamp(raw, -self.output_limit_nm, self.output_limit_nm)

        # Basic anti-windup: if output is saturated and error pushes further into saturation,
        # roll back this step's integral contribution.
        if raw != saturated:
            pushing_same_direction = (raw > 0.0 and error > 0.0) or (raw < 0.0 and error < 0.0)
            if pushing_same_direction:
                self.integral -= error * dt

        return saturated
