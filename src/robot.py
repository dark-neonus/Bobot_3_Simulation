from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict

import pybullet as p


@dataclass
class JointIndices:
    left_wheel: int
    right_wheel: int
    battery_slider: int


class Bobot3Robot:
    def __init__(self, client_id: int, urdf_path: str, start_tilt_deg: float = 5.0) -> None:
        self.client_id = client_id
        self.urdf_path = urdf_path
        self.start_tilt_deg = start_tilt_deg
        self.robot_id: int | None = None
        self.joints: JointIndices | None = None

    def load(self) -> int:
        orientation = p.getQuaternionFromEuler((0.0, math.radians(self.start_tilt_deg), 0.0))
        self.robot_id = p.loadURDF(
            self.urdf_path,
            basePosition=[0.0, 0.0, 0.055],
            baseOrientation=orientation,
            useFixedBase=False,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self.client_id,
        )

        self.joints = self._resolve_joint_indices()

        # Disable default velocity motors so torque control can drive the wheel joints.
        for joint_index in (self.joints.left_wheel, self.joints.right_wheel):
            p.setJointMotorControl2(
                self.robot_id,
                joint_index,
                p.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=0.0,
                physicsClientId=self.client_id,
            )
            p.changeDynamics(
                self.robot_id,
                joint_index,
                lateralFriction=1.2,
                rollingFriction=0.02,
                spinningFriction=0.02,
                physicsClientId=self.client_id,
            )

        self.set_battery_position(0.0)
        return self.robot_id

    def _resolve_joint_indices(self) -> JointIndices:
        if self.robot_id is None:
            raise RuntimeError("Robot must be loaded before joint mapping")

        index_by_name: Dict[str, int] = {}
        joint_count = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        for idx in range(joint_count):
            info = p.getJointInfo(self.robot_id, idx, physicsClientId=self.client_id)
            name = info[1].decode("utf-8")
            index_by_name[name] = idx

        required = ["left_wheel_joint", "right_wheel_joint", "battery_slider_joint"]
        missing = [name for name in required if name not in index_by_name]
        if missing:
            raise RuntimeError(f"Missing joints in URDF: {missing}")

        return JointIndices(
            left_wheel=index_by_name["left_wheel_joint"],
            right_wheel=index_by_name["right_wheel_joint"],
            battery_slider=index_by_name["battery_slider_joint"],
        )

    def set_battery_position(self, z_offset_m: float) -> None:
        if self.robot_id is None or self.joints is None:
            raise RuntimeError("Robot must be loaded before setting battery position")

        p.setJointMotorControl2(
            self.robot_id,
            self.joints.battery_slider,
            p.POSITION_CONTROL,
            targetPosition=z_offset_m,
            force=220.0,
            physicsClientId=self.client_id,
        )

    def apply_wheel_torques(self, left_torque_nm: float, right_torque_nm: float) -> None:
        if self.robot_id is None or self.joints is None:
            raise RuntimeError("Robot must be loaded before applying torques")

        p.setJointMotorControl2(
            self.robot_id,
            self.joints.left_wheel,
            p.TORQUE_CONTROL,
            force=left_torque_nm,
            physicsClientId=self.client_id,
        )
        p.setJointMotorControl2(
            self.robot_id,
            self.joints.right_wheel,
            p.TORQUE_CONTROL,
            force=right_torque_nm,
            physicsClientId=self.client_id,
        )

    def _lowest_link_z(self) -> float:
        if self.robot_id is None:
            raise RuntimeError("Robot must be loaded before querying AABB")

        lowest_z = float("inf")
        joint_count = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        for link_index in range(-1, joint_count):
            aabb_min, _ = p.getAABB(self.robot_id, link_index, physicsClientId=self.client_id)
            lowest_z = min(lowest_z, aabb_min[2])
        return lowest_z

    def reset_pose(self, tilt_deg: float, base_position: tuple[float, float, float] | None = None) -> None:
        if self.robot_id is None or self.joints is None:
            raise RuntimeError("Robot must be loaded before reset")

        if base_position is None:
            target_x, target_y, target_z = 0.0, 0.0, 0.055
        else:
            target_x, target_y, target_z = base_position

        orientation = p.getQuaternionFromEuler((0.0, math.radians(tilt_deg), 0.0))
        p.resetBasePositionAndOrientation(
            self.robot_id,
            posObj=[target_x, target_y, target_z],
            ornObj=orientation,
            physicsClientId=self.client_id,
        )

        # Keep the robot slightly above ground on reset so tilted chassis corners
        # do not start in penetration and generate a launch impulse.
        min_ground_clearance_m = 0.002
        lowest_z = self._lowest_link_z()
        if lowest_z < min_ground_clearance_m:
            corrected_z = target_z + (min_ground_clearance_m - lowest_z)
            p.resetBasePositionAndOrientation(
                self.robot_id,
                posObj=[target_x, target_y, corrected_z],
                ornObj=orientation,
                physicsClientId=self.client_id,
            )

        p.resetBaseVelocity(
            self.robot_id,
            linearVelocity=[0.0, 0.0, 0.0],
            angularVelocity=[0.0, 0.0, 0.0],
            physicsClientId=self.client_id,
        )

        for joint_index in (self.joints.left_wheel, self.joints.right_wheel):
            p.resetJointState(
                self.robot_id,
                joint_index,
                targetValue=0.0,
                targetVelocity=0.0,
                physicsClientId=self.client_id,
            )
            p.setJointMotorControl2(
                self.robot_id,
                joint_index,
                p.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=0.0,
                physicsClientId=self.client_id,
            )

    def get_pitch_state(self) -> tuple[float, float]:
        if self.robot_id is None:
            raise RuntimeError("Robot must be loaded before reading state")

        _, quat = p.getBasePositionAndOrientation(self.robot_id, physicsClientId=self.client_id)
        euler = p.getEulerFromQuaternion(quat)
        _, angular_velocity = p.getBaseVelocity(self.robot_id, physicsClientId=self.client_id)
        pitch_rad = euler[1]
        pitch_rate_rad_s = angular_velocity[1]
        return pitch_rad, pitch_rate_rad_s

    def get_wheel_state(self) -> Dict[str, float]:
        if self.robot_id is None or self.joints is None:
            raise RuntimeError("Robot must be loaded before reading wheel states")

        left_state = p.getJointState(self.robot_id, self.joints.left_wheel, physicsClientId=self.client_id)
        right_state = p.getJointState(self.robot_id, self.joints.right_wheel, physicsClientId=self.client_id)
        return {
            "left_angle_rad": left_state[0],
            "left_velocity_rad_s": left_state[1],
            "right_angle_rad": right_state[0],
            "right_velocity_rad_s": right_state[1],
        }

    def get_base_pose(self) -> Dict[str, float]:
        if self.robot_id is None:
            raise RuntimeError("Robot must be loaded before reading pose")

        position, orientation = p.getBasePositionAndOrientation(self.robot_id, physicsClientId=self.client_id)
        roll, pitch, yaw = p.getEulerFromQuaternion(orientation)
        return {
            "x_m": position[0],
            "y_m": position[1],
            "z_m": position[2],
            "roll_rad": roll,
            "pitch_rad": pitch,
            "yaw_rad": yaw,
        }


def create_gui_controls() -> Dict[str, int]:
    return {
        "pid_kp": p.addUserDebugParameter("PID Kp", 0.0, 250.0, 55.0),
        "pid_ki": p.addUserDebugParameter("PID Ki", 0.0, 60.0, 2.0),
        "pid_kd": p.addUserDebugParameter("PID Kd", 0.0, 30.0, 4.0),
        "control_polarity": p.addUserDebugParameter("Polarity", -1.0, 1.0, -1.0),
        "target_pitch_deg": p.addUserDebugParameter("Target Pitch [deg]", -10.0, 10.0, 0.0),
        "max_torque_nm": p.addUserDebugParameter("Max Torque [Nm]", 0.2, 12.0, 4.5),
        "backlash_deg": p.addUserDebugParameter("Backlash [deg]", 0.0, 10.0, 1.5),
        "backlash_impact_gain": p.addUserDebugParameter("Backlash Impact", 0.0, 4.0, 2.2),
        "drive_mode": p.addUserDebugParameter("Drive Mode 0-3", 0.0, 3.0, 0.0),
        "reducer_ratio": p.addUserDebugParameter("Reducer Ratio", 1.0, 10.0, 4.0),
        "belt_ratio": p.addUserDebugParameter("Belt Ratio", 1.0, 4.0, 1.5),
        "battery_z_m": p.addUserDebugParameter("Battery Z [m]", -0.20, 0.12, 0.0),
        "battery_nominal_v": p.addUserDebugParameter("Battery Nominal [V]", 15.0, 21.0, 18.5),
        "battery_internal_r": p.addUserDebugParameter("Battery Rint [ohm]", 0.01, 0.50, 0.12),
        "battery_max_current_a": p.addUserDebugParameter("Battery Imax [A]", 1.0, 30.0, 15.0),
        "motor_kt_nm_per_a": p.addUserDebugParameter("Motor Kt [Nm/A]", 0.01, 0.30, 0.08),
        "motor_ke_v_per_rad_s": p.addUserDebugParameter("Motor Ke [V/(rad/s)]", 0.01, 0.30, 0.08),
        "motor_phase_r_ohm": p.addUserDebugParameter("Motor Phase R [ohm]", 0.05, 2.0, 0.35),
        "disturbance_torque_nm": p.addUserDebugParameter("Disturbance [Nm]", -2.0, 2.0, 0.0),
    }


def read_gui_controls(control_ids: Dict[str, int]) -> Dict[str, float]:
    return {name: p.readUserDebugParameter(control_id) for name, control_id in control_ids.items()}
