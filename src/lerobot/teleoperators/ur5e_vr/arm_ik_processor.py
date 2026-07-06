from __future__ import annotations

import logging
import math
import time
from typing import Any, Callable, Dict, List

import numpy as np

logger = logging.getLogger(__name__)


class UR5eArmIKProcessor:
    """
    Converts VR wrist data to UR5e joint targets.

    The actual IK is delegated to a callback, normally UR5e.inverse_kinematics(),
    so real hardware uses the controller's calibrated IK.
    """

    def __init__(self, config=None):
        self.config = config or {}
        self.verbose = self.config.get("verbose", False)
        self.smoothing_factor = self.config.get("smoothing_factor", 0.4)
        self.movement_scale = self.config.get("movement_scale", 1.0)
        self.max_position_offset = self.config.get("max_position_offset", 0.65)
        self.max_position_step = self.config.get("max_position_step", 0.015)
        self.position_deadzone = self.config.get("position_deadzone", 0.001)
        self.orientation_deadzone = self.config.get("orientation_deadzone", 0.03)
        self.control_orientation = self.config.get("control_orientation", True)

        self.is_initialized = False
        self.initial_robot_pose = None
        self.initial_vr_pose = None
        self.vr_initialized = False
        self.last_target_joints = None
        self.last_target_matrix = None

    def setup(self, initial_robot_pose: List[float]) -> bool:
        try:
            # UR5e observations follow LeFranX/libfranka column-major feature
            # convention. Convert back to a standard 4x4 transform.
            self.initial_robot_pose = np.array(initial_robot_pose, dtype=float).reshape(4, 4).T
            self.is_initialized = True
            self.vr_initialized = False
            self.last_target_joints = None
            self.last_target_matrix = None
            return True
        except Exception as exc:
            logger.error("Failed to setup UR5e IK processor: %s", exc)
            return False

    def process_wrist_data(
        self,
        wrist_data,
        current_joints: List[float],
        ik_solver: Callable[[list[float], list[float]], list[float] | None],
    ) -> Dict[str, float]:
        if not self.is_initialized:
            logger.warning("UR5e arm IK processor not initialized")
            return {f"arm_joint_{i}.pos": float(current_joints[i]) for i in range(6)}

        if not getattr(wrist_data, "valid", False):
            return {f"arm_joint_{i}.pos": float(current_joints[i]) for i in range(6)}

        if not self.vr_initialized:
            try:
                self._compute_target_matrix(wrist_data)
            except Exception:
                logger.debug("Ignoring first-frame VR initialization error", exc_info=True)
            return {f"arm_joint_{i}.pos": float(current_joints[i]) for i in range(6)}

        try:
            start_time = time.perf_counter()
            target_matrix = self._compute_target_matrix(wrist_data)
            target_pose = self._matrix_to_pose6(target_matrix)
            pose_time = time.perf_counter() - start_time

            ik_start = time.perf_counter()
            target_joints = ik_solver(target_pose, current_joints)
            ik_time = time.perf_counter() - ik_start

            if target_joints is None or len(target_joints) != 6:
                if self.verbose:
                    logger.debug("UR5e IK failed; holding current joints")
                target_joints = current_joints
            else:
                target_joints = [float(value) for value in target_joints]
                if self.last_target_joints is not None:
                    target_joints = self._apply_smoothing(target_joints, self.last_target_joints)
                self.last_target_joints = target_joints

            if self.verbose:
                logger.debug(
                    "UR5e IK timing: pose=%.2fms, solve=%.2fms",
                    pose_time * 1000,
                    ik_time * 1000,
                )

            return {f"arm_joint_{i}.pos": float(target_joints[i]) for i in range(6)}
        except Exception as exc:
            logger.error("Error in UR5e arm IK processing: %s", exc)
            return {f"arm_joint_{i}.pos": float(current_joints[i]) for i in range(6)}

    def _compute_target_matrix(self, wrist_data) -> np.ndarray:
        robot_position = np.array(
            [
                wrist_data.position[2],
                -wrist_data.position[0],
                wrist_data.position[1],
            ],
            dtype=float,
        )
        robot_quaternion = self._transform_vr_quaternion_to_robot(wrist_data.quaternion)

        if not self.vr_initialized:
            self.initial_vr_pose = {
                "position": robot_position,
                "quaternion": robot_quaternion,
            }
            self.vr_initialized = True
            logger.info("UR5e VR initialized at robot-frame position: %s", robot_position)
            self.last_target_matrix = self.initial_robot_pose.copy()
            return self.last_target_matrix.copy()

        position_delta = robot_position - self.initial_vr_pose["position"]
        if np.linalg.norm(position_delta) < self.position_deadzone:
            position_delta = np.zeros(3)

        position_delta = position_delta * self.movement_scale
        offset_norm = np.linalg.norm(position_delta)
        if offset_norm > self.max_position_offset:
            position_delta = position_delta / offset_norm * self.max_position_offset

        target = self.initial_robot_pose.copy()
        target[:3, 3] = self.initial_robot_pose[:3, 3] + position_delta
        target = self._limit_position_step(target)

        if self.control_orientation:
            initial_quat = self.initial_vr_pose["quaternion"]
            current_quat = robot_quaternion
            initial_wxyz = np.array([initial_quat[3], initial_quat[0], initial_quat[1], initial_quat[2]])
            current_wxyz = np.array([current_quat[3], current_quat[0], current_quat[1], current_quat[2]])
            quat_delta = self._quaternion_multiply(current_wxyz, self._quaternion_inverse(initial_wxyz))

            delta_angle = 2 * math.acos(float(np.clip(quat_delta[0], -1.0, 1.0)))
            if abs(delta_angle) >= self.orientation_deadzone:
                initial_robot_quat = self._rotation_matrix_to_quaternion(self.initial_robot_pose[:3, :3])
                target_quat = self._quaternion_multiply(quat_delta, initial_robot_quat)
                target_quat = target_quat / np.linalg.norm(target_quat)
                target[:3, :3] = self._quaternion_to_rotation_matrix(target_quat)

        self.last_target_matrix = target.copy()
        return target

    def reset_vr_reference(self) -> None:
        self.vr_initialized = False
        self.initial_vr_pose = None
        self.last_target_joints = None
        self.last_target_matrix = None

    def get_status(self) -> Dict[str, Any]:
        return {
            "initialized": self.is_initialized,
            "vr_initialized": self.vr_initialized,
            "has_initial_pose": self.initial_robot_pose is not None,
        }

    def _apply_smoothing(self, current: List[float], previous: List[float]) -> List[float]:
        alpha = 1.0 - self.smoothing_factor
        return [alpha * c + (1 - alpha) * p for c, p in zip(current, previous)]

    def _limit_position_step(self, target: np.ndarray) -> np.ndarray:
        if self.last_target_matrix is None or self.max_position_step <= 0:
            return target

        previous_position = self.last_target_matrix[:3, 3]
        requested_step = target[:3, 3] - previous_position
        step_norm = np.linalg.norm(requested_step)
        if step_norm > self.max_position_step:
            limited = target.copy()
            limited[:3, 3] = previous_position + requested_step / step_norm * self.max_position_step
            if self.verbose:
                logger.debug(
                    "Limited UR5e VR position step from %.4fm to %.4fm",
                    step_norm,
                    self.max_position_step,
                )
            return limited

        return target

    @staticmethod
    def _matrix_to_pose6(transform: np.ndarray) -> list[float]:
        rotvec = UR5eArmIKProcessor._rotation_matrix_to_rotvec(transform[:3, :3])
        return [
            float(transform[0, 3]),
            float(transform[1, 3]),
            float(transform[2, 3]),
            float(rotvec[0]),
            float(rotvec[1]),
            float(rotvec[2]),
        ]

    @staticmethod
    def _transform_vr_quaternion_to_robot(vr_quaternion):
        qx, qy, qz, qw = vr_quaternion
        vr_matrix = np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ]
        )
        transform_matrix = np.array(
            [
                [0, 0, 1],
                [-1, 0, 0],
                [0, 1, 0],
            ]
        )
        robot_matrix = transform_matrix @ vr_matrix @ transform_matrix.T
        robot_quat_wxyz = UR5eArmIKProcessor._rotation_matrix_to_quaternion(robot_matrix)
        return np.array([robot_quat_wxyz[1], robot_quat_wxyz[2], robot_quat_wxyz[3], robot_quat_wxyz[0]])

    @staticmethod
    def _quaternion_multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ]
        )

    @staticmethod
    def _quaternion_inverse(q):
        w, x, y, z = q
        norm_sq = w * w + x * x + y * y + z * z
        return np.array([w, -x, -y, -z]) / norm_sq

    @staticmethod
    def _rotation_matrix_to_quaternion(rotation):
        trace = np.trace(rotation)
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2
            w = 0.25 * s
            x = (rotation[2, 1] - rotation[1, 2]) / s
            y = (rotation[0, 2] - rotation[2, 0]) / s
            z = (rotation[1, 0] - rotation[0, 1]) / s
        else:
            if rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
                s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
                w = (rotation[2, 1] - rotation[1, 2]) / s
                x = 0.25 * s
                y = (rotation[0, 1] + rotation[1, 0]) / s
                z = (rotation[0, 2] + rotation[2, 0]) / s
            elif rotation[1, 1] > rotation[2, 2]:
                s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
                w = (rotation[0, 2] - rotation[2, 0]) / s
                x = (rotation[0, 1] + rotation[1, 0]) / s
                y = 0.25 * s
                z = (rotation[1, 2] + rotation[2, 1]) / s
            else:
                s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
                w = (rotation[1, 0] - rotation[0, 1]) / s
                x = (rotation[0, 2] + rotation[2, 0]) / s
                y = (rotation[1, 2] + rotation[2, 1]) / s
                z = 0.25 * s
        return np.array([w, x, y, z])

    @staticmethod
    def _quaternion_to_rotation_matrix(q):
        w, x, y, z = q
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )

    @staticmethod
    def _rotation_matrix_to_rotvec(rotation: np.ndarray) -> np.ndarray:
        cos_theta = (np.trace(rotation) - 1.0) / 2.0
        cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
        theta = math.acos(cos_theta)
        if theta < 1e-12:
            return np.zeros(3)
        if abs(theta - math.pi) < 1e-5:
            axis = np.sqrt(np.maximum(np.diag(rotation) + 1.0, 0.0) / 2.0)
            axis[0] = math.copysign(axis[0], rotation[2, 1] - rotation[1, 2])
            axis[1] = math.copysign(axis[1], rotation[0, 2] - rotation[2, 0])
            axis[2] = math.copysign(axis[2], rotation[1, 0] - rotation[0, 1])
            norm = np.linalg.norm(axis)
            if norm < 1e-12:
                return np.zeros(3)
            return axis / norm * theta
        axis = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        ) / (2.0 * math.sin(theta))
        return axis * theta
