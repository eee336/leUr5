from __future__ import annotations

import logging
import math
import time
from functools import cached_property
from typing import Any, Optional

import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .ur5e_config import UR5eConfig

logger = logging.getLogger(__name__)


class UR5e(Robot):
    """
    UR5e arm controlled directly through the Universal Robots RTDE Python API.

    Install the RTDE Python bindings in the LeRobot environment before connecting
    to real hardware:

        pip install ur-rtde
    """

    config_class = UR5eConfig
    name = "ur5e"

    def __init__(self, config: UR5eConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self.joint_names = [f"joint_{i}" for i in range(6)]
        self._is_connected = False
        self._rtde_c = None
        self._rtde_r = None
        self._stub_q = np.array(config.home_position, dtype=float)
        self._stub_qd = np.zeros(6, dtype=float)
        self._vr_teleoperator = None

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = {}
        for joint_name in self.joint_names:
            features[f"{joint_name}.pos"] = float
        for joint_name in self.joint_names:
            features[f"{joint_name}.vel"] = float
        for i in range(16):
            features[f"ee_pose.{i:02d}"] = float
        for cam_name, cam_config in self.config.cameras.items():
            features[cam_name] = (cam_config.height, cam_config.width, 3)
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{joint_name}.pos": float for joint_name in self.joint_names}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:  # pylint: disable=unused-argument
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        if self.config.use_stub:
            logger.warning("Using UR5e stub mode; no hardware commands will be sent")
            self._is_connected = True
        else:
            try:
                import rtde_control
                import rtde_receive
            except ImportError as exc:
                raise ImportError(
                    "UR5e requires the Universal Robots RTDE Python bindings. "
                    "Install them with `pip install ur-rtde`, or set use_stub=True."
                ) from exc

            try:
                self._rtde_c = rtde_control.RTDEControlInterface(self.config.robot_ip)
                try:
                    self._rtde_r = rtde_receive.RTDEReceiveInterface(
                        self.config.robot_ip, self.config.rtde_frequency
                    )
                except TypeError:
                    self._rtde_r = rtde_receive.RTDEReceiveInterface(self.config.robot_ip)

                if self.config.tcp_offset is not None:
                    self._rtde_c.setTcp(self.config.tcp_offset)

                self._is_connected = True
                logger.info("Connected to UR5e controller at %s", self.config.robot_ip)
            except Exception as exc:
                self._rtde_c = None
                self._rtde_r = None
                raise ConnectionError(f"Failed to connect to UR5e at {self.config.robot_ip}: {exc}") from exc

        for cam in self.cameras.values():
            cam.connect()

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        if not self.config.use_stub and self._rtde_c is not None:
            try:
                self._rtde_c.servoStop()
            except Exception:
                logger.debug("UR5e servoStop failed during disconnect", exc_info=True)
            try:
                self._rtde_c.stopScript()
            except Exception:
                logger.debug("UR5e stopScript failed during disconnect", exc_info=True)

        for interface in (self._rtde_r, self._rtde_c):
            if interface is not None and hasattr(interface, "disconnect"):
                try:
                    interface.disconnect()
                except Exception:
                    logger.debug("UR RTDE disconnect failed", exc_info=True)

        for cam in self.cameras.values():
            cam.disconnect()

        self._rtde_c = None
        self._rtde_r = None
        self._is_connected = False
        logger.info("%s disconnected", self)

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

    def register_vr_teleoperator(self, vr_teleoperator) -> None:
        self._vr_teleoperator = vr_teleoperator
        logger.info("VR teleoperator registered with UR5e for coordinated pose reset")

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        obs_dict: dict[str, Any] = {}
        start = time.perf_counter()
        positions = self._get_joint_positions()
        velocities = self._get_joint_velocities()
        tcp_pose = self._get_tcp_pose()

        for i, pos in enumerate(positions):
            obs_dict[f"joint_{i}.pos"] = float(pos)
        for i, vel in enumerate(velocities):
            obs_dict[f"joint_{i}.vel"] = float(vel)

        # LeFranX stores libfranka poses in column-major layout. Keep the same
        # feature convention so downstream code reads translation at 12:15.
        ee_pose = self._pose6_to_matrix(tcp_pose).T.flatten()
        for i, value in enumerate(ee_pose):
            obs_dict[f"ee_pose.{i:02d}"] = float(value)

        logger.debug("%s read robot state: %.1fms", self, (time.perf_counter() - start) * 1e3)

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            logger.debug("%s read %s: %.1fms", self, cam_key, (time.perf_counter() - start) * 1e3)

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        target_positions = []
        for i in range(6):
            key = f"joint_{i}.pos"
            if key not in action:
                raise ValueError(f"Missing joint position for {key}")
            target_positions.append(action[key])

        target_positions = np.array(target_positions, dtype=float)

        if self.config.max_relative_target is not None:
            current_positions = self._get_joint_positions()
            goal_present_pos = {
                f"joint_{i}": (target_positions[i], current_positions[i]) for i in range(6)
            }
            safe_positions = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)
            target_positions = np.array([safe_positions[f"joint_{i}"] for i in range(6)], dtype=float)

        if self.config.use_stub:
            self._stub_qd = (target_positions - self._stub_q) / max(self.config.servo_time, 1e-6)
            self._stub_q = target_positions
        else:
            try:
                self._rtde_c.servoJ(
                    target_positions.tolist(),
                    self.config.servo_speed,
                    self.config.servo_acceleration,
                    self.config.servo_time,
                    self.config.servo_lookahead_time,
                    self.config.servo_gain,
                )
            except Exception as exc:
                raise RuntimeError(f"Failed to send UR5e servoJ command: {exc}") from exc

        return {f"joint_{i}.pos": float(target_positions[i]) for i in range(6)}

    def reset_to_home(self) -> bool:
        if not self.is_connected:
            return False

        home_position = np.array(self.config.home_position, dtype=float)
        try:
            if self.config.use_stub:
                self._stub_qd = np.zeros(6, dtype=float)
                self._stub_q = home_position
            else:
                self._rtde_c.moveJ(
                    home_position.tolist(),
                    self.config.move_speed,
                    self.config.move_acceleration,
                )

            if self._vr_teleoperator is not None and hasattr(self._vr_teleoperator, "reset_initial_pose"):
                self._vr_teleoperator.reset_initial_pose()

            return True
        except Exception as exc:
            logger.error("Failed to reset UR5e to home: %s", exc)
            return False

    def stop(self) -> bool:
        if not self.is_connected:
            return False

        if self.config.use_stub:
            self._stub_qd = np.zeros(6, dtype=float)
            return True

        success = True
        for method_name, args in (("servoStop", ()), ("stopJ", (self.config.move_acceleration,))):
            try:
                getattr(self._rtde_c, method_name)(*args)
            except Exception as exc:
                logger.warning("UR5e %s failed: %s", method_name, exc)
                success = False
        return success

    def recover_from_errors(self) -> bool:
        if not self.is_connected:
            return False
        return self.stop()

    def inverse_kinematics(
        self,
        target_tcp_pose: list[float],
        qnear: Optional[list[float]] = None,
    ) -> Optional[list[float]]:
        """Return a 6-joint IK solution for a UR TCP pose [x, y, z, rx, ry, rz]."""
        if qnear is None:
            qnear = self._get_joint_positions().tolist()

        if self.config.use_stub or self._rtde_c is None:
            return self._fallback_inverse_kinematics(target_tcp_pose, qnear)

        try:
            try:
                q = self._rtde_c.getInverseKinematics(
                    target_tcp_pose,
                    qnear,
                    self.config.ik_position_tolerance,
                    self.config.ik_orientation_tolerance,
                )
            except TypeError:
                q = self._rtde_c.getInverseKinematics(target_tcp_pose, qnear)
        except Exception as exc:
            logger.warning("UR5e controller IK failed: %s", exc)
            return None

        if q is None or len(q) != 6:
            return None
        return [float(value) for value in q]

    def _get_joint_positions(self) -> np.ndarray:
        if self.config.use_stub or self._rtde_r is None:
            return self._stub_q.copy()
        return np.array(self._rtde_r.getActualQ(), dtype=float)

    def _get_joint_velocities(self) -> np.ndarray:
        if self.config.use_stub or self._rtde_r is None:
            return self._stub_qd.copy()
        return np.array(self._rtde_r.getActualQd(), dtype=float)

    def _get_tcp_pose(self) -> list[float]:
        if self.config.use_stub or self._rtde_r is None:
            return self._matrix_to_pose6(self._fk_ur5e(self._stub_q))
        return [float(value) for value in self._rtde_r.getActualTCPPose()]

    @staticmethod
    def _pose6_to_matrix(pose: list[float] | np.ndarray) -> np.ndarray:
        pose = np.array(pose, dtype=float)
        transform = np.eye(4)
        transform[:3, :3] = UR5e._rotvec_to_matrix(pose[3:6])
        transform[:3, 3] = pose[:3]
        return transform

    @staticmethod
    def _matrix_to_pose6(transform: np.ndarray) -> list[float]:
        return [
            float(transform[0, 3]),
            float(transform[1, 3]),
            float(transform[2, 3]),
            *[float(v) for v in UR5e._matrix_to_rotvec(transform[:3, :3])],
        ]

    @staticmethod
    def _rotvec_to_matrix(rotvec: np.ndarray) -> np.ndarray:
        theta = float(np.linalg.norm(rotvec))
        if theta < 1e-12:
            return np.eye(3)
        axis = rotvec / theta
        x, y, z = axis
        skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        return np.eye(3) + math.sin(theta) * skew + (1 - math.cos(theta)) * (skew @ skew)

    @staticmethod
    def _matrix_to_rotvec(rotation: np.ndarray) -> np.ndarray:
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

    @staticmethod
    def _dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        return np.array(
            [
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0.0, sa, ca, d],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    @staticmethod
    def _fk_ur5e(q: np.ndarray) -> np.ndarray:
        # Nominal UR5e DH parameters. Controller-side IK should be preferred on
        # real robots because it includes calibration data from the controller.
        a = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
        d = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
        alpha = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]
        transform = np.eye(4)
        for i in range(6):
            transform = transform @ UR5e._dh_transform(a[i], alpha[i], d[i], float(q[i]))
        return transform

    def _fallback_inverse_kinematics(
        self,
        target_tcp_pose: list[float],
        qnear: list[float],
    ) -> Optional[list[float]]:
        target = self._pose6_to_matrix(target_tcp_pose)
        q = np.array(qnear, dtype=float)

        for _ in range(self.config.fallback_ik_max_iterations):
            current = self._fk_ur5e(q)
            error = self._pose_error(target, current)
            if np.linalg.norm(error[:3]) < self.config.ik_position_tolerance and np.linalg.norm(error[3:]) < 5e-3:
                return [float(value) for value in q]

            jacobian = self._numeric_jacobian(q, target)
            damping = self.config.fallback_ik_damping
            lhs = jacobian @ jacobian.T + damping * np.eye(6)
            try:
                dq = -jacobian.T @ np.linalg.solve(lhs, error)
            except np.linalg.LinAlgError:
                return None

            dq = np.clip(dq, -0.08, 0.08)
            q = self._wrap_joints(q + dq)

        logger.debug("UR5e fallback IK did not converge")
        return None

    def _pose_error(self, target: np.ndarray, current: np.ndarray) -> np.ndarray:
        position_error = target[:3, 3] - current[:3, 3]
        rotation_error = self._matrix_to_rotvec(target[:3, :3] @ current[:3, :3].T)
        return np.concatenate(
            [
                self.config.fallback_ik_position_weight * position_error,
                self.config.fallback_ik_orientation_weight * rotation_error,
            ]
        )

    def _numeric_jacobian(self, q: np.ndarray, target: np.ndarray) -> np.ndarray:
        eps = 1e-5
        base_error = self._pose_error(target, self._fk_ur5e(q))
        jacobian = np.zeros((6, 6), dtype=float)
        for i in range(6):
            q_step = q.copy()
            q_step[i] += eps
            step_error = self._pose_error(target, self._fk_ur5e(q_step))
            jacobian[:, i] = (step_error - base_error) / eps
        return jacobian

    @staticmethod
    def _wrap_joints(q: np.ndarray) -> np.ndarray:
        return (q + math.pi) % (2 * math.pi) - math.pi
