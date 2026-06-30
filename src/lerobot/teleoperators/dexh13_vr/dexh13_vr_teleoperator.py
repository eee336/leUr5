from __future__ import annotations

import logging
import math
from typing import Any, Dict, Optional

import numpy as np

from lerobot.teleoperators.teleoperator import Teleoperator

from .config_dexh13_vr import DexH13VRTeleoperatorConfig
from ..vr_router_manager import VRRouterConfig, get_vr_router_manager

logger = logging.getLogger(__name__)


class DexH13VRTeleoperator(Teleoperator):
    """
    Landmark-based VR teleoperator for a 13-active-DOF DexH13 hand.

    Joint order matches the DexH13 SDK/EtherCAT motor order:
    0-2 index: abduction, mcp, pip
    3-5 middle: abduction, mcp, pip
    6-8 ring: abduction, mcp, pip
    9-12 thumb: abduction, mcp, pip, dip
    """

    config_class = DexH13VRTeleoperatorConfig
    name = "dexh13_vr"

    # Mediapipe/Quest hand landmark indices.
    THUMB = (1, 2, 3, 4)
    INDEX = (5, 6, 7, 8)
    MIDDLE = (9, 10, 11, 12)
    RING = (13, 14, 15, 16)
    PINKY = (17, 18, 19, 20)

    def __init__(self, config: DexH13VRTeleoperatorConfig):
        super().__init__(config)
        self.config = config
        self.vr_manager = get_vr_router_manager()
        self._is_connected = False
        self._last_positions: Optional[np.ndarray] = None
        self._joint_limits = np.array(
            [
                [math.radians(-30.0), math.radians(30.0)],
                [0.0, math.radians(93.0)],
                [0.0, math.radians(74.5)],
                [math.radians(-30.0), math.radians(30.0)],
                [0.0, math.radians(93.0)],
                [0.0, math.radians(74.5)],
                [math.radians(-30.0), math.radians(30.0)],
                [0.0, math.radians(93.0)],
                [0.0, math.radians(74.5)],
                [math.radians(-10.0), math.radians(90.0)],
                [math.radians(-10.0), math.radians(90.0)],
                [0.0, math.radians(90.0)],
                [math.radians(-30.0), math.radians(30.0)],
            ],
            dtype=float,
        )

    @property
    def action_features(self) -> dict:
        return {f"joint_{i}.pos": float for i in range(13)}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:  # pylint: disable=unused-argument
        if self._is_connected:
            raise RuntimeError("DexH13VRTeleoperator is already connected")

        vr_config = VRRouterConfig(
            tcp_port=self.config.vr_tcp_port,
            verbose=self.config.vr_verbose,
            message_timeout_ms=1000.0,
            setup_adb=self.config.setup_adb,
        )
        success = self.vr_manager.register_teleoperator(vr_config, "dexh13_vr")
        if not success:
            raise ConnectionError(f"Failed to register with VR router manager on port {self.config.vr_tcp_port}")
        self._is_connected = True
        logger.info("DexH13VRTeleoperator connected")

    def disconnect(self) -> None:
        if not self._is_connected:
            return
        self.vr_manager.unregister_teleoperator("dexh13_vr")
        self._is_connected = False
        logger.info("DexH13VRTeleoperator disconnected")

    def get_action(self) -> Dict[str, Any]:
        if not self._is_connected:
            raise RuntimeError("DexH13VRTeleoperator is not connected")

        try:
            landmarks_data, status = self.vr_manager.get_landmarks_data()
            if not status.get("tcp_connected", False) or landmarks_data is None:
                return self._to_action(self._last_positions if self._last_positions is not None else np.zeros(13))

            joint_positions = self._landmarks_to_joint_positions(landmarks_data)
            if joint_positions is None:
                return self._to_action(self._last_positions if self._last_positions is not None else np.zeros(13))

            if self._last_positions is not None:
                alpha = self.config.smoothing_alpha
                joint_positions = alpha * joint_positions + (1.0 - alpha) * self._last_positions

            joint_positions = np.clip(joint_positions, self._joint_limits[:, 0], self._joint_limits[:, 1])
            self._last_positions = joint_positions.copy()
            return self._to_action(joint_positions)
        except Exception as exc:
            logger.warning("Unexpected error getting DexH13 VR action: %s", exc)
            return self._to_action(self._last_positions if self._last_positions is not None else np.zeros(13))

    def _landmarks_to_joint_positions(self, landmarks_data) -> Optional[np.ndarray]:
        if not hasattr(landmarks_data, "landmarks") or len(landmarks_data.landmarks) != 21:
            if self.config.vr_verbose:
                logger.warning("Expected 21 VR hand landmarks for DexH13")
            return None

        points = np.array(landmarks_data.landmarks, dtype=float)
        if points.shape != (21, 3):
            return None

        # Make wrist origin. Geometric angles are invariant to the world frame.
        points = points - points[0:1]

        ring_indices = self.PINKY if self.config.use_pinky_for_ring else self.RING

        thumb = self._thumb_joints(points)
        index = self._finger_joints(points, self.INDEX, reference=self.MIDDLE)
        middle = self._finger_joints(points, self.MIDDLE, reference=self.INDEX, neutral_abduction=0.0)
        ring = self._finger_joints(points, ring_indices, reference=self.MIDDLE)

        joints = np.array(
            [
                *index,
                *middle,
                *ring,
                *thumb,
            ],
            dtype=float,
        )
        return np.clip(joints, self._joint_limits[:, 0], self._joint_limits[:, 1])

    def _thumb_joints(self, points: np.ndarray) -> tuple[float, float, float, float]:
        cmc, mcp, ip, tip = self.THUMB
        index_mcp = self.INDEX[0]
        thumb_abduction = self._signed_angle_about_axis(
            points[index_mcp],
            points[mcp],
            self._palm_normal(points),
        )
        thumb_abduction *= self.config.abduction_scale

        thumb_mcp = self._curl_angle(points[cmc], points[mcp], points[ip])
        thumb_pip = self._curl_angle(points[mcp], points[ip], points[tip])
        thumb_dip = 0.65 * thumb_pip
        return (
            thumb_abduction,
            thumb_mcp * self.config.flexion_scale,
            thumb_pip * self.config.flexion_scale,
            thumb_dip * self.config.flexion_scale,
        )

    def _finger_joints(
        self,
        points: np.ndarray,
        indices: tuple[int, int, int, int],
        reference: tuple[int, int, int, int],
        neutral_abduction: float | None = None,
    ) -> tuple[float, float, float]:
        mcp, pip, dip, tip = indices
        ref_mcp = reference[0]
        palm_normal = self._palm_normal(points)
        abduction = self._signed_angle_about_axis(points[ref_mcp], points[mcp], palm_normal)
        if neutral_abduction is not None:
            abduction = neutral_abduction
        abduction *= self.config.abduction_scale

        mcp_flex = self._curl_angle(points[0], points[mcp], points[pip])
        pip_flex = self._curl_angle(points[mcp], points[pip], points[tip])
        return (
            abduction,
            mcp_flex * self.config.flexion_scale,
            pip_flex * self.config.flexion_scale,
        )

    @staticmethod
    def _curl_angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
        v1 = a - b
        v2 = c - b
        denom = np.linalg.norm(v1) * np.linalg.norm(v2)
        if denom < 1e-9:
            return 0.0
        angle = math.acos(float(np.clip(np.dot(v1, v2) / denom, -1.0, 1.0)))
        return max(0.0, math.pi - angle)

    @staticmethod
    def _palm_normal(points: np.ndarray) -> np.ndarray:
        index = points[5] - points[0]
        ring = points[13] - points[0]
        normal = np.cross(index, ring)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            return np.array([0.0, 0.0, 1.0])
        return normal / norm

    @staticmethod
    def _signed_angle_about_axis(v1: np.ndarray, v2: np.ndarray, axis: np.ndarray) -> float:
        v1_norm = np.linalg.norm(v1)
        v2_norm = np.linalg.norm(v2)
        if v1_norm < 1e-9 or v2_norm < 1e-9:
            return 0.0
        a = v1 / v1_norm
        b = v2 / v2_norm
        unsigned = math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0)))
        sign = math.copysign(1.0, float(np.dot(axis, np.cross(a, b))))
        return sign * unsigned

    @staticmethod
    def _to_action(joint_positions: np.ndarray) -> Dict[str, Any]:
        return {f"joint_{i}.pos": float(joint_positions[i]) for i in range(13)}

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def send_feedback(self, feedback: Dict[str, Any]) -> None:  # pylint: disable=unused-argument
        pass
