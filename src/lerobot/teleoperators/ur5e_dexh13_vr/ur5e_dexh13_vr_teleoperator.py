from __future__ import annotations

import logging
from typing import Any, Dict

from lerobot.teleoperators.dexh13_vr.config_dexh13_vr import DexH13VRTeleoperatorConfig
from lerobot.teleoperators.dexh13_vr.dexh13_vr_teleoperator import DexH13VRTeleoperator
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.teleoperators.ur5e_vr.config_ur5e_vr import UR5eVRTeleoperatorConfig
from lerobot.teleoperators.ur5e_vr.ur5e_vr_teleoperator import UR5eVRTeleoperator

from .config_ur5e_dexh13_vr import UR5eDexH13VRTeleoperatorConfig

logger = logging.getLogger(__name__)


class UR5eDexH13VRTeleoperator(Teleoperator):
    """Dual VR teleoperator for a UR5e + DexH13 composite robot."""

    config_class = UR5eDexH13VRTeleoperatorConfig
    name = "ur5e_dexh13_vr"

    def __init__(self, config: UR5eDexH13VRTeleoperatorConfig):
        super().__init__(config)
        self.config = config

        self.arm_teleop = UR5eVRTeleoperator(
            UR5eVRTeleoperatorConfig(
                tcp_port=config.vr_tcp_port,
                setup_adb=config.setup_adb,
                verbose=config.vr_verbose,
                smoothing_factor=config.arm_smoothing_factor,
                movement_scale=config.arm_movement_scale,
                max_position_offset=config.arm_max_position_offset,
                position_deadzone=config.arm_position_deadzone,
                orientation_deadzone=config.arm_orientation_deadzone,
                control_orientation=config.arm_control_orientation,
            )
        )
        self.hand_teleop = DexH13VRTeleoperator(
            DexH13VRTeleoperatorConfig(
                vr_tcp_port=config.vr_tcp_port,
                setup_adb=False,
                vr_verbose=config.vr_verbose,
                control_frequency=config.hand_control_frequency,
                smoothing_alpha=config.hand_smoothing_alpha,
                flexion_scale=config.hand_flexion_scale,
                abduction_scale=config.hand_abduction_scale,
                use_pinky_for_ring=config.hand_use_pinky_for_ring,
            )
        )
        self._is_connected = False
        self._robot_reference = None

    @property
    def action_features(self) -> Dict[str, type]:
        features = {}
        for key, value in self.arm_teleop.action_features.items():
            features[f"arm_{key}"] = value
        for key, value in self.hand_teleop.action_features.items():
            features[f"hand_{key}"] = value
        return features

    @property
    def feedback_features(self) -> Dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected and self.arm_teleop.is_connected and self.hand_teleop.is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise RuntimeError("UR5eDexH13VRTeleoperator is already connected")

        try:
            self.arm_teleop.connect(calibrate=calibrate)
            self.hand_teleop.connect(calibrate=calibrate)
            self._is_connected = True
            logger.info("UR5e + DexH13 dual VR teleoperator connected")
        except Exception as exc:
            logger.error("Failed to connect UR5e + DexH13 VR teleoperator: %s", exc)
            try:
                if self.hand_teleop.is_connected:
                    self.hand_teleop.disconnect()
                if self.arm_teleop.is_connected:
                    self.arm_teleop.disconnect()
            except Exception:
                logger.debug("Cleanup after failed dual VR connection failed", exc_info=True)
            raise ConnectionError(f"Failed to connect UR5e + DexH13 VR teleoperator: {exc}") from exc

    def disconnect(self) -> None:
        if not self._is_connected:
            return

        try:
            self.hand_teleop.disconnect()
        except Exception as exc:
            logger.error("Error disconnecting DexH13 VR teleoperator: %s", exc)

        try:
            self.arm_teleop.disconnect()
        except Exception as exc:
            logger.error("Error disconnecting UR5e VR teleoperator: %s", exc)

        self._is_connected = False
        self._robot_reference = None
        logger.info("UR5e + DexH13 dual VR teleoperator disconnected")

    def set_robot(self, robot):
        self._robot_reference = robot
        if hasattr(robot, "arm") and hasattr(robot, "hand"):
            self.arm_teleop.set_robot(robot.arm)
            logger.info("UR5e robot reference set for DexH13 dual VR teleoperator")
        else:
            logger.warning("Robot does not have expected 'arm' and 'hand' attributes")

    def get_action(self) -> Dict[str, Any]:
        if not self._is_connected:
            raise RuntimeError("UR5e + DexH13 dual VR teleoperator is not connected")

        try:
            action = {}
            for key, value in self.arm_teleop.get_action().items():
                action[f"arm_{key}"] = value
            for key, value in self.hand_teleop.get_action().items():
                action[f"hand_{key}"] = value
            return action
        except Exception as exc:
            logger.error("Error getting UR5e + DexH13 VR action: %s", exc)
            safe_action = {}
            for i in range(6):
                safe_action[f"arm_joint_{i}.pos"] = 0.0
            for i in range(13):
                safe_action[f"hand_joint_{i}.pos"] = 0.0
            return safe_action

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def send_feedback(self, feedback: Dict[str, Any]) -> None:  # pylint: disable=unused-argument
        pass

    def get_status(self) -> Dict[str, Any]:
        return {
            "connected": self._is_connected,
            "arm_status": self.arm_teleop.get_status() if hasattr(self.arm_teleop, "get_status") else {},
            "hand_connected": self.hand_teleop.is_connected,
        }
