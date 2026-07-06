from __future__ import annotations

import logging
from typing import Any, Dict

from dex_retargeting.constants import HandType, RetargetingType, RobotName
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.teleoperators.ur5e_vr.config_ur5e_vr import UR5eVRTeleoperatorConfig
from lerobot.teleoperators.ur5e_vr.ur5e_vr_teleoperator import UR5eVRTeleoperator
from lerobot.teleoperators.xhand_vr.config_xhand_vr import XHandVRTeleoperatorConfig
from lerobot.teleoperators.xhand_vr.xhand_vr_teleoperator import XHandVRTeleoperator

from .config_ur5e_xhand_vr import UR5eXHandVRTeleoperatorConfig

logger = logging.getLogger(__name__)


class UR5eXHandVRTeleoperator(Teleoperator):
    """Dual VR teleoperator for a UR5e + XHand composite robot."""

    config_class = UR5eXHandVRTeleoperatorConfig
    name = "ur5e_xhand_vr"

    def __init__(self, config: UR5eXHandVRTeleoperatorConfig):
        super().__init__(config)
        self.config = config

        arm_config = UR5eVRTeleoperatorConfig(
            tcp_port=config.vr_tcp_port,
            setup_adb=config.setup_adb,
            verbose=config.vr_verbose,
            smoothing_factor=config.arm_smoothing_factor,
            movement_scale=config.arm_movement_scale,
            max_position_offset=config.arm_max_position_offset,
            max_position_step=config.arm_max_position_step,
            position_deadzone=config.arm_position_deadzone,
            orientation_deadzone=config.arm_orientation_deadzone,
            control_orientation=config.arm_control_orientation,
        )
        self.arm_teleop = UR5eVRTeleoperator(arm_config)

        robot_name_map = {
            "xhand_left": RobotName.xhand,
            "xhand_right": RobotName.xhand,
            "xhand": RobotName.xhand,
        }
        retargeting_map = {
            "vector": RetargetingType.vector,
            "dexpilot": RetargetingType.dexpilot,
        }
        hand_type_map = {
            "left": HandType.left,
            "right": HandType.right,
        }

        hand_config = XHandVRTeleoperatorConfig(
            robot_name=robot_name_map.get(config.hand_robot_name, RobotName.xhand),
            retargeting_type=retargeting_map.get(config.hand_retargeting_type, RetargetingType.dexpilot),
            hand_type=hand_type_map.get(config.hand_type, HandType.right),
            vr_tcp_port=config.vr_tcp_port,
            setup_adb=False,
            vr_verbose=config.vr_verbose,
            control_frequency=config.hand_control_frequency,
            smoothing_alpha=config.hand_smoothing_alpha,
        )
        self.hand_teleop = XHandVRTeleoperator(hand_config)

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
            raise RuntimeError("UR5eXHandVRTeleoperator is already connected")

        try:
            self.arm_teleop.connect(calibrate=calibrate)
            self.hand_teleop.connect(calibrate=calibrate)
            self._is_connected = True
            logger.info("UR5e + XHand dual VR teleoperator connected")
        except Exception as exc:
            logger.error("Failed to connect UR5e + XHand VR teleoperator: %s", exc)
            try:
                if self.hand_teleop.is_connected:
                    self.hand_teleop.disconnect()
                if self.arm_teleop.is_connected:
                    self.arm_teleop.disconnect()
            except Exception:
                logger.debug("Cleanup after failed dual VR connection failed", exc_info=True)
            raise ConnectionError(f"Failed to connect UR5e + XHand VR teleoperator: {exc}") from exc

    def disconnect(self) -> None:
        if not self._is_connected:
            return

        try:
            self.hand_teleop.disconnect()
        except Exception as exc:
            logger.error("Error disconnecting XHand VR teleoperator: %s", exc)

        try:
            self.arm_teleop.disconnect()
        except Exception as exc:
            logger.error("Error disconnecting UR5e VR teleoperator: %s", exc)

        self._is_connected = False
        self._robot_reference = None
        logger.info("UR5e + XHand dual VR teleoperator disconnected")

    def set_robot(self, robot):
        self._robot_reference = robot
        if hasattr(robot, "arm") and hasattr(robot, "hand"):
            self.arm_teleop.set_robot(robot.arm)
            logger.info("UR5e robot reference set for dual VR teleoperator")
        else:
            logger.warning("Robot does not have expected 'arm' and 'hand' attributes")

    def get_action(self) -> Dict[str, Any]:
        if not self._is_connected:
            raise RuntimeError("UR5e + XHand dual VR teleoperator is not connected")

        try:
            action = {}
            arm_action = self.arm_teleop.get_action()
            for key, value in arm_action.items():
                action[f"arm_{key}"] = value

            hand_action = self.hand_teleop.get_action()
            for key, value in hand_action.items():
                action[f"hand_{key}"] = value

            return action
        except Exception as exc:
            logger.error("Error getting UR5e + XHand VR action: %s", exc)
            safe_action = {}
            for i in range(6):
                safe_action[f"arm_joint_{i}.pos"] = 0.0
            for i in range(12):
                safe_action[f"hand_joint_{i}.pos"] = 0.0
            return safe_action

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def send_feedback(self, feedback: Dict[str, Any]) -> None:  # pylint: disable=unused-argument
        pass

    def get_status(self) -> Dict[str, Any]:
        status = {
            "connected": self._is_connected,
            "arm_status": {},
            "hand_status": {},
        }
        if self._is_connected:
            if hasattr(self.arm_teleop, "get_status"):
                status["arm_status"] = self.arm_teleop.get_status()
            if hasattr(self.hand_teleop, "get_status"):
                status["hand_status"] = self.hand_teleop.get_status()
        return status
