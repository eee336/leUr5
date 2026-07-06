from __future__ import annotations

import logging
from typing import Any, Dict

from lerobot.teleoperators.teleoperator import Teleoperator

from .arm_ik_processor import UR5eArmIKProcessor
from .config_ur5e_vr import UR5eVRTeleoperatorConfig
from ..vr_router_manager import VRRouterConfig, get_vr_router_manager

logger = logging.getLogger(__name__)


class UR5eVRTeleoperator(Teleoperator):
    """VR teleoperator for a 6-DOF UR5e arm."""

    config_class = UR5eVRTeleoperatorConfig
    name = "ur5e_vr"

    def __init__(self, config: UR5eVRTeleoperatorConfig):
        super().__init__(config)
        self.config = config
        self.vr_manager = get_vr_router_manager()
        self.arm_ik_processor = UR5eArmIKProcessor(
            {
                "verbose": config.verbose,
                "smoothing_factor": config.smoothing_factor,
                "movement_scale": config.movement_scale,
                "max_position_offset": config.max_position_offset,
                "max_position_step": config.max_position_step,
                "position_deadzone": config.position_deadzone,
                "orientation_deadzone": config.orientation_deadzone,
                "control_orientation": config.control_orientation,
            }
        )
        self._robot_reference = None
        self._initialized = False
        self._is_connected = False
        self._last_action = None

    def connect(self, calibrate: bool = True) -> None:  # pylint: disable=unused-argument
        if self._is_connected:
            raise RuntimeError("UR5eVRTeleoperator is already connected")

        vr_config = VRRouterConfig(
            tcp_port=self.config.tcp_port,
            verbose=self.config.verbose,
            message_timeout_ms=1000.0,
            setup_adb=self.config.setup_adb,
        )

        success = self.vr_manager.register_teleoperator(vr_config, "ur5e_vr")
        if not success:
            raise ConnectionError(f"Failed to register with VR router manager on port {self.config.tcp_port}")

        if self._robot_reference and getattr(self._robot_reference, "is_connected", False):
            self._initialize_ik_solver()

        self._is_connected = True
        logger.info("UR5eVRTeleoperator connected successfully")

    def disconnect(self) -> None:
        if not self._is_connected:
            return
        self.vr_manager.unregister_teleoperator("ur5e_vr")
        self._is_connected = False
        self._initialized = False
        self._robot_reference = None
        logger.info("UR5eVRTeleoperator disconnected")

    def set_robot(self, robot):
        self._robot_reference = robot
        if hasattr(robot, "register_vr_teleoperator"):
            robot.register_vr_teleoperator(self)
        logger.info("Robot reference set for UR5e VR teleoperator: %s", robot.__class__.__name__)

    def get_action(self) -> Dict[str, Any]:
        if not self._is_connected:
            raise RuntimeError("UR5eVRTeleoperator is not connected")

        if self._robot_reference is None:
            logger.error("Robot reference not available for UR5e VR teleoperator")
            return self._last_action or {f"joint_{i}.pos": 0.0 for i in range(6)}

        if not self._initialized and not self._initialize_ik_solver():
            return self._last_action or {f"joint_{i}.pos": 0.0 for i in range(6)}

        try:
            current_obs = self._robot_reference.get_observation()
            current_joints = [current_obs[f"joint_{i}.pos"] for i in range(6)]
        except Exception as exc:
            logger.error("Failed to get UR5e observation: %s", exc)
            return self._last_action or {f"joint_{i}.pos": 0.0 for i in range(6)}

        try:
            wrist_data, status = self.vr_manager.get_wrist_data()
            if not status.get("tcp_connected", False) or wrist_data is None:
                return {f"joint_{i}.pos": float(current_joints[i]) for i in range(6)}

            arm_action = self.arm_ik_processor.process_wrist_data(
                wrist_data,
                current_joints,
                self._robot_reference.inverse_kinematics,
            )

            action = {f"joint_{i}.pos": arm_action[f"arm_joint_{i}.pos"] for i in range(6)}
            self._last_action = action
            return action
        except Exception as exc:
            logger.error("Unexpected error getting UR5e VR action: %s", exc)
            return {f"joint_{i}.pos": float(current_joints[i]) for i in range(6)}

    def _initialize_ik_solver(self) -> bool:
        if self._robot_reference is None:
            logger.error("No UR5e robot reference available for IK initialization")
            return False

        try:
            current_obs = self._robot_reference.get_observation()
            ee_pose_keys = [f"ee_pose.{i:02d}" for i in range(16)]
            if all(key in current_obs for key in ee_pose_keys):
                initial_robot_pose = [current_obs[key] for key in ee_pose_keys]
            else:
                logger.warning("No UR5e ee_pose in observation; using identity pose")
                initial_robot_pose = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]

            if not self.arm_ik_processor.setup(initial_robot_pose=initial_robot_pose):
                return False

            self._initialized = True
            logger.info("UR5e VR IK initialized successfully")
            return True
        except Exception as exc:
            logger.error("Failed to initialize UR5e VR IK: %s", exc)
            return False

    def reset_initial_pose(self) -> bool:
        if not self._is_connected:
            logger.warning("UR5e VR teleoperator is not connected; cannot reset pose")
            return False
        self.arm_ik_processor.reset_vr_reference()
        success = self._initialize_ik_solver()
        if success:
            logger.info("UR5e VR initial pose reset successfully")
        return success

    def get_status(self) -> Dict[str, Any]:
        status = {
            "connected": self._is_connected,
            "initialized": self._initialized,
            "vr_connected": False,
            "vr_ready": False,
        }
        if self._is_connected:
            try:
                vr_status = self.vr_manager.get_status()
                status.update(vr_status)
                status["vr_ready"] = vr_status.get("tcp_connected", False)
                status["ik_processor"] = self.arm_ik_processor.get_status()
            except Exception as exc:
                logger.error("Failed to get UR5e VR status: %s", exc)
        return status

    @property
    def action_features(self) -> dict:
        return {f"joint_{i}.pos": float for i in range(6)}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def send_feedback(self, feedback: Dict[str, Any]) -> None:  # pylint: disable=unused-argument
        pass
