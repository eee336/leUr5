from __future__ import annotations

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("ur5e_xhand_vr")
@dataclass
class UR5eXHandVRTeleoperatorConfig(TeleoperatorConfig):
    """Configuration for dual VR teleoperation of UR5e arm + XHand."""

    vr_tcp_port: int = 8000
    setup_adb: bool = True
    vr_verbose: bool = False

    arm_smoothing_factor: float = 0.65
    arm_movement_scale: float = 0.25
    arm_max_position_offset: float = 0.20
    arm_max_position_step: float = 0.015
    arm_position_deadzone: float = 0.005
    arm_orientation_deadzone: float = 0.03
    arm_control_orientation: bool = False

    hand_robot_name: str = "xhand_right"
    hand_retargeting_type: str = "dexpilot"
    hand_type: str = "right"
    hand_control_frequency: float = 30.0
    hand_smoothing_alpha: float = 0.3
