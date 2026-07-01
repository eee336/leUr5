from __future__ import annotations

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("ur5e_dexh13_vr")
@dataclass
class UR5eDexH13VRTeleoperatorConfig(TeleoperatorConfig):
    """Configuration for dual VR teleoperation of UR5e arm + DexH13 hand."""

    vr_tcp_port: int = 8000
    setup_adb: bool = True
    vr_verbose: bool = False

    arm_smoothing_factor: float = 0.4
    arm_movement_scale: float = 1.0
    arm_max_position_offset: float = 0.65
    arm_position_deadzone: float = 0.001
    arm_orientation_deadzone: float = 0.03
    arm_control_orientation: bool = True

    hand_control_frequency: float = 30.0
    hand_smoothing_alpha: float = 0.45
    hand_mapping_backend: str = "retargeting"
    hand_retargeting_config_path: str = "dexh13_right/config/dexh13_right_dexpilot.yml"
    hand_retargeting_urdf_dir: str = "."
    hand_fallback_to_geometry: bool = True
    hand_flexion_scale: float = 1.0
    hand_abduction_scale: float = 0.7
    hand_use_pinky_for_ring: bool = False
