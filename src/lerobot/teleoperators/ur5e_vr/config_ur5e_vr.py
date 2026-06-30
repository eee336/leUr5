from __future__ import annotations

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("ur5e_vr")
@dataclass
class UR5eVRTeleoperatorConfig(TeleoperatorConfig):
    """Configuration for VR teleoperation of a UR5e arm."""

    tcp_port: int = 8000
    setup_adb: bool = True
    verbose: bool = False

    smoothing_factor: float = 0.4
    movement_scale: float = 1.0
    position_deadzone: float = 0.001
    orientation_deadzone: float = 0.03
    max_position_offset: float = 0.65
    control_orientation: bool = True
