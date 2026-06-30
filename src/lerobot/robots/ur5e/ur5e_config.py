#!/usr/bin/env python

from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("ur5e")
@dataclass
class UR5eConfig(RobotConfig):
    """Configuration for a UR5e arm controlled through Universal Robots RTDE."""

    # UR controller connection.
    robot_ip: str = "192.168.1.10"
    rtde_frequency: float = 125.0
    use_stub: bool = False

    # Six UR joint positions, in radians.
    home_position: list[float] = field(
        default_factory=lambda: [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
    )

    # Caps each joint command relative to the current joint position.
    max_relative_target: float | None = 0.08

    # RTDE servoJ parameters for streaming position targets.
    servo_speed: float = 0.5
    servo_acceleration: float = 1.0
    servo_time: float = 0.008
    servo_lookahead_time: float = 0.1
    servo_gain: int = 300

    # moveJ parameters used for reset_to_home.
    move_speed: float = 0.25
    move_acceleration: float = 0.5

    # Optional TCP offset as [x, y, z, rx, ry, rz], in meters and radians.
    tcp_offset: list[float] | None = None

    # Controller-side IK tolerances used by rtde_control.getInverseKinematics.
    ik_position_tolerance: float = 1e-4
    ik_orientation_tolerance: float = 1e-3

    # Offline/stub numerical IK fallback settings.
    fallback_ik_max_iterations: int = 120
    fallback_ik_damping: float = 1e-3
    fallback_ik_position_weight: float = 1.0
    fallback_ik_orientation_weight: float = 0.4

    cameras: dict[str, CameraConfig] = field(default_factory=dict)
