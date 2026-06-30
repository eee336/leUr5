#!/usr/bin/env python

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("dexh13")
@dataclass
class DexH13Config(RobotConfig):
    """
    Configuration for a PaXini/Pasini DexH13 dexterous hand.

    The vendor SDK exposes 16 joint-angle slots grouped as four fingers:
    index[0:4], middle[4:8], ring[8:12], thumb[12:16]. DexH13 has 13 active
    motors, so the fourth slot of each non-thumb finger is unused.
    """

    # "STUB", "SDK", "MODBUS_TCP", or "MODBUS_RTU".
    protocol: str = "STUB"

    # Vendor SDK mode from DexHandSDK-v1.1.0.
    sdk_module: str = "pxdex.dh13"
    sdk_class: str = "DexH13Control"
    sdk_kwargs: Dict[str, Any] = field(default_factory=dict)
    hand_port: str = "/dev/ttyUSB0"
    camera_port: str = "none"
    sdk_control_mode: str = "POSITION_CONTROL_MODE"
    sdk_enable_on_connect: bool = True
    sdk_init_motor_position: bool = False
    sdk_clear_faults_on_connect: bool = True
    sdk_use_radian_api: bool = True

    # Modbus mode. Fill register addresses from the DexH13 vendor manual.
    modbus_host: str = "192.168.1.20"
    modbus_port: int = 502
    serial_port: str = "/dev/ttyUSB0"
    baud_rate: int = 115200
    modbus_slave_id: int = 1
    position_command_register_start: int | None = None
    position_feedback_register_start: int | None = None
    current_feedback_register_start: int | None = None
    register_scale: float = 10000.0

    # Joint model follows the SDK/EtherCAT motor order:
    # index, middle, ring, then thumb. Values are radians.
    joint_names: tuple[str, ...] = (
        "index_abduction",
        "index_mcp",
        "index_pip",
        "middle_abduction",
        "middle_mcp",
        "middle_pip",
        "ring_abduction",
        "ring_mcp",
        "ring_pip",
        "thumb_abduction",
        "thumb_mcp",
        "thumb_pip",
        "thumb_dip",
    )

    home_position_rad: tuple[float, ...] = (
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )

    # Limits from the DexH13 v1.3 manual, converted from degrees.
    joint_limits_rad: tuple[tuple[float, float], ...] = (
        (math.radians(-30.0), math.radians(30.0)),
        (0.0, math.radians(93.0)),
        (0.0, math.radians(74.5)),
        (math.radians(-30.0), math.radians(30.0)),
        (0.0, math.radians(93.0)),
        (0.0, math.radians(74.5)),
        (math.radians(-30.0), math.radians(30.0)),
        (0.0, math.radians(93.0)),
        (0.0, math.radians(74.5)),
        (math.radians(-10.0), math.radians(90.0)),
        (math.radians(-10.0), math.radians(90.0)),
        (0.0, math.radians(90.0)),
        (math.radians(-30.0), math.radians(30.0)),
    )

    max_relative_target: float | None = 0.12
    control_frequency: float = 30.0
    timeout: float = 1.0

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self):
        super().__post_init__()
        if len(self.joint_names) != 13:
            raise ValueError("DexH13Config.joint_names must contain 13 active joints")
        if len(self.home_position_rad) != 13:
            raise ValueError("DexH13Config.home_position_rad must contain 13 values")
        if len(self.joint_limits_rad) != 13:
            raise ValueError("DexH13Config.joint_limits_rad must contain 13 limits")
