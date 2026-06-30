from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict

from lerobot.cameras.utils import CameraConfig
from lerobot.robots.config import RobotConfig
from lerobot.robots.ur5e.ur5e_config import UR5eConfig
from lerobot.robots.xhand.xhand_config import XHandConfig


@RobotConfig.register_subclass("ur5e_xhand")
@dataclass
class UR5eXHandConfig(RobotConfig):
    """Configuration for a combined UR5e arm + XHand robot."""

    arm_config: UR5eConfig = field(default_factory=lambda: UR5eConfig(cameras={}))
    hand_config: XHandConfig = field(default_factory=lambda: XHandConfig(cameras={}))
    cameras: Dict[str, CameraConfig] = field(default_factory=dict)

    synchronize_actions: bool = True
    action_timeout: float = 0.1

    check_arm_hand_collision: bool = True
    emergency_stop_both: bool = True

    def __post_init__(self):
        super().__post_init__()

        if not isinstance(self.arm_config, UR5eConfig):
            raise TypeError("arm_config must be a UR5eConfig")
        if not isinstance(self.hand_config, XHandConfig):
            raise TypeError("hand_config must be a XHandConfig")

    @property
    def all_cameras(self) -> Dict[str, CameraConfig]:
        cameras = {}
        for name, config in self.arm_config.cameras.items():
            cameras[f"arm_{name}"] = config
        for name, config in self.hand_config.cameras.items():
            cameras[f"hand_{name}"] = config
        return cameras
