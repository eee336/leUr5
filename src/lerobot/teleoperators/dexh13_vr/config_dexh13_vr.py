from __future__ import annotations

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("dexh13_vr")
@dataclass
class DexH13VRTeleoperatorConfig(TeleoperatorConfig):
    """Configuration for DexH13 VR hand teleoperation from 21 hand landmarks."""

    vr_tcp_port: int = 8000
    setup_adb: bool = True
    vr_verbose: bool = False

    control_frequency: float = 30.0
    smoothing_alpha: float = 0.45

    # DexH13 has four fingers; this heuristic ignores human pinky landmarks.
    use_pinky_for_ring: bool = False

    # Scale raw geometric curl before applying joint limits.
    flexion_scale: float = 1.0
    abduction_scale: float = 0.7
