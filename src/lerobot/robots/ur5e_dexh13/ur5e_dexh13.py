from __future__ import annotations

import logging
import time
from functools import cached_property
from typing import Any, Dict

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.robots.dexh13 import DexH13
from lerobot.robots.robot import Robot
from lerobot.robots.ur5e import UR5e

from .ur5e_dexh13_config import UR5eDexH13Config

logger = logging.getLogger(__name__)


class UR5eDexH13(Robot):
    """Composite robot combining a 6-DOF UR5e arm with a 13-DOF DexH13 hand."""

    config_class = UR5eDexH13Config
    name = "ur5e_dexh13"

    def __init__(self, config: UR5eDexH13Config):
        super().__init__(config)
        self.config = config
        self.arm = UR5e(config.arm_config)
        self.hand = DexH13(config.hand_config)
        self.cameras = make_cameras_from_configs(config.cameras)
        self._is_connected = False

    @cached_property
    def observation_features(self) -> Dict[str, type]:
        features = {}
        for key, value in self.arm.observation_features.items():
            if not key.startswith(("camera", "cam")):
                features[f"arm_{key}"] = value
        for key, value in self.hand.observation_features.items():
            if not key.startswith(("camera", "cam")):
                features[f"hand_{key}"] = value
        for cam_name, cam_config in self.config.cameras.items():
            features[cam_name] = (cam_config.height, cam_config.width, 3)
        return features

    @cached_property
    def action_features(self) -> Dict[str, type]:
        features = {}
        for key, value in self.arm.action_features.items():
            features[f"arm_{key}"] = value
        for key, value in self.hand.action_features.items():
            features[f"hand_{key}"] = value
        return features

    @property
    def is_connected(self) -> bool:
        return self._is_connected and self.arm.is_connected and self.hand.is_connected

    @property
    def is_calibrated(self) -> bool:
        return self.arm.is_calibrated and self.hand.is_calibrated

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info("Connecting UR5e + DexH13 composite robot...")
        try:
            self.arm.connect(calibrate=calibrate)
            self.hand.connect(calibrate=calibrate)
            for cam in self.cameras.values():
                cam.connect()
            self._is_connected = True
            logger.info("%s connected successfully", self)
        except Exception as exc:
            logger.error("Failed to connect UR5e + DexH13: %s", exc)
            try:
                if self.arm.is_connected:
                    self.arm.disconnect()
                if self.hand.is_connected:
                    self.hand.disconnect()
                for cam in self.cameras.values():
                    if cam.is_connected:
                        cam.disconnect()
            except Exception:
                logger.debug("Cleanup after failed UR5e + DexH13 connection failed", exc_info=True)
            raise ConnectionError(f"Failed to connect UR5e + DexH13: {exc}") from exc

    def configure(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        self.arm.configure()
        self.hand.configure()

    def calibrate(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        self.arm.calibrate()
        self.hand.calibrate()

    def get_observation(self) -> Dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        obs_dict = {}
        start = time.perf_counter()
        arm_obs = self.arm.get_observation()
        arm_time = time.perf_counter() - start
        for key, value in arm_obs.items():
            if not key.startswith(("camera", "cam")):
                obs_dict[f"arm_{key}"] = value

        start = time.perf_counter()
        hand_obs = self.hand.get_observation()
        hand_time = time.perf_counter() - start
        for key, value in hand_obs.items():
            if not key.startswith(("camera", "cam")):
                obs_dict[f"hand_{key}"] = value

        start = time.perf_counter()
        for cam_name, cam in self.cameras.items():
            obs_dict[cam_name] = cam.read()
        cam_time = time.perf_counter() - start

        logger.debug(
            "Arm obs: %.1fms, Hand obs: %.1fms, Cameras: %.1fms",
            arm_time * 1000,
            hand_time * 1000,
            cam_time * 1000,
        )
        return obs_dict

    def send_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        arm_action = {}
        hand_action = {}
        for key, value in action.items():
            if key.startswith("arm_"):
                arm_action[key[4:]] = value
            elif key.startswith("hand_"):
                hand_action[key[5:]] = value
            else:
                logger.warning("Unknown action key: %s (should start with 'arm_' or 'hand_')", key)

        performed_action = {}
        try:
            if arm_action:
                arm_result = self.arm.send_action(arm_action)
                for key, value in arm_result.items():
                    performed_action[f"arm_{key}"] = value
            if hand_action:
                hand_result = self.hand.send_action(hand_action)
                for key, value in hand_result.items():
                    performed_action[f"hand_{key}"] = value
        except Exception:
            if self.config.emergency_stop_both:
                self.stop()
            raise

        return performed_action

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        for component_name, component in (("UR5e arm", self.arm), ("DexH13 hand", self.hand)):
            try:
                component.disconnect()
            except Exception as exc:
                logger.error("Failed to disconnect %s: %s", component_name, exc)

        for cam in self.cameras.values():
            try:
                cam.disconnect()
            except Exception as exc:
                logger.error("Failed to disconnect camera: %s", exc)

        self._is_connected = False
        logger.info("%s disconnected", self)

    def reset_to_home(self) -> bool:
        if not self.is_connected:
            return False
        arm_success = self.arm.reset_to_home()
        hand_success = self.hand.reset_to_home()
        return arm_success and hand_success

    def stop(self) -> bool:
        if not self.is_connected:
            return False
        arm_success = self.arm.stop()
        hand_success = self.hand.stop()
        return arm_success and hand_success

    def recover_from_errors(self) -> bool:
        if not self.is_connected:
            return False
        arm_success = self.arm.recover_from_errors()
        hand_success = self.hand.recover_from_errors()
        return arm_success and hand_success
