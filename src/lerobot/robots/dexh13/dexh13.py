from __future__ import annotations

import importlib
import logging
import time
from functools import cached_property
from typing import Any, Optional

import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .dexh13_config import DexH13Config

logger = logging.getLogger(__name__)


class DexH13(Robot):
    """PaXini/Pasini DexH13 hand with 13 active joint position commands."""

    config_class = DexH13Config
    name = "dexh13"

    def __init__(self, config: DexH13Config):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self.joint_names = list(config.joint_names)
        self._is_connected = False
        self._device = None
        self._sdk_module = None
        self._modbus_client = None
        self._stub_positions = np.array(config.home_position_rad, dtype=float)
        self._stub_currents = np.zeros(13, dtype=float)

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = {}
        for i in range(13):
            features[f"joint_{i}.pos"] = float
        for i in range(13):
            features[f"joint_{i}.current"] = float
        for cam_name, cam_config in self.config.cameras.items():
            features[cam_name] = (cam_config.height, cam_config.width, 3)
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"joint_{i}.pos": float for i in range(13)}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:  # pylint: disable=unused-argument
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        protocol = self.config.protocol.upper()
        if protocol == "STUB":
            logger.warning("Using DexH13 stub mode; no hardware commands will be sent")
        elif protocol == "SDK":
            self._connect_sdk()
        elif protocol in {"MODBUS_TCP", "MODBUS_RTU"}:
            self._connect_modbus(protocol)
        else:
            raise ValueError(f"Unsupported DexH13 protocol: {self.config.protocol}")

        for cam in self.cameras.values():
            cam.connect()

        self._is_connected = True
        logger.info("%s connected via %s", self, protocol)

    def _connect_sdk(self) -> None:
        try:
            self._sdk_module = importlib.import_module(self.config.sdk_module)
            device_class = getattr(self._sdk_module, self.config.sdk_class)
        except Exception as exc:
            raise ImportError(
                f"Could not import DexH13 SDK class {self.config.sdk_module}.{self.config.sdk_class}. "
                "Install DexHandSDK pxdex or use protocol='STUB'/'MODBUS_TCP'/'MODBUS_RTU'."
            ) from exc

        self._device = device_class(**self.config.sdk_kwargs)
        result = self._device.activeHandy(self.config.hand_port, self.config.camera_port)
        if hasattr(self._device, "isConnectHandy") and not self._device.isConnectHandy():
            raise ConnectionError(
                f"DexH13 SDK activeHandy failed for hand_port={self.config.hand_port!r}, "
                f"camera_port={self.config.camera_port!r}; return={result!r}"
            )

        if self.config.sdk_clear_faults_on_connect and getattr(self._device, "isFault", lambda: False)():
            logger.warning("DexH13 SDK reported a fault on connect; clearing fault code")
            self._device.clearFaultCode()

        if self.config.sdk_init_motor_position:
            self._device.initMotorPosition()

        if self.config.sdk_enable_on_connect:
            self._configure_sdk_position_mode()

        version_getters = ("getFirmwareVersion", "getSDKVersion")
        versions = []
        for getter_name in version_getters:
            getter = getattr(self._device, getter_name, None)
            if callable(getter):
                versions.append(f"{getter_name}={getter()}")
        if versions:
            logger.info("DexH13 SDK connected: %s", ", ".join(versions))

    def _configure_sdk_position_mode(self) -> None:
        if self._device is None or self._sdk_module is None:
            return
        control_mode = getattr(self._sdk_module, "ControlMode", None)
        if control_mode is None:
            raise AttributeError("DexH13 SDK module does not expose ControlMode")
        mode = getattr(control_mode, self.config.sdk_control_mode)

        if hasattr(self._device, "disableMotor"):
            self._device.disableMotor()
        ok = self._device.setMotorControlMode(mode)
        if ok is False:
            raise RuntimeError(f"Failed to set DexH13 SDK control mode: {self.config.sdk_control_mode}")
        self._device.enableMotor()

    def _connect_modbus(self, protocol: str) -> None:
        if self.config.position_command_register_start is None:
            raise ValueError("Set position_command_register_start from the DexH13 Modbus manual")
        if self.config.position_feedback_register_start is None:
            raise ValueError("Set position_feedback_register_start from the DexH13 Modbus manual")

        try:
            try:
                from pymodbus.client import ModbusSerialClient, ModbusTcpClient
            except ImportError:
                from pymodbus.client.sync import ModbusSerialClient, ModbusTcpClient
        except ImportError as exc:
            raise ImportError("Modbus DexH13 mode requires `pip install pymodbus`.") from exc

        if protocol == "MODBUS_TCP":
            self._modbus_client = ModbusTcpClient(
                host=self.config.modbus_host,
                port=self.config.modbus_port,
                timeout=self.config.timeout,
            )
        else:
            self._modbus_client = ModbusSerialClient(
                method="rtu",
                port=self.config.serial_port,
                baudrate=self.config.baud_rate,
                timeout=self.config.timeout,
            )

        if not self._modbus_client.connect():
            raise ConnectionError("Failed to connect to DexH13 Modbus interface")

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        try:
            self.stop()
        except Exception:
            logger.debug("DexH13 stop failed during disconnect", exc_info=True)

        if self._device is not None:
            disconnect = getattr(self._device, "disconnectHandy", None)
            if callable(disconnect):
                disconnect()
            else:
                self._call_first_available(self._device, ["disconnect", "close", "stop"], required=False)
            self._device = None
            self._sdk_module = None

        if self._modbus_client is not None:
            self._modbus_client.close()
            self._modbus_client = None

        for cam in self.cameras.values():
            cam.disconnect()

        self._is_connected = False
        logger.info("%s disconnected", self)

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        start = time.perf_counter()
        positions = self._read_positions()
        currents = self._read_currents()
        obs_dict: dict[str, Any] = {}
        for i in range(13):
            obs_dict[f"joint_{i}.pos"] = float(positions[i])
            obs_dict[f"joint_{i}.current"] = float(currents[i])

        logger.debug("%s read hand state: %.1fms", self, (time.perf_counter() - start) * 1e3)

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            logger.debug("%s read %s: %.1fms", self, cam_key, (time.perf_counter() - start) * 1e3)

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        target_positions = []
        for i in range(13):
            key = f"joint_{i}.pos"
            if key not in action:
                raise ValueError(f"Missing joint position for {key}")
            target_positions.append(action[key])

        target_positions = np.array(target_positions, dtype=float)
        target_positions = self._apply_limits(target_positions)

        if self.config.max_relative_target is not None:
            current_positions = self._read_positions()
            goal_present_pos = {
                f"joint_{i}": (target_positions[i], current_positions[i]) for i in range(13)
            }
            safe_positions = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)
            target_positions = np.array([safe_positions[f"joint_{i}"] for i in range(13)], dtype=float)

        self._write_positions(target_positions)
        return {f"joint_{i}.pos": float(target_positions[i]) for i in range(13)}

    def reset_to_home(self) -> bool:
        if not self.is_connected:
            return False
        try:
            self._write_positions(np.array(self.config.home_position_rad, dtype=float))
            return True
        except Exception as exc:
            logger.error("Failed to reset DexH13 to home: %s", exc)
            return False

    def stop(self) -> bool:
        if not self.is_connected:
            return False
        if self._device is not None:
            disable = getattr(self._device, "disableMotor", None)
            if callable(disable):
                disable()
            else:
                self._call_first_available(self._device, ["stop", "disable", "emergency_stop"], required=False)
        return True

    def recover_from_errors(self) -> bool:
        if not self.is_connected:
            return False
        if self._device is not None:
            clear_fault = getattr(self._device, "clearFaultCode", None)
            if callable(clear_fault):
                clear_fault()
            else:
                self._call_first_available(self._device, ["recover", "clear_error", "reset_error"], required=False)
        return True

    def _read_positions(self) -> np.ndarray:
        protocol = self.config.protocol.upper()
        if protocol == "STUB":
            return self._stub_positions.copy()
        if protocol == "SDK":
            if self.config.sdk_use_radian_api:
                sdk_positions = self._device.getJointPositionsRadian()
                return self._sdk16_to_active13(self._coerce_sdk16(sdk_positions, "radian positions"))
            finger_angles = self._device.getJointPositionsAngle()
            sdk_angles_deg = self._finger_angles_to_list(finger_angles)
            return self._sdk16_to_active13(np.radians(sdk_angles_deg))
        return self._read_modbus_vector(self.config.position_feedback_register_start)

    def _read_currents(self) -> np.ndarray:
        protocol = self.config.protocol.upper()
        if protocol == "STUB":
            return self._stub_currents.copy()
        if protocol == "SDK":
            data = self._call_first_available(
                self._device,
                ["read_joint_currents", "get_joint_currents", "get_currents", "get_torques", "read_torques"],
                required=False,
            )
            if data is None:
                return np.zeros(13, dtype=float)
            return self._coerce_vector(data, "currents")
        if self.config.current_feedback_register_start is None:
            return np.zeros(13, dtype=float)
        return self._read_modbus_vector(self.config.current_feedback_register_start)

    def _write_positions(self, positions: np.ndarray) -> None:
        protocol = self.config.protocol.upper()
        if protocol == "STUB":
            self._stub_positions = positions.copy()
            return
        if protocol == "SDK":
            sdk_positions = self._active13_to_sdk16(positions)
            if self.config.sdk_use_radian_api:
                self._device.setJointPositionsRadian(sdk_positions.tolist())
            else:
                finger_angles = self._list_to_finger_angles(np.degrees(sdk_positions))
                self._device.setJointPositionsAngle(finger_angles)
            return
        self._write_modbus_vector(self.config.position_command_register_start, positions)

    def _apply_limits(self, positions: np.ndarray) -> np.ndarray:
        limited = positions.copy()
        for i, (lower, upper) in enumerate(self.config.joint_limits_rad):
            limited[i] = np.clip(limited[i], lower, upper)
        return limited

    def _read_modbus_vector(self, start_register: Optional[int]) -> np.ndarray:
        if start_register is None:
            return np.zeros(13, dtype=float)
        response = self._modbus_call(
            "read_holding_registers",
            start_register,
            13,
        )
        if hasattr(response, "isError") and response.isError():
            raise RuntimeError(f"Modbus read failed at register {start_register}: {response}")
        values = [self._register_to_signed(value) / self.config.register_scale for value in response.registers[:13]]
        return np.array(values, dtype=float)

    def _write_modbus_vector(self, start_register: Optional[int], values: np.ndarray) -> None:
        if start_register is None:
            raise ValueError("DexH13 position command register start is not configured")
        registers = [self._signed_to_register(value * self.config.register_scale) for value in values]
        response = self._modbus_call("write_registers", start_register, registers)
        if hasattr(response, "isError") and response.isError():
            raise RuntimeError(f"Modbus write failed at register {start_register}: {response}")

    def _modbus_call(self, method_name: str, *args):
        method = getattr(self._modbus_client, method_name)
        try:
            return method(*args, slave=self.config.modbus_slave_id)
        except TypeError:
            return method(*args, unit=self.config.modbus_slave_id)

    @staticmethod
    def _register_to_signed(value: int) -> int:
        return value - 0x10000 if value & 0x8000 else value

    @staticmethod
    def _signed_to_register(value: float) -> int:
        integer = int(round(value))
        integer = max(min(integer, 32767), -32768)
        return integer & 0xFFFF

    @staticmethod
    def _active13_to_sdk16(values: np.ndarray) -> np.ndarray:
        vector = np.array(values, dtype=float).reshape(-1)
        if len(vector) != 13:
            raise ValueError(f"DexH13 active command must contain 13 values, got {len(vector)}")
        return np.array(
            [
                vector[0],
                vector[1],
                vector[2],
                0.0,
                vector[3],
                vector[4],
                vector[5],
                0.0,
                vector[6],
                vector[7],
                vector[8],
                0.0,
                vector[9],
                vector[10],
                vector[11],
                vector[12],
            ],
            dtype=float,
        )

    @staticmethod
    def _sdk16_to_active13(values: np.ndarray) -> np.ndarray:
        vector = np.array(values, dtype=float).reshape(-1)
        if len(vector) != 16:
            raise ValueError(f"DexH13 SDK positions must contain 16 values, got {len(vector)}")
        return np.array(
            [
                vector[0],
                vector[1],
                vector[2],
                vector[4],
                vector[5],
                vector[6],
                vector[8],
                vector[9],
                vector[10],
                vector[12],
                vector[13],
                vector[14],
                vector[15],
            ],
            dtype=float,
        )

    @staticmethod
    def _coerce_sdk16(data, label: str) -> np.ndarray:
        vector = np.array(data, dtype=float).reshape(-1)
        if len(vector) != 16:
            raise ValueError(f"DexH13 SDK {label} must contain 16 values, got {len(vector)}")
        return vector

    def _finger_angles_to_list(self, finger_angles) -> np.ndarray:
        if len(finger_angles) != 4:
            raise ValueError(f"DexH13 SDK angle feedback must contain 4 FingerAngle values, got {len(finger_angles)}")
        values = []
        for finger_angle in finger_angles:
            values.extend(
                [
                    float(finger_angle.joint1),
                    float(finger_angle.joint2),
                    float(finger_angle.joint3),
                    float(finger_angle.joint4),
                ]
            )
        return self._coerce_sdk16(values, "angle positions")

    def _list_to_finger_angles(self, values_deg: np.ndarray):
        if self._sdk_module is None:
            raise RuntimeError("DexH13 SDK module is not loaded")
        finger_angle_class = getattr(self._sdk_module, "FingerAngle")
        values = self._coerce_sdk16(values_deg, "angle command")
        fingers = []
        for start in range(0, 16, 4):
            finger = finger_angle_class()
            finger.joint1 = float(values[start])
            finger.joint2 = float(values[start + 1])
            finger.joint3 = float(values[start + 2])
            finger.joint4 = float(values[start + 3])
            fingers.append(finger)
        return fingers

    @staticmethod
    def _call_first_available(
        obj,
        method_names: list[str],
        args: tuple = (),
        required: bool = False,
    ):
        for method_name in method_names:
            method = getattr(obj, method_name, None)
            if callable(method):
                return method(*args)
        if required:
            raise AttributeError(f"None of these DexH13 SDK methods exist: {method_names}")
        return None

    @staticmethod
    def _coerce_vector(data, label: str) -> np.ndarray:
        vector = np.array(data, dtype=float).reshape(-1)
        if len(vector) != 13:
            raise ValueError(f"DexH13 {label} must contain 13 values, got {len(vector)}")
        return vector
