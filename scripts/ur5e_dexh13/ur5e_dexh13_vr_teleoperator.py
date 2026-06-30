#!/usr/bin/env python3
"""Run VR teleoperation for a UR5e + DexH13 composite robot."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from lerobot.robots.dexh13.dexh13_config import DexH13Config
from lerobot.robots.ur5e.ur5e_config import UR5eConfig
from lerobot.robots.ur5e_dexh13.ur5e_dexh13 import UR5eDexH13
from lerobot.robots.ur5e_dexh13.ur5e_dexh13_config import UR5eDexH13Config
from lerobot.teleoperators.ur5e_dexh13_vr.config_ur5e_dexh13_vr import UR5eDexH13VRTeleoperatorConfig
from lerobot.teleoperators.ur5e_dexh13_vr.ur5e_dexh13_vr_teleoperator import UR5eDexH13VRTeleoperator

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


def parse_args():
    parser = argparse.ArgumentParser(description="UR5e + DexH13 VR teleoperation")
    parser.add_argument("--robot-ip", default="192.168.1.10", help="UR5e controller IP")
    parser.add_argument("--vr-port", type=int, default=8000, help="Shared VR TCP port")
    parser.add_argument("--control-freq", type=float, default=30.0, help="Control frequency in Hz")
    parser.add_argument("--no-adb", action="store_true", help="Do not configure adb reverse")
    parser.add_argument("--ur-stub", action="store_true", help="Use software stub for the UR5e")
    parser.add_argument("--no-home", action="store_true", help="Skip reset_to_home before control")
    parser.add_argument("--verbose", action="store_true", help="Enable verbose VR logging")

    parser.add_argument(
        "--dexh13-protocol",
        choices=["STUB", "SDK", "MODBUS_TCP", "MODBUS_RTU"],
        default="STUB",
        help="DexH13 communication backend",
    )
    parser.add_argument("--dexh13-sdk-module", default="pxdex.dh13")
    parser.add_argument("--dexh13-sdk-class", default="DexH13Control")
    parser.add_argument("--dexh13-hand-port", default="/dev/ttyUSB0")
    parser.add_argument("--dexh13-camera-port", default="none")
    parser.add_argument("--dexh13-init-motor-position", action="store_true")
    parser.add_argument("--dexh13-no-enable", action="store_true")
    parser.add_argument("--dexh13-host", default="192.168.1.20")
    parser.add_argument("--dexh13-modbus-port", type=int, default=502)
    parser.add_argument("--dexh13-serial-port", default="/dev/ttyUSB0")
    parser.add_argument("--dexh13-baud-rate", type=int, default=115200)
    parser.add_argument("--dexh13-slave-id", type=int, default=1)
    parser.add_argument("--dexh13-command-register", type=int)
    parser.add_argument("--dexh13-feedback-register", type=int)
    parser.add_argument("--dexh13-current-register", type=int)
    parser.add_argument("--dexh13-register-scale", type=float, default=10000.0)
    return parser.parse_args()


def make_robot(args) -> UR5eDexH13:
    hand_config = DexH13Config(
        protocol=args.dexh13_protocol,
        sdk_module=args.dexh13_sdk_module,
        sdk_class=args.dexh13_sdk_class,
        hand_port=args.dexh13_hand_port,
        camera_port=args.dexh13_camera_port,
        sdk_init_motor_position=args.dexh13_init_motor_position,
        sdk_enable_on_connect=not args.dexh13_no_enable,
        modbus_host=args.dexh13_host,
        modbus_port=args.dexh13_modbus_port,
        serial_port=args.dexh13_serial_port,
        baud_rate=args.dexh13_baud_rate,
        modbus_slave_id=args.dexh13_slave_id,
        position_command_register_start=args.dexh13_command_register,
        position_feedback_register_start=args.dexh13_feedback_register,
        current_feedback_register_start=args.dexh13_current_register,
        register_scale=args.dexh13_register_scale,
        control_frequency=args.control_freq,
        cameras={},
    )
    robot_config = UR5eDexH13Config(
        arm_config=UR5eConfig(
            robot_ip=args.robot_ip,
            rtde_frequency=max(args.control_freq, 30.0),
            use_stub=args.ur_stub,
            cameras={},
        ),
        hand_config=hand_config,
        cameras={},
        synchronize_actions=True,
        action_timeout=0.2,
    )
    return UR5eDexH13(robot_config)


def main():
    args = parse_args()
    robot = make_robot(args)
    teleop = UR5eDexH13VRTeleoperator(
        UR5eDexH13VRTeleoperatorConfig(
            vr_tcp_port=args.vr_port,
            setup_adb=not args.no_adb,
            vr_verbose=args.verbose,
            hand_control_frequency=args.control_freq,
        )
    )

    try:
        logger.info("Connecting UR5e + DexH13...")
        robot.connect(calibrate=False)
        teleop.set_robot(robot)

        if not args.no_home:
            logger.info("Moving UR5e + DexH13 to home positions...")
            robot.reset_to_home()
            time.sleep(1.0)

        logger.info("Connecting shared VR teleoperator...")
        teleop.connect(calibrate=False)

        period = 1.0 / args.control_freq
        frame_count = 0
        logger.info("Move your VR source to control UR5e + DexH13. Press Ctrl+C to stop.")
        while True:
            loop_start = time.perf_counter()
            action = teleop.get_action()
            robot.send_action(action)

            frame_count += 1
            if args.verbose and frame_count % int(args.control_freq) == 0:
                logger.info("Frame %d", frame_count)

            sleep_time = period - (time.perf_counter() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        logger.info("Stopping UR5e + DexH13 VR teleoperation")
    finally:
        try:
            if teleop.is_connected:
                teleop.disconnect()
        finally:
            if robot.is_connected:
                robot.stop()
                robot.disconnect()


if __name__ == "__main__":
    main()
