#!/usr/bin/env python3
"""Run VR teleoperation for a UR5e + XHand composite robot."""

import argparse
import logging
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "vr-dex-retargeting" / "src"))

from lerobot.robots.ur5e.ur5e_config import UR5eConfig
from lerobot.robots.ur5e_xhand.ur5e_xhand import UR5eXHand
from lerobot.robots.ur5e_xhand.ur5e_xhand_config import UR5eXHandConfig
from lerobot.robots.xhand.xhand_config import XHandConfig
from lerobot.teleoperators.ur5e_xhand_vr.config_ur5e_xhand_vr import UR5eXHandVRTeleoperatorConfig
from lerobot.teleoperators.ur5e_xhand_vr.ur5e_xhand_vr_teleoperator import UR5eXHandVRTeleoperator


logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


def parse_args():
    parser = argparse.ArgumentParser(description="UR5e + XHand VR teleoperation")
    parser.add_argument("--robot-ip", default="192.168.1.10", help="UR5e controller IP")
    parser.add_argument("--xhand-port", default="/dev/ttyUSB0", help="XHand RS485 serial port")
    parser.add_argument("--vr-port", type=int, default=8000, help="Shared VR TCP port")
    parser.add_argument("--control-freq", type=float, default=30.0, help="Control frequency in Hz")
    parser.add_argument("--no-adb", action="store_true", help="Do not configure adb reverse")
    parser.add_argument("--ur-stub", action="store_true", help="Use software stub for the UR5e")
    parser.add_argument("--no-home", action="store_true", help="Skip reset_to_home before control")
    parser.add_argument("--verbose", action="store_true", help="Enable verbose VR/IK logging")
    parser.add_argument("--arm-movement-scale", type=float, default=0.25, help="Scale VR wrist displacement before sending it to UR5e")
    parser.add_argument("--arm-max-position-offset", type=float, default=0.20, help="Maximum UR5e EE offset from the reset pose in meters")
    parser.add_argument("--arm-max-position-step", type=float, default=0.015, help="Maximum UR5e EE target position change per control cycle in meters")
    parser.add_argument("--arm-position-deadzone", type=float, default=0.005, help="Ignore VR wrist displacement below this value in meters")
    parser.add_argument("--arm-smoothing-factor", type=float, default=0.65, help="Joint target smoothing factor from 0 to 1")
    parser.add_argument("--arm-control-orientation", action="store_true", help="Also control UR5e EE orientation from the VR wrist quaternion")
    return parser.parse_args()


def main():
    args = parse_args()

    robot_config = UR5eXHandConfig(
        arm_config=UR5eConfig(
            robot_ip=args.robot_ip,
            rtde_frequency=max(args.control_freq, 30.0),
            use_stub=args.ur_stub,
            cameras={},
        ),
        hand_config=XHandConfig(
            protocol="RS485",
            serial_port=args.xhand_port,
            baud_rate=3000000,
            hand_id=0,
            control_frequency=args.control_freq,
            cameras={},
        ),
        cameras={},
        synchronize_actions=True,
        action_timeout=0.2,
    )
    robot = UR5eXHand(robot_config)

    teleop_config = UR5eXHandVRTeleoperatorConfig(
        vr_tcp_port=args.vr_port,
        setup_adb=not args.no_adb,
        vr_verbose=args.verbose,
        arm_smoothing_factor=args.arm_smoothing_factor,
        arm_movement_scale=args.arm_movement_scale,
        arm_max_position_offset=args.arm_max_position_offset,
        arm_max_position_step=args.arm_max_position_step,
        arm_position_deadzone=args.arm_position_deadzone,
        arm_control_orientation=args.arm_control_orientation,
        hand_robot_name="xhand_right",
        hand_retargeting_type="dexpilot",
        hand_type="right",
        hand_control_frequency=args.control_freq,
        hand_smoothing_alpha=0.5,
    )
    teleop = UR5eXHandVRTeleoperator(teleop_config)

    try:
        logger.info("Connecting UR5e + XHand...")
        robot.connect(calibrate=False)
        teleop.set_robot(robot)

        if not args.no_home:
            logger.info("Moving UR5e + XHand to home positions...")
            robot.reset_to_home()
            time.sleep(1.0)

        logger.info("Connecting shared VR teleoperator...")
        teleop.connect(calibrate=False)

        period = 1.0 / args.control_freq
        frame_count = 0
        logger.info("Move your VR source to control UR5e + XHand. Press Ctrl+C to stop.")
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
        logger.info("Stopping UR5e + XHand VR teleoperation")
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
