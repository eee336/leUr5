#!/usr/bin/env python3
"""Record LeRobot episodes with UR5e + DexH13 VR teleoperation."""

from __future__ import annotations

import argparse
import logging
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from lerobot.cameras.configs import ColorMode
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.record import record_loop
from lerobot.robots.dexh13.dexh13_config import DexH13Config
from lerobot.robots.ur5e.ur5e_config import UR5eConfig
from lerobot.robots.ur5e_dexh13.ur5e_dexh13 import UR5eDexH13
from lerobot.robots.ur5e_dexh13.ur5e_dexh13_config import UR5eDexH13Config
from lerobot.teleoperators.ur5e_dexh13_vr.config_ur5e_dexh13_vr import UR5eDexH13VRTeleoperatorConfig
from lerobot.teleoperators.ur5e_dexh13_vr.ur5e_dexh13_vr_teleoperator import UR5eDexH13VRTeleoperator
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import _init_rerun

init_logging()
logger = logging.getLogger(__name__)


def parse_camera(value: str) -> tuple[str, str]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("Camera must be formatted as name=/dev/videoX")
    name, path = value.split("=", 1)
    return name, path


def parse_args():
    parser = argparse.ArgumentParser(description="Record UR5e + DexH13 VR episodes")
    parser.add_argument("--dataset-path", required=True)
    parser.add_argument("--robot-ip", default="192.168.1.10")
    parser.add_argument("--vr-port", type=int, default=8000)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--num-episodes", type=int, default=100)
    parser.add_argument("--episode-time", type=float, default=60.0)
    parser.add_argument("--task", default="Teleoperate UR5e and DexH13")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--no-adb", action="store_true")
    parser.add_argument("--ur-stub", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--camera", action="append", type=parse_camera, default=[])
    parser.add_argument("--camera-width", type=int, default=320)
    parser.add_argument("--camera-height", type=int, default=240)

    parser.add_argument("--dexh13-protocol", choices=["STUB", "SDK", "MODBUS_TCP", "MODBUS_RTU"], default="STUB")
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


def existing_episode_count(dataset_path: Path) -> int:
    if not dataset_path.exists():
        return 0
    data_dir = dataset_path / "data"
    if not data_dir.exists():
        return 0
    return len(list(data_dir.glob("**/episode_*.parquet")))


def make_cameras(args):
    return {
        name: OpenCVCameraConfig(
            index_or_path=path,
            fps=args.fps,
            width=args.camera_width,
            height=args.camera_height,
            color_mode=ColorMode.RGB,
        )
        for name, path in args.camera
    }


def make_robot(args) -> UR5eDexH13:
    return UR5eDexH13(
        UR5eDexH13Config(
            arm_config=UR5eConfig(
                robot_ip=args.robot_ip,
                rtde_frequency=max(args.fps, 30.0),
                use_stub=args.ur_stub,
                cameras={},
            ),
            hand_config=DexH13Config(
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
                control_frequency=args.fps,
                cameras={},
            ),
            cameras=make_cameras(args),
            synchronize_actions=True,
            action_timeout=0.2,
        )
    )


def make_dataset(args, robot: UR5eDexH13):
    dataset_path = Path(args.dataset_path).expanduser()
    existing = existing_episode_count(dataset_path)

    if dataset_path.exists() and args.overwrite and not args.resume:
        shutil.rmtree(dataset_path)
        existing = 0
    if args.resume and existing > 0:
        return LeRobotDataset(str(dataset_path)), existing
    if dataset_path.exists() and not args.resume and not args.overwrite:
        raise ValueError(f"Dataset path already exists: {dataset_path}. Use --resume or --overwrite.")

    dataset_features = {
        **hw_to_dataset_features(robot.action_features, "action"),
        **hw_to_dataset_features(robot.observation_features, "observation"),
    }
    return (
        LeRobotDataset.create(
            repo_id=str(dataset_path),
            fps=args.fps,
            features=dataset_features,
            robot_type=robot.name,
            use_videos=bool(robot.config.cameras),
            image_writer_threads=4,
        ),
        0,
    )


def prepare_episode(robot, teleop, episode_index: int):
    print(f"\n=== EPISODE {episode_index} PREPARATION ===")
    if teleop.is_connected:
        teleop.disconnect()
    print("Homing UR5e + DexH13...")
    robot.reset_to_home()
    time.sleep(2.0)
    print("=" * 60)
    print(f">>> Press ENTER to start recording episode {episode_index} <<<")
    print("=" * 60)
    input("Waiting for your confirmation: ")
    if not teleop.is_connected:
        teleop.connect(calibrate=False)
        teleop.set_robot(robot)
        time.sleep(2.0)
        if hasattr(teleop.arm_teleop, "reset_initial_pose"):
            teleop.arm_teleop.reset_initial_pose()


def main():
    args = parse_args()
    robot = make_robot(args)
    teleop = UR5eDexH13VRTeleoperator(
        UR5eDexH13VRTeleoperatorConfig(
            vr_tcp_port=args.vr_port,
            setup_adb=not args.no_adb,
            vr_verbose=args.verbose,
            hand_control_frequency=args.fps,
        )
    )
    dataset = None
    listener = None

    try:
        robot.connect(calibrate=False)
        teleop.set_robot(robot)
        dataset, recorded_episodes = make_dataset(args, robot)
        _init_rerun(session_name="ur5e_dexh13_vr_record")
        listener, events = init_keyboard_listener()

        log_say(f"Recording {args.num_episodes} episodes")
        while recorded_episodes < args.num_episodes and not events.get("stop_recording", False):
            episode_index = recorded_episodes + 1
            prepare_episode(robot, teleop, episode_index)
            record_loop(
                robot=robot,
                events=events,
                fps=args.fps,
                dataset=dataset,
                teleop=teleop,
                control_time_s=args.episode_time,
                single_task=args.task,
                display_data=True,
            )
            if events.get("rerecord_episode", False):
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                if teleop.is_connected:
                    teleop.disconnect()
                continue
            dataset.save_episode()
            recorded_episodes += 1
            logger.info("Episode %d saved", recorded_episodes)
            if teleop.is_connected:
                teleop.disconnect()
    finally:
        if teleop.is_connected:
            teleop.disconnect()
        if robot.is_connected:
            robot.disconnect()
        if listener is not None:
            listener.stop()
        if dataset is not None:
            logger.info("Recording finished. Dataset: %s", dataset.root)


if __name__ == "__main__":
    main()
