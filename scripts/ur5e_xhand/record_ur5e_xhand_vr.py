#!/usr/bin/env python3
"""Record LeRobot episodes with UR5e + XHand VR teleoperation."""

import argparse
import logging
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "vr-dex-retargeting" / "src"))

from lerobot.cameras.configs import ColorMode
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.record import record_loop
from lerobot.robots.ur5e.ur5e_config import UR5eConfig
from lerobot.robots.ur5e_xhand.ur5e_xhand import UR5eXHand
from lerobot.robots.ur5e_xhand.ur5e_xhand_config import UR5eXHandConfig
from lerobot.robots.xhand.xhand_config import XHandConfig
from lerobot.teleoperators.ur5e_xhand_vr.config_ur5e_xhand_vr import UR5eXHandVRTeleoperatorConfig
from lerobot.teleoperators.ur5e_xhand_vr.ur5e_xhand_vr_teleoperator import UR5eXHandVRTeleoperator
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import _init_rerun

init_logging()
logger = logging.getLogger(__name__)


def parse_camera(value: str) -> tuple[str, str]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("Camera must be formatted as name=/dev/videoX")
    name, path = value.split("=", 1)
    if not name or not path:
        raise argparse.ArgumentTypeError("Camera name and path must be non-empty")
    return name, path


def parse_args():
    parser = argparse.ArgumentParser(description="Record UR5e + XHand VR episodes")
    parser.add_argument("--dataset-path", required=True, help="Local dataset path or repo id")
    parser.add_argument("--robot-ip", default="192.168.1.10", help="UR5e controller IP")
    parser.add_argument("--xhand-port", default="/dev/ttyUSB0", help="XHand RS485 serial port")
    parser.add_argument("--vr-port", type=int, default=8000, help="Shared VR TCP port")
    parser.add_argument("--fps", type=int, default=30, help="Recording FPS")
    parser.add_argument("--num-episodes", type=int, default=100, help="Total episodes to record")
    parser.add_argument("--episode-time", type=float, default=60.0, help="Episode duration in seconds")
    parser.add_argument("--task", default="Teleoperate UR5e and XHand", help="Task description")
    parser.add_argument("--resume", action="store_true", help="Resume an existing dataset")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite dataset if it exists")
    parser.add_argument("--no-adb", action="store_true", help="Do not configure adb reverse")
    parser.add_argument("--ur-stub", action="store_true", help="Use software stub for the UR5e")
    parser.add_argument(
        "--camera",
        action="append",
        type=parse_camera,
        default=[],
        help="Add OpenCV camera as name=/dev/videoX. Can be passed multiple times.",
    )
    parser.add_argument("--camera-width", type=int, default=320)
    parser.add_argument("--camera-height", type=int, default=240)
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def existing_episode_count(dataset_path: Path) -> int:
    if not dataset_path.exists():
        return 0
    data_dir = dataset_path / "data"
    if not data_dir.exists():
        return 0
    return len(list(data_dir.glob("**/episode_*.parquet")))


def make_cameras(args) -> dict[str, OpenCVCameraConfig]:
    cameras = {}
    for name, path in args.camera:
        cameras[name] = OpenCVCameraConfig(
            index_or_path=path,
            fps=args.fps,
            width=args.camera_width,
            height=args.camera_height,
            color_mode=ColorMode.RGB,
        )
    return cameras


def make_robot(args) -> UR5eXHand:
    cameras = make_cameras(args)
    robot_config = UR5eXHandConfig(
        arm_config=UR5eConfig(
            robot_ip=args.robot_ip,
            rtde_frequency=max(args.fps, 30.0),
            use_stub=args.ur_stub,
            cameras={},
        ),
        hand_config=XHandConfig(
            protocol="RS485",
            serial_port=args.xhand_port,
            baud_rate=3000000,
            hand_id=0,
            control_frequency=args.fps,
            cameras={},
        ),
        cameras=cameras,
        synchronize_actions=True,
        action_timeout=0.2,
    )
    return UR5eXHand(robot_config)


def make_teleop(args) -> UR5eXHandVRTeleoperator:
    teleop_config = UR5eXHandVRTeleoperatorConfig(
        vr_tcp_port=args.vr_port,
        setup_adb=not args.no_adb,
        vr_verbose=args.verbose,
        arm_smoothing_factor=0.4,
        arm_movement_scale=1.0,
        arm_max_position_offset=0.65,
        hand_robot_name="xhand_right",
        hand_retargeting_type="dexpilot",
        hand_type="right",
        hand_control_frequency=args.fps,
        hand_smoothing_alpha=0.5,
    )
    return UR5eXHandVRTeleoperator(teleop_config)


def make_dataset(args, robot: UR5eXHand) -> tuple[LeRobotDataset, int]:
    dataset_path = Path(args.dataset_path).expanduser()
    existing = existing_episode_count(dataset_path)

    if dataset_path.exists() and args.overwrite and not args.resume:
        logger.warning("Removing existing dataset directory: %s", dataset_path)
        shutil.rmtree(dataset_path)
        existing = 0

    if args.resume and existing > 0:
        logger.info("Resuming dataset at %s with %d existing episodes", dataset_path, existing)
        return LeRobotDataset(str(dataset_path)), existing

    if dataset_path.exists() and not args.resume and not args.overwrite:
        raise ValueError(f"Dataset path already exists: {dataset_path}. Use --resume or --overwrite.")

    action_features = hw_to_dataset_features(robot.action_features, "action")
    obs_features = hw_to_dataset_features(robot.observation_features, "observation")
    dataset_features = {**action_features, **obs_features}

    dataset = LeRobotDataset.create(
        repo_id=str(dataset_path),
        fps=args.fps,
        features=dataset_features,
        robot_type=robot.name,
        use_videos=bool(robot.config.cameras),
        image_writer_threads=4,
    )
    return dataset, 0


def prepare_episode(robot: UR5eXHand, teleop: UR5eXHandVRTeleoperator, episode_index: int) -> None:
    print(f"\n=== EPISODE {episode_index} PREPARATION ===")
    if teleop.is_connected:
        teleop.disconnect()

    print("Homing UR5e + XHand...")
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
    teleop = make_teleop(args)
    dataset = None
    listener = None

    try:
        robot.connect(calibrate=False)
        teleop.set_robot(robot)

        dataset, recorded_episodes = make_dataset(args, robot)
        _init_rerun(session_name="ur5e_xhand_vr_record")
        listener, events = init_keyboard_listener()

        log_say(f"Recording {args.num_episodes} episodes")
        log_say("Press 's' to stop recording, 'r' to re-record current episode")

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

        logger.info("Dataset saved at: %s", dataset.root)
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
