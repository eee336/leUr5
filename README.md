# LeFranX
[🤗HF Dataset](https://huggingface.co/collections/wengmister/lefranx-dataset-68b4684269dbb97fd3061be0) | [arXiv](https://arxiv.org/abs/2509.14349) | [XHand Coupling CAD](https://github.com/wengmister/xhand-coupling)

LeRobot Extension for Franka FER Robot & XHand Robot. An instantiation of the [LeVR](https://arxiv.org/abs/2509.14349) framework.

[![Watch the video](https://img.youtube.com/vi/TzlUEWCjQ1M/0.jpg)](https://www.youtube.com/watch?v=TzlUEWCjQ1M)

## Architecture

There are three main parts to this LeRobot Franka robot extension, `franka_server`, `franka_xhand_teleoperator`, and added class implementation under `src/lerobot`. Check out system flowchart below:


<img width="1891" height="1649" alt="flow-chart" src="https://github.com/user-attachments/assets/cfd8389a-2ecf-4e1c-8f6f-ca1aa0905fbf" />

## UR5e adaptation

This fork also includes a UR5e path alongside the original Franka implementation:

- `src/lerobot/robots/ur5e`: UR5e arm controlled through Universal Robots RTDE.
- `src/lerobot/robots/ur5e_xhand`: composite UR5e + XHand robot.
- `src/lerobot/robots/dexh13`: PaXini/Pasini DexH13 hand with 13 active joints.
- `src/lerobot/robots/ur5e_dexh13`: composite UR5e + DexH13 robot.
- `src/lerobot/teleoperators/ur5e_vr`: VR wrist teleoperation for UR5e.
- `src/lerobot/teleoperators/ur5e_xhand_vr`: shared VR teleoperation for UR5e + XHand.
- `src/lerobot/teleoperators/dexh13_vr`: landmark-based VR teleoperation for DexH13.
- `src/lerobot/teleoperators/ur5e_dexh13_vr`: shared VR teleoperation for UR5e + DexH13.
- `scripts/ur5e` and `scripts/ur5e_xhand`: example control scripts.
- `scripts/ur5e_dexh13`: UR5e + DexH13 teleoperation and recording scripts.

Unlike the Franka path, UR5e does not use `franka_server`. Install the UR RTDE Python bindings in the same environment as LeRobot:

```bash
[uv] pip install ur-rtde
```

For UR5e + XHand VR teleoperation, still build the VR message router extension:

```bash
cd franka_xhand_teleoperator
[uv] pip install -e .
```

Then run one of the UR5e examples:

```bash
python scripts/ur5e/ur5e_vr_teleoperator.py --robot-ip YOUR_UR5E_IP
python scripts/ur5e_xhand/ur5e_xhand_vr_teleoperator.py --robot-ip YOUR_UR5E_IP --xhand-port /dev/ttyUSB0
python scripts/ur5e_xhand/record_ur5e_xhand_vr.py --dataset-path ./datasets/ur5e_xhand_demo --robot-ip YOUR_UR5E_IP --xhand-port /dev/ttyUSB0
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py --robot-ip YOUR_UR5E_IP --dexh13-protocol SDK --dexh13-hand-port /dev/ttyUSB0
python scripts/ur5e_dexh13/record_ur5e_dexh13_vr.py --dataset-path ./datasets/ur5e_dexh13_demo --robot-ip YOUR_UR5E_IP --dexh13-protocol SDK --dexh13-hand-port /dev/ttyUSB0
```

The new LeRobot config types are `ur5e`, `dexh13`, `ur5e_xhand`, `ur5e_dexh13`, `ur5e_vr`, `dexh13_vr`, `ur5e_xhand_vr`, and `ur5e_dexh13_vr`.

DexH13 communication supports `STUB`, `SDK`, `MODBUS_TCP`, and `MODBUS_RTU`. The SDK path uses PaXini DexHandSDK v1.1.0 (`pxdex.dh13.DexH13Control`) and maps LeRobot's 13 active hand actions to the SDK's 16 angle slots. Hand teleoperation defaults to `dex-retargeting` / DexPilot with the bundled `dexh13_right` URDF; use `--dexh13-hand-backend geometry` to fall back to the lightweight landmark-angle heuristic. Install the vendor package on the robot PC:

```bash
sudo dpkg -i DexHandSDK-1.1.0-Linux.deb
python3 -m pip install pxdex-1.1.0-cp310-cp310-linux_x86_64.whl
```

The vendor SDK package is built for Ubuntu 22.04 / Python 3.10.12. If your LeRobot environment is Python 3.12, run DexH13 in a Python 3.10 environment or obtain a matching `pxdex` wheel from the vendor. The DexH13 manual also notes that RJ45 EtherCAT and RS485 Modbus-RTU should not be used at the same time; if both are connected, RS485 takes effect and RJ45 cannot be used.

For a command-by-command setup, see [`REPRODUCTION_UR5E_DEXH13.md`](REPRODUCTION_UR5E_DEXH13.md). For Meta Quest USB/ADB networking, see [`QUEST_CONNECTION.md`](QUEST_CONNECTION.md).

## Build

Project was tested on [`LeRobot`](https://github.com/huggingface/lerobot) commit [`ce3b9f627e55223d6d1c449d348c6b351b35d082`](https://github.com/huggingface/lerobot/commit/ce3b9f627e55223d6d1c449d348c6b351b35d082), with Ubuntu `24.04` and Python `3.12`. To use this extension, copy and paste all content inside the repo over to your `LeRobot` directory and do the following:

### 1. Franka server
This needs to be built and deployed on your real-time machine that controls the robot.

```bash
cd franka_server
bash build.sh
```

Build `franka_server` and run on your robot RTPC (or run in a second terminal if it's the same PC) 

<details> 
<summary><strong>franka_server Dependencies</strong></summary>

- [ruckig](https://github.com/pantor/ruckig)

- [libfranka](https://github.com/frankarobotics/libfranka) (must match your robot firmware version)

- [pinocchio](https://github.com/stack-of-tasks/pinocchio) (required if `libfranka` > 0.14.0)

</details>

>[!NOTE]
> You will need to run the following commands on RTPC to start up the server before using any LeRobot utilities to move the arm.

```bash
./franka_server [YOUR_FRANKA_ROBOT_IP]
```

### 2. Franka teleoperator

This needs to be built and added to your environment
   
```bash
cd franka_xhand_teleoperator
[uv] pip install -e .
```

Additionally, you will also need to set up Meta Quest VR App from [this repo](https://github.com/wengmister/franka-vr-teleop)

<details>
<summary><strong>XHand Dependencies</strong></summary>

For `XHand`, we will use a repository modified based on Yuzhe Qin's amazing work on [`dex-retargeting`](https://github.com/dexsuite/dex-retargeting) to map human hand motion to the robot hand. 

To enable XHand Motion Retargeting:

```bash
# First, update all git submodule
git submodule update --init --recursive 

# Build dependencies
cd vr-dex-retargeting
[uv] pip install -e .
```

You will also need to install RobotEra's python API to control the XHand robot. Download the python wheel from [RobotEra's document center](https://di6kz6gamrw.feishu.cn/file/HucBbWKPEo7JGAxosn7ckicOnTf), and install to your environment.

```bash
# Use your corresponding python version
[uv] pip install xhand_controller-1.1.7-cp312-cp312-linux_x86_64.whl 
```

</details>

### 3. LeRobot classes

Copy to merge with files under LeRobot's `src` directory. This includes new Robot and Teleoperator class implementations needed to work with the rest of the framework.

>[!NOTE]
> Due to time constraints and the nature of the project, I didn't develop an interface for the Franka Hand gripper since I didn't use it, but contributions are welcome!

## Usage

Call any LeRobot utility as you would with the new Robots! Examples can be found under [`scripts`](https://github.com/wengmister/LeFranX/tree/main/scripts).

>[!CAUTION] 
>If you're comboing robots together like I did with Franka + XHand, it might be a good idea to call utility methods directly in your own python script as opposed to using the existing python implementations with arguments (for training and rollout, etc.). Combo robot setup will cause circular import calls with `Draccus`; I'm currently bypassing this issue by constructing train and deployment scripts directly. Please let me know if you would have a better method to deal with this.

## Demo tasks
### Pick up orange cube and place in blue bin:

https://github.com/user-attachments/assets/5e6e1930-6bca-4d1a-b175-423de4388dc1

### Pick up toast, insert in toaster, and press toast lever:

https://github.com/user-attachments/assets/03dbfd55-91e3-40f0-9e5c-fca9b33fad30

### Open box lid, pick up pie and place in brown bin:

https://github.com/user-attachments/assets/e5b54e07-031d-42e2-994b-030e646e2768

## Datasets
Open-source datasets for the demo tasks could be found on HuggingFace [here](https://huggingface.co/wengmister).

## Citation
If you find this work helpful, please consider citing as:

      @misc{weng2025levr,
            title={LeVR: A Modular VR Teleoperation Framework for Imitation Learning in Dexterous Manipulation}, 
            author={Zhengyang Kris Weng and Matthew L. Elwin and Han Liu},
            year={2025},
            eprint={2509.14349},
            archivePrefix={arXiv},
            primaryClass={cs.RO},
            url={https://arxiv.org/abs/2509.14349}, 
      }

## License
Apache-2.0
