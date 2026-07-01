# UR5e + DexH13 Reproduction Guide

This guide reproduces the UR5e + PaXini DexH13 adaptation in this repository.

## 1. Clone

```bash
git clone --recursive https://github.com/eee336/leUr5.git
cd leUr5
git submodule update --init --recursive
```

If you cloned without `--recursive`, run only the second command after entering the repo.

## 2. Create Python Environment

The original LeFranX/LeRobot stack was tested with Python 3.12, but PaXini DexHandSDK v1.1.0 provides a `cp310` wheel for Python 3.10. For real DexH13 hardware, use Python 3.10 unless PaXini provides a wheel matching your Python version.

```bash
conda create -n leur5-dexh13 python=3.10 -y
conda activate leur5-dexh13
python -m pip install --upgrade pip
```

If you only want to test software stub mode without DexH13 hardware, Python 3.12 is also acceptable for the LeRobot side.

## 3. Install LeRobot Base

This repository is a LeRobot extension. Install the upstream LeRobot package first, then install this repo on top of it.

```bash
git clone https://github.com/huggingface/lerobot.git ../lerobot
cd ../lerobot
git checkout ce3b9f627e55223d6d1c449d348c6b351b35d082
python -m pip install -e .

cd ../leUr5
python -m pip install -e .
```

If editable install is not available for your local checkout, copy this repository's `src/lerobot` contents over the upstream LeRobot checkout and run the scripts from this repository root with `PYTHONPATH=src`.

## 4. Install UR5e Dependencies

```bash
python -m pip install ur-rtde
```

Make sure the UR5e control PC can reach the robot controller IP, for example:

```bash
ping 192.168.1.10
```

Replace `192.168.1.10` with your actual UR5e controller IP in later commands.

## 5. Install VR Teleoperation Router

```bash
cd franka_xhand_teleoperator
python -m pip install -e .
cd ..
```

If you use a Meta Quest device, keep ADB enabled. The scripts configure ADB reverse by default; pass `--no-adb` only if you handle networking manually.

## 5.1. Install Dex Retargeting Dependencies

DexH13 hand mapping uses the `dexh13_right` URDF with `dex-retargeting` / DexPilot by default. Install the retargeting package from the bundled source:

```bash
python -m pip install -e vr-dex-retargeting
```

The DexH13 retargeting config is:

```text
dexh13_right/config/dexh13_right_dexpilot.yml
```

It uses the URDF:

```text
dexh13_right/urdf/dexh13_right.urdf
```

## 6. Install PaXini DexH13 SDK

Unzip the SDK package supplied by PaXini:

```bash
mkdir -p vendor/DexHandSDK
unzip /path/to/DexHandSDK-v1.1.0-Linux-x86_64.zip -d vendor/DexHandSDK
cd vendor/DexHandSDK/DexHandSDK
```

Install the Debian package and Python wheel:

```bash
sudo dpkg -i DexHandSDK-1.1.0-Linux.deb
python -m pip install pxdex-1.1.0-cp310-cp310-linux_x86_64.whl
```

Check that the Python SDK imports:

```bash
python - <<'PY'
from pxdex.dh13 import DexH13Control, ControlMode, FingerAngle
print("DexH13 SDK import OK")
PY
```

Return to the repository root:

```bash
cd ../../..
```

## 7. Connect Hardware

Use one DexH13 control interface at a time.

- SDK/RS485: connect DexH13 RS485 and power, then find the serial device.
- EtherCAT/RJ45: use the vendor EtherCAT example separately; this LeRobot path currently uses the SDK/RS485 API.
- Do not connect RJ45 EtherCAT and RS485 Modbus-RTU for simultaneous control. The DexH13 manual says RS485 takes effect when both are connected.

Find serial and camera devices:

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
ls /dev/video* 2>/dev/null || true
```

Typical DexH13 SDK arguments are:

```bash
DEXH13_HAND_PORT=/dev/ttyUSB0
DEXH13_CAMERA_PORT=none
```

Use `/dev/video0` instead of `none` if you want the SDK to open the hand camera.

## 8. Software Stub Smoke Test

This checks the script wiring without moving real hardware:

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --ur-stub \
  --dexh13-protocol STUB \
  --no-adb \
  --no-home \
  --verbose
```

Stop with `Ctrl+C`.

## 9. Real UR5e + DexH13 VR Teleoperation

Start with a low-risk setup: robot in a clear workspace, emergency stop reachable, and no automatic DexH13 zeroing.

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --robot-ip 192.168.1.10 \
  --dexh13-protocol SDK \
  --dexh13-hand-port /dev/ttyUSB0 \
  --dexh13-camera-port none \
  --dexh13-hand-backend retargeting \
  --dexh13-retargeting-config dexh13_right/config/dexh13_right_dexpilot.yml \
  --control-freq 30
```

If PaXini support instructs you to initialize motor positions at startup, add:

```bash
--dexh13-init-motor-position
```

## 10. Record a Dataset

```bash
python scripts/ur5e_dexh13/record_ur5e_dexh13_vr.py \
  --dataset-path ./datasets/ur5e_dexh13_demo \
  --robot-ip 192.168.1.10 \
  --dexh13-protocol SDK \
  --dexh13-hand-port /dev/ttyUSB0 \
  --dexh13-camera-port none \
  --dexh13-hand-backend retargeting \
  --fps 30 \
  --num-episodes 10 \
  --episode-time 60 \
  --camera wrist=/dev/video0
```

Use `--resume` to continue an existing dataset or `--overwrite` to recreate it.

## 11. Useful Variants

UR5e only:

```bash
python scripts/ur5e/ur5e_vr_teleoperator.py --robot-ip 192.168.1.10
```

UR5e + XHand:

```bash
python scripts/ur5e_xhand/ur5e_xhand_vr_teleoperator.py \
  --robot-ip 192.168.1.10 \
  --xhand-port /dev/ttyUSB0
```

DexH13 with SDK import but no motor enable:

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --ur-stub \
  --dexh13-protocol SDK \
  --dexh13-hand-port /dev/ttyUSB0 \
  --dexh13-no-enable \
  --no-home
```

DexH13 with the old lightweight geometry mapping instead of URDF retargeting:

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --robot-ip 192.168.1.10 \
  --dexh13-protocol SDK \
  --dexh13-hand-port /dev/ttyUSB0 \
  --dexh13-camera-port none \
  --dexh13-hand-backend geometry
```

## 12. Troubleshooting

If `ModuleNotFoundError: pxdex` appears, confirm that the PaXini wheel was installed into the active Python 3.10 environment.

If `activeHandy` fails, check the serial device path and permissions:

```bash
groups
sudo usermod -aG dialout "$USER"
```

Log out and log back in after changing groups.

If the UR5e does not connect, check the robot IP, network route, UR remote control mode, and RTDE availability.

If the hand moves in the wrong finger order, verify that actions follow the repository order:

```text
0-2   index:  abduction, mcp, pip
3-5   middle: abduction, mcp, pip
6-8   ring:   abduction, mcp, pip
9-12  thumb:  abduction, mcp, pip, dip
```

The `dexh13_right` URDF order is thumb/index/middle/ring, while the DexH13 SDK/EtherCAT active command order in this repository is index/middle/ring/thumb. The teleoperator maps by URDF joint name before sending actions, so do not reorder action indices manually unless you also update the mapping in `DexH13VRTeleoperator`.
