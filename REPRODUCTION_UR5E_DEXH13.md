# UR5e + DexH13 完整复现流程

这份文档用于在一台新电脑上复现本仓库的 UR5e + 帕西尼 DexH13 灵巧手遥操作与数据录制流程。

建议真机环境使用：

```text
Ubuntu 22.04
Python 3.10
UR5e RTDE
PaXini DexHandSDK v1.1.0
Quest 或 PICO 头显
```

## 1. 克隆仓库

```bash
git clone --recursive https://github.com/eee336/leUr5.git
cd leUr5
git submodule update --init --recursive
```

如果克隆时没有加 `--recursive`，进入仓库后执行第二条命令即可补齐子模块。

## 2. 创建 Python 环境

原始 LeFranX/LeRobot 代码主要按 Python 3.12 测试，但帕西尼 DexHandSDK v1.1.0 提供的是 `cp310` wheel，也就是 Python 3.10。要连接真实 DexH13，建议使用 Python 3.10。

```bash
conda create -n leur5-dexh13 python=3.10 -y
conda activate leur5-dexh13
python -m pip install --upgrade pip
```

如果只测试 `STUB` 软件模式，不连接真实 DexH13，LeRobot 侧使用 Python 3.12 也可以。

## 3. 安装 LeRobot 基础环境

本仓库是 LeRobot 扩展。先安装上游 LeRobot，再安装本仓库：

```bash
git clone https://github.com/huggingface/lerobot.git ../lerobot
cd ../lerobot
git checkout ce3b9f627e55223d6d1c449d348c6b351b35d082
python -m pip install -e .

cd ../leUr5
python -m pip install -e .
```

如果当前 checkout 不支持 editable install，可以把本仓库的 `src/lerobot` 覆盖到上游 LeRobot checkout 中，或者在本仓库根目录用 `PYTHONPATH=src` 运行脚本。

## 4. 安装 UR5e 依赖

```bash
python -m pip install ur-rtde
```

确认控制电脑能访问 UR5e 控制器 IP：

```bash
ping 192.168.1.10
```

后续命令里的 `192.168.1.10` 请替换成你的 UR5e 实际 IP。

## 5. 安装 VR 消息路由模块

```bash
cd franka_xhand_teleoperator
python -m pip install -e .
cd ..
```

如果使用 Meta Quest，默认脚本会自动配置 ADB reverse。除非你自己管理网络连接，否则不要加 `--no-adb`。

Quest 连接完整教程见：

```text
QUEST_CONNECTION.md
```

如果使用 PICO 头显，走 WiFi TCP 直连，并且运行脚本时加 `--no-adb`。完整教程见：

```text
pico/PICO_CONNECTION.md
```

## 5.1. 安装 Dex Retargeting 依赖

DexH13 手部映射默认使用 `dexh13_right` URDF 和 `dex-retargeting` / DexPilot：

```bash
python -m pip install -e vr-dex-retargeting
```

DexH13 retargeting 配置文件：

```text
dexh13_right/config/dexh13_right_dexpilot.yml
```

使用的 URDF：

```text
dexh13_right/urdf/dexh13_right.urdf
```

## 6. 安装帕西尼 DexH13 SDK

解压帕西尼提供的 SDK 压缩包：

```bash
mkdir -p vendor/DexHandSDK
unzip /path/to/DexHandSDK-v1.1.0-Linux-x86_64.zip -d vendor/DexHandSDK
cd vendor/DexHandSDK/DexHandSDK
```

安装 Debian 包和 Python wheel：

```bash
sudo dpkg -i DexHandSDK-1.1.0-Linux.deb
python -m pip install pxdex-1.1.0-cp310-cp310-linux_x86_64.whl
```

检查 SDK 是否能 import：

```bash
python - <<'PY'
from pxdex.dh13 import DexH13Control, ControlMode, FingerAngle
print("DexH13 SDK import OK")
PY
```

回到仓库根目录：

```bash
cd ../../..
```

## 7. 连接硬件

DexH13 一次只使用一种控制接口。

- SDK/RS485：连接 DexH13 RS485 和电源，然后确认串口设备。
- EtherCAT/RJ45：请单独使用厂商 EtherCAT 示例；当前 LeRobot 路线使用 SDK/RS485 API。
- 不要同时用 RJ45 EtherCAT 和 RS485 Modbus-RTU 控制。DexH13 说明书写明：两者同时连接时 RS485 生效，RJ45 不能使用。

查看串口和相机设备：

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
ls /dev/video* 2>/dev/null || true
```

常见 DexH13 SDK 参数：

```bash
DEXH13_HAND_PORT=/dev/ttyUSB0
DEXH13_CAMERA_PORT=none
```

如果希望 SDK 打开灵巧手相机，把 `none` 改成实际设备，例如 `/dev/video0`。

## 8. 软件 Stub 烟测

这一步不会移动真实硬件，只检查脚本和 VR router 是否能启动：

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --ur-stub \
  --dexh13-protocol STUB \
  --no-adb \
  --no-home \
  --verbose
```

按 `Ctrl+C` 停止。

## 9. UR5e + DexH13 真机 VR 遥操作

第一次真机测试请低风险启动：工作空间清空，急停按钮在手边，不自动执行 DexH13 零位初始化。

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

如果帕西尼工程师或说明文档要求启动时初始化电机位置，再额外添加：

```bash
--dexh13-init-motor-position
```

## 10. 录制数据集

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

继续已有数据集使用 `--resume`，重新创建数据集使用 `--overwrite`。

如果暂时没有外部相机，可以去掉 `--camera wrist=/dev/video0`。

## 11. 常用变体

只运行 UR5e：

```bash
python scripts/ur5e/ur5e_vr_teleoperator.py --robot-ip 192.168.1.10
```

UR5e + XHand：

```bash
python scripts/ur5e_xhand/ur5e_xhand_vr_teleoperator.py \
  --robot-ip 192.168.1.10 \
  --xhand-port /dev/ttyUSB0
```

DexH13 SDK import 和连接测试，但不使能电机：

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --ur-stub \
  --dexh13-protocol SDK \
  --dexh13-hand-port /dev/ttyUSB0 \
  --dexh13-no-enable \
  --no-home
```

使用旧的轻量几何映射，而不是 URDF retargeting：

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --robot-ip 192.168.1.10 \
  --dexh13-protocol SDK \
  --dexh13-hand-port /dev/ttyUSB0 \
  --dexh13-camera-port none \
  --dexh13-hand-backend geometry
```

## 12. 排错

如果出现 `ModuleNotFoundError: pxdex`，确认帕西尼 wheel 安装到了当前 Python 3.10 环境。

如果 `activeHandy` 失败，检查串口路径和权限：

```bash
groups
sudo usermod -aG dialout "$USER"
```

修改用户组后需要注销并重新登录。

如果 UR5e 无法连接，检查：

- 机器人 IP 是否正确
- 控制电脑和机器人网络是否互通
- UR 是否开启 remote control
- RTDE 是否可用

如果灵巧手动作顺序不对，确认本仓库 action 顺序为：

```text
0-2   食指：外展/侧摆, MCP, PIP
3-5   中指：外展/侧摆, MCP, PIP
6-8   无名指：外展/侧摆, MCP, PIP
9-12  拇指：外展/侧摆, MCP, PIP, DIP
```

注意：`dexh13_right` URDF 顺序是 thumb/index/middle/ring，而 DexH13 SDK/EtherCAT 主动控制顺序是 index/middle/ring/thumb。代码已经按 URDF joint name 做映射，不要手动重排 action index，除非同时修改 `DexH13VRTeleoperator` 里的映射逻辑。
