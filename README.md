# LeFranX
[🤗HF Dataset](https://huggingface.co/collections/wengmister/lefranx-dataset-68b4684269dbb97fd3061be0) | [arXiv](https://arxiv.org/abs/2509.14349) | [XHand Coupling CAD](https://github.com/wengmister/xhand-coupling)

LeRobot Extension for Franka FER Robot & XHand Robot. An instantiation of the [LeVR](https://arxiv.org/abs/2509.14349) framework.

[![Watch the video](https://img.youtube.com/vi/TzlUEWCjQ1M/0.jpg)](https://www.youtube.com/watch?v=TzlUEWCjQ1M)

## 架构

这个 LeRobot Franka 扩展主要包含三部分：`franka_server`、`franka_xhand_teleoperator`，以及 `src/lerobot` 下新增的类实现。系统流程见下图：


<img width="1891" height="1649" alt="flow-chart" src="https://github.com/user-attachments/assets/cfd8389a-2ecf-4e1c-8f6f-ca1aa0905fbf" />

## UR5e 适配

这个 fork 在原 Franka 路线之外，新增了 UR5e 路线：

- `src/lerobot/robots/ur5e`：通过 Universal Robots RTDE 控制 UR5e 机械臂。
- `src/lerobot/robots/ur5e_xhand`：UR5e + XHand 组合机器人。
- `src/lerobot/robots/dexh13`：帕西尼 DexH13 灵巧手，13 个主动关节。
- `src/lerobot/robots/ur5e_dexh13`：UR5e + DexH13 组合机器人。
- `src/lerobot/teleoperators/ur5e_vr`：基于 VR 手腕位姿控制 UR5e。
- `src/lerobot/teleoperators/ur5e_xhand_vr`：UR5e + XHand 共享 VR 数据的遥操作。
- `src/lerobot/teleoperators/dexh13_vr`：基于手部 landmarks 的 DexH13 遥操作。
- `src/lerobot/teleoperators/ur5e_dexh13_vr`：UR5e + DexH13 共享 VR 数据的遥操作。
- `scripts/ur5e` 和 `scripts/ur5e_xhand`：示例控制脚本。
- `scripts/ur5e_dexh13`：UR5e + DexH13 遥操作与数据录制脚本。

和 Franka 路线不同，UR5e 不使用 `franka_server`。请在 LeRobot 同一个环境中安装 UR RTDE Python 绑定：

```bash
[uv] pip install ur-rtde
```

UR5e + XHand / DexH13 的 VR 遥操作仍然需要构建 VR message router 扩展：

```bash
cd franka_xhand_teleoperator
[uv] pip install -e .
```

然后可以运行下面的 UR5e 示例：

```bash
python scripts/ur5e/ur5e_vr_teleoperator.py --robot-ip YOUR_UR5E_IP
python scripts/ur5e_xhand/ur5e_xhand_vr_teleoperator.py --robot-ip YOUR_UR5E_IP --xhand-port /dev/ttyUSB0
python scripts/ur5e_xhand/record_ur5e_xhand_vr.py --dataset-path ./datasets/ur5e_xhand_demo --robot-ip YOUR_UR5E_IP --xhand-port /dev/ttyUSB0
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py --robot-ip YOUR_UR5E_IP --dexh13-protocol SDK --dexh13-hand-port /dev/ttyUSB0
python scripts/ur5e_dexh13/record_ur5e_dexh13_vr.py --dataset-path ./datasets/ur5e_dexh13_demo --robot-ip YOUR_UR5E_IP --dexh13-protocol SDK --dexh13-hand-port /dev/ttyUSB0
```

新增的 LeRobot 配置类型包括：`ur5e`、`dexh13`、`ur5e_xhand`、`ur5e_dexh13`、`ur5e_vr`、`dexh13_vr`、`ur5e_xhand_vr`、`ur5e_dexh13_vr`。

DexH13 通信支持 `STUB`、`SDK`、`MODBUS_TCP`、`MODBUS_RTU`。SDK 路线使用 PaXini DexHandSDK v1.1.0，也就是 `pxdex.dh13.DexH13Control`，并把 LeRobot 的 13 个主动关节 action 映射到 SDK 的 16 个角度槽位。手部遥操作默认使用 `dex-retargeting` / DexPilot 和仓库内置的 `dexh13_right` URDF；如果要回退到轻量几何映射，可使用 `--dexh13-hand-backend geometry`。在机器人控制电脑上安装厂商 SDK：

```bash
sudo dpkg -i DexHandSDK-1.1.0-Linux.deb
python3 -m pip install pxdex-1.1.0-cp310-cp310-linux_x86_64.whl
```

厂商 SDK 包面向 Ubuntu 22.04 / Python 3.10.12 构建。如果你的 LeRobot 环境是 Python 3.12，请为 DexH13 使用 Python 3.10 环境，或向厂商获取匹配版本的 `pxdex` wheel。DexH13 说明书还说明：RJ45 EtherCAT 和 RS485 Modbus-RTU 不应同时用于控制；如果两者同时连接，RS485 生效，RJ45 不能使用。

完整命令级复现流程见 [`REPRODUCTION_UR5E_DEXH13.md`](REPRODUCTION_UR5E_DEXH13.md)。Meta Quest USB/ADB 连接教程见 [`QUEST_CONNECTION.md`](QUEST_CONNECTION.md)。PICO WiFi 推流教程见 [`pico/PICO_CONNECTION.md`](pico/PICO_CONNECTION.md)。

## 构建

原项目在 [`LeRobot`](https://github.com/huggingface/lerobot) commit [`ce3b9f627e55223d6d1c449d348c6b351b35d082`](https://github.com/huggingface/lerobot/commit/ce3b9f627e55223d6d1c449d348c6b351b35d082)、Ubuntu `24.04`、Python `3.12` 下测试。使用该扩展时，可将本仓库内容覆盖到你的 `LeRobot` 目录中，然后执行下面步骤。

### 1. Franka server

这部分需要在控制机器人的实时机器上构建和部署。

```bash
cd franka_server
bash build.sh
```

构建 `franka_server` 后，在机器人 RTPC 上运行；如果控制和开发在同一台机器，也可以另开一个终端运行。

<details> 
<summary><strong>franka_server Dependencies</strong></summary>

- [ruckig](https://github.com/pantor/ruckig)

- [libfranka](https://github.com/frankarobotics/libfranka) (must match your robot firmware version)

- [pinocchio](https://github.com/stack-of-tasks/pinocchio) (required if `libfranka` > 0.14.0)

</details>

>[!NOTE]
> 在使用 LeRobot 工具移动机械臂前，需要先在 RTPC 上执行下面命令启动 server。

```bash
./franka_server [YOUR_FRANKA_ROBOT_IP]
```

### 2. Franka teleoperator

这部分需要构建并安装到当前 Python 环境。
   
```bash
cd franka_xhand_teleoperator
[uv] pip install -e .
```

此外，还需要按照 [这个仓库](https://github.com/wengmister/franka-vr-teleop) 配置 Meta Quest VR App。

<details>
<summary><strong>XHand 依赖</strong></summary>

对于 `XHand`，项目使用了基于 Yuzhe Qin 的 [`dex-retargeting`](https://github.com/dexsuite/dex-retargeting) 修改而来的仓库，将人手动作映射到机器人手。

启用 XHand 动作 retargeting：

```bash
# 首先更新所有 git submodule
git submodule update --init --recursive 

# 构建依赖
cd vr-dex-retargeting
[uv] pip install -e .
```

还需要安装 RobotEra 的 Python API 来控制 XHand。请从 [RobotEra 文档中心](https://di6kz6gamrw.feishu.cn/file/HucBbWKPEo7JGAxosn7ckicOnTf) 下载 Python wheel，并安装到当前环境。

```bash
# 使用与你 Python 版本对应的 wheel
[uv] pip install xhand_controller-1.1.7-cp312-cp312-linux_x86_64.whl 
```

</details>

### 3. LeRobot classes

将本仓库内容复制并合并到 LeRobot 的 `src` 目录下。这里包含与框架其他部分配合所需的新 Robot 和 Teleoperator 类实现。

>[!NOTE]
> 由于时间和项目需求限制，原作者没有为 Franka Hand gripper 开发接口，因为项目中没有使用它；欢迎贡献补充。

## 使用

可以像使用其他 LeRobot 机器人一样调用相关工具。示例脚本位于 [`scripts`](https://github.com/wengmister/LeFranX/tree/main/scripts)。

>[!CAUTION] 
> 如果像 Franka + XHand 这样组合多个机器人，建议在自己的 Python 脚本里直接调用工具方法，而不是完全依赖已有命令行参数实现。组合机器人配置可能会触发 `Draccus` 的循环 import。原作者当前通过手写训练和部署脚本绕过该问题；如果你有更好的方式，也欢迎改进。

## 演示任务

### 拾取橙色方块并放入蓝色盒子

https://github.com/user-attachments/assets/5e6e1930-6bca-4d1a-b175-423de4388dc1

### 拾取吐司，放入烤面包机，并按下开关

https://github.com/user-attachments/assets/03dbfd55-91e3-40f0-9e5c-fca9b33fad30

### 打开盒盖，拾取派并放入棕色盒子

https://github.com/user-attachments/assets/e5b54e07-031d-42e2-994b-030e646e2768

## 数据集

演示任务的开源数据集可在 HuggingFace [这里](https://huggingface.co/wengmister) 获取。

## 引用

如果这项工作对你有帮助，请考虑引用：

      @misc{weng2025levr,
            title={LeVR: A Modular VR Teleoperation Framework for Imitation Learning in Dexterous Manipulation}, 
            author={Zhengyang Kris Weng and Matthew L. Elwin and Han Liu},
            year={2025},
            eprint={2509.14349},
            archivePrefix={arXiv},
            primaryClass={cs.RO},
            url={https://arxiv.org/abs/2509.14349}, 
      }

## 许可证
Apache-2.0
