# PICO 连接教程

这份文档说明如何用 PICO 头显替代 Quest USB/ADB 数据源。

电脑端保持不变：LeFranX 通过 `vr_message_router` 启动 TCP server，并接收下面两类文本消息：

```text
Right wrist:, x, y, z, qx, qy, qz, qw, leftFist: state
Right landmarks: x0,y0,z0,x1,y1,z1,...,x20,y20,z20
```

PICO 端运行一个 Unity App，读取 PICO 手部追踪数据，并通过 WiFi 把这两类消息发给电脑。

## 1. PICO 提供的数据

PICO 官方手部追踪文档：<https://developer-cn.picoxr.com/document/unity/hand-tracking/>

PICO Unity XR SDK 通过下面接口提供手部追踪：

- `PXR_HandTracking.GetSettingState()`
- `PXR_HandTracking.GetActiveInputDevice()`
- `PXR_HandTracking.GetJointLocations(HandType.HandRight, ref jointLocations)`
- `PXR_HandTracking.GetAimState(HandType.HandRight, ref aimState)`

PICO API 提供 26 个手部关节。LeFranX 电脑端沿用 21 点 MediaPipe/Quest 风格 landmarks，因此 `PicoLeUr5HandStreamer.cs` 会把 PICO 关节映射为：

```text
0 wrist
1-4 thumb
5-8 index
9-12 middle
13-16 ring
17-20 little
```

## 2. 网络结构

推荐 PICO 连接方式：

```text
PICO 头显和机器人控制电脑在同一 WiFi/LAN
PICO Unity App 连接 COMPUTER_IP:8000
电脑脚本使用 --no-adb --vr-port 8000
```

这条路线不使用 Quest 的 `adb reverse`。

## 3. 准备电脑端

查看电脑在同一网络下的 IP：

```bash
ip addr
```

如果启用了防火墙，放行 TCP 端口：

```bash
sudo ufw allow 8000/tcp
```

先运行软件模式烟测：

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --ur-stub \
  --dexh13-protocol STUB \
  --no-adb \
  --no-home \
  --vr-port 8000 \
  --verbose
```

连接真实硬件时：

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --robot-ip 192.168.1.10 \
  --dexh13-protocol SDK \
  --dexh13-hand-port /dev/ttyUSB0 \
  --dexh13-camera-port none \
  --dexh13-hand-backend retargeting \
  --no-adb \
  --vr-port 8000
```

注意保留 `--no-adb`，因为 PICO 通过 WiFi 直接连接电脑。

PICO 初次联调时建议使用更保守的手臂参数。当前默认已经采用保守值：

```text
arm_movement_scale = 0.25
arm_max_position_offset = 0.20 m
arm_max_position_step = 0.015 m/control-cycle
arm_position_deadzone = 0.005 m
arm_control_orientation = false
```

如果真实机械臂仍然移动过大，可以继续降低：

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --robot-ip 192.168.1.10 \
  --dexh13-protocol SDK \
  --dexh13-hand-port /dev/ttyUSB0 \
  --dexh13-camera-port none \
  --dexh13-hand-backend retargeting \
  --no-adb \
  --vr-port 8000 \
  --arm-movement-scale 0.10 \
  --arm-max-position-offset 0.10 \
  --arm-max-position-step 0.005 \
  --arm-position-deadzone 0.01
```

先不要加 `--arm-control-orientation`。等位移方向、尺度和安全空间都确认正确后，再打开姿态控制。

## 4. 准备 PICO Unity 工程

### 4.1 安装 Unity Android 构建环境

1. 安装 Unity Hub。
2. 安装 PICO Unity XR SDK 推荐的 Unity LTS 版本。通常优先选 `2021.3 LTS` 或 `2022.3 LTS`，以你下载的 PICO SDK 文档为准。
3. 在 Unity Hub 的对应 Unity 版本里安装这些模块：
   - `Android Build Support`
   - `Android SDK & NDK Tools`
   - `OpenJDK`
4. 在电脑上安装 Android platform tools，确保能使用 `adb`：

```bash
adb version
```

如果没有 `adb`，Ubuntu 可以安装：

```bash
sudo apt update
sudo apt install -y android-tools-adb
```

macOS 可以安装：

```bash
brew install android-platform-tools
```

### 4.2 创建 Unity 工程并导入 PICO SDK

1. 用 Unity Hub 新建一个 `3D Core` 工程，例如 `LeUr5PicoStreamer`。
2. 打开 `File -> Build Settings...`。
3. 选择 `Android`，点击 `Switch Platform`。
4. 导入 PICO Unity Integration SDK / PICO Unity XR SDK。
   - 如果是 `.unitypackage`：双击或用 `Assets -> Import Package -> Custom Package...` 导入。
   - 如果是 UPM package：用 `Window -> Package Manager -> + -> Add package from disk...` 或 PICO 文档指定方式导入。
5. 打开 `Edit -> Project Settings -> XR Plug-in Management`。
6. 在 `Android` 标签页启用 `PICO` XR plugin。
7. 打开 PICO 相关 Project Settings，根据 PICO SDK 文档启用 hand tracking / hand tracking support。

### 4.3 设置 Android Player

打开 `Edit -> Project Settings -> Player -> Android`，建议检查这些项：

```text
Company Name = lefranx
Product Name = LeUr5PicoStreamer
Package Name = com.lefranx.pico
Internet Access = Require
Scripting Backend = IL2CPP
Target Architectures = ARM64
Minimum API Level = 按 PICO SDK 推荐值
Target API Level = Automatic 或按 PICO SDK 推荐值
```

`Internet Access = Require` 很重要，因为 PICO App 需要通过 WiFi 连接电脑端 TCP server。

### 4.4 加入 LeUr5 PICO 发送脚本

1. 在 Unity 工程里新建目录 `Assets/Scripts`。
2. 把本仓库的脚本复制到 Unity 工程：

```bash
cp /path/to/LeFranX/pico/unity/PicoLeUr5HandStreamer.cs \
  /path/to/LeUr5PicoStreamer/Assets/Scripts/
```

3. 在 Unity Hierarchy 里新建空对象：

```text
GameObject -> Create Empty
Name = LeUr5PicoStreamer
```

4. 选中 `LeUr5PicoStreamer`，在 Inspector 里点击 `Add Component`。
5. 添加 `Pico Le Ur5 Hand Streamer` 组件。
6. 在 Inspector 中设置：

```text
Computer Host = 电脑 IP
Computer Port = 8000
Stream Hz = 30
Verbose = true
```

这里的 `Computer Host` 必须填电脑在同一 WiFi/LAN 下的 IP，不能填 `localhost` 或 `127.0.0.1`。

### 4.5 Build APK

1. 打开 `File -> Build Settings...`。
2. 确认平台是 `Android`。
3. 点击 `Add Open Scenes`，把当前 scene 加入 `Scenes In Build`。
4. 点击 `Build`。
5. 选择输出路径，例如：

```text
LeUr5PicoStreamer/build/LeUr5PicoStreamer.apk
```

如果你已经用 USB 连接好了 PICO，也可以点击 `Build And Run`，Unity 会尝试构建并直接安装到头显。

### 4.6 安装到 PICO 头显

先在 PICO 头显里打开开发者模式和 USB 调试。不同系统版本菜单名称略有差异，一般在开发者选项或设备管理相关页面里。

用 USB-C 连接 PICO 和电脑后，在电脑上检查设备：

```bash
adb devices
```

第一次连接时，头显里通常会弹出 USB 调试授权，请在头显内确认允许。正常情况下会看到类似：

```text
List of devices attached
XXXXXXXX	device
```

安装 APK：

```bash
adb install -r /path/to/LeUr5PicoStreamer/build/LeUr5PicoStreamer.apk
```

安装成功后，在 PICO 的应用列表里启动 `LeUr5PicoStreamer`。如果应用没有出现在普通应用列表，查看 PICO 的未知来源、开发者应用或全部应用列表。

如果你希望用 Unity 一键安装：

1. USB 连接 PICO。
2. 运行 `adb devices` 确认设备状态是 `device`。
3. 回到 Unity。
4. 打开 `File -> Build Settings...`。
5. 选择 `Build And Run`。

Unity 会完成 build、install、launch 三步。失败时优先看 Unity Console 和 `adb devices` 状态。

## 5. 运行顺序

1. 确认 PICO 和电脑在同一网络。
2. 先启动 LeFranX 电脑端脚本。
3. 再启动 PICO Unity App。
4. 把右手放到头显手部追踪范围内。
5. 查看电脑端日志，确认 `tcp_connected`、`wrist_valid`、`landmarks_valid` 变为有效。

## 6. 坐标系标定

PICO 脚本默认发送 PICO/Unity 世界坐标。当前 UR5e VR 处理器会套用和 Quest 相同的 VR 到机器人坐标变换：

```text
Robot X = VR Z
Robot Y = -VR X
Robot Z = VR Y
```

这只是起始假设，不是现场标定。如果你在 PICO 里手向前移动，而 UR5e 末端方向不对，可以调整下面任意一处：

- Unity 端：`PicoLeUr5HandStreamer.cs` 里的 `ConvertUnityPositionForLeFranX()`
- Python 端：`src/lerobot/teleoperators/ur5e_vr/arm_ik_processor.py` 里的 `_compute_target_matrix()`

建议一次只改一边，避免叠加变换后更难排查。

## 7. 排错

检查电脑端是否在监听：

```bash
ss -ltnp | grep 8000
```

检查 PICO 是否能访问电脑：

```bash
ping COMPUTER_IP
```

如果电脑日志里没有 VR 连接：

- 确认 Unity 中 `Computer Host` 是电脑 IP，而不是 `localhost`
- 确认防火墙允许 `8000/tcp`
- 确认 PICO 和电脑在同一网段
- 确认先启动了 LeFranX 电脑端脚本，再启动 PICO App

如果已经连接但没有手部数据：

- 确认 PICO 头显设置里启用了 hand tracking
- 确认控制器没有抢占 active input
- 确认手在头显追踪范围内
- 在 Unity 里打印 `PXR_HandTracking.GetSettingState()` 和 `GetActiveInputDevice()` 的结果

如果 landmarks 有数据但 retargeting 看起来镜像或旋转不对：

- 先切到几何后端做简单验证：

  ```bash
  --dexh13-hand-backend geometry
  ```

- 再调整 Unity 坐标转换或 Python VR 到 robot 的坐标转换。

如果手稍微动一下，UR5e 末端就大幅扭转或移动：

- 先确认启动命令没有加 `--arm-control-orientation`。PICO 手腕 quaternion 未标定前很容易让 UR5e 末端姿态突然翻转。
- 把 `--arm-movement-scale` 降到 `0.10` 或 `0.05`。
- 把 `--arm-max-position-step` 降到 `0.005`，这会限制每个控制周期最多移动 5mm。
- 把 `--arm-max-position-offset` 降到 `0.10`，先把工作空间限制在初始点周围 10cm。
- 如果静止时还抖动，把 `--arm-position-deadzone` 提到 `0.01`。
- 每次戴上 PICO 或重新进入 Unity App 后，先让手保持在舒适的中立位置，再启动电脑端脚本；电脑端会把第一帧 VR 手腕位置作为零点。
- 如果手向前，UR5e 却向左/向下移动，说明坐标轴映射不对，优先改 `src/lerobot/teleoperators/ur5e_vr/arm_ik_processor.py` 的 `_compute_target_matrix()`，不要同时改 Unity 端和 Python 端。
