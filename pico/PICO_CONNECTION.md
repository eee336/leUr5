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

## 4. 准备 PICO Unity 工程

1. 创建或打开一个 Unity Android XR 工程。
2. 导入 PICO Unity Integration SDK / PICO Unity XR SDK。
3. 在 XR Plug-in Management 中启用 PICO XR plugin。
4. 在 PICO 项目设置和头显设置中启用手部追踪。
5. 新建一个空 GameObject，命名为 `LeUr5PicoStreamer`。
6. 把 `pico/unity/PicoLeUr5HandStreamer.cs` 加入 Unity 工程。
7. 将 `PicoLeUr5HandStreamer` 挂载到 `LeUr5PicoStreamer`。
8. 在 Inspector 中设置：

```text
Computer Host = 电脑 IP
Computer Port = 8000
Stream Hz = 30
```

然后 Build 并安装到 PICO 头显。

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
