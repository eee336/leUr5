# Meta Quest 连接教程

这份文档说明如何把 Meta Quest 头显连接到机器人控制电脑，用于 UR5e + DexH13 VR 遥操作。

推荐连接方式是：

```text
Quest VR App
  -> USB 数据线
  -> adb reverse tcp:8000
  -> 电脑 localhost:8000
  -> LeFranX VR message router
  -> UR5e + DexH13 遥操作
```

## 1. 准备 Quest

在 Quest 头显上：

1. 开启开发者模式。
2. 安装匹配的 Quest VR 遥操作 App。
3. 使用 USB-C 数据线连接 Quest 和电脑。
4. 戴上头显，在弹窗中允许 USB debugging / RSA 授权。

## 2. 安装 ADB

Ubuntu：

```bash
sudo apt update
sudo apt install android-tools-adb -y
```

macOS：

```bash
brew install android-platform-tools
```

检查 Quest 是否被电脑识别：

```bash
adb devices
```

正常输出类似：

```text
List of devices attached
1WMHHxxxxxxxx    device
```

如果显示 `unauthorized`，戴上 Quest 并同意调试授权。

## 3. 配置 USB 端口转发

项目脚本默认会自动配置 ADB reverse。除非你想手动管理网络，否则不要传 `--no-adb`。

手动配置命令：

```bash
adb reverse tcp:8000 tcp:8000
adb reverse --list
```

正常输出类似：

```text
1WMHHxxxxxxxx tcp:8000 tcp:8000
```

含义是：Quest App 连接头显自己的 `localhost:8000` 时，ADB 会把流量转发到电脑的 `localhost:8000`。

## 4. 启动电脑端遥操作脚本

先启动电脑端 LeFranX 脚本：

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --robot-ip 192.168.1.10 \
  --dexh13-protocol SDK \
  --dexh13-hand-port /dev/ttyUSB0 \
  --dexh13-camera-port none \
  --dexh13-hand-backend retargeting \
  --vr-port 8000
```

只做软件启动测试：

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --ur-stub \
  --dexh13-protocol STUB \
  --no-home \
  --vr-port 8000
```

## 5. 启动 Quest App

电脑端脚本运行后，再打开 Quest 里的 VR 遥操作 App。

App 应该发送：

- 手腕位置
- 手腕四元数
- 21 个手部 landmarks
- tracking valid/status 状态

电脑端脚本会用 wrist 数据控制 UR5e 末端，用 hand landmarks 做 DexH13 retargeting。

## 6. 验证数据流

另开一个终端：

```bash
adb devices
adb reverse --list
```

如果电脑端脚本开启了 verbose：

```bash
--verbose
```

日志里应该能看到 VR connection/status 信息。

## 7. 排错

重启 ADB：

```bash
adb kill-server
adb start-server
adb devices
adb reverse tcp:8000 tcp:8000
```

如果看不到设备：

```bash
adb devices
```

然后重新插拔 USB 线，并在 Quest 里确认调试授权。

如果设备状态是 `unauthorized`，戴上头显并允许 USB debugging。

如果脚本提示没有 VR 数据：

```bash
adb reverse --list
```

如果没有 `tcp:8000 tcp:8000`，执行：

```bash
adb reverse tcp:8000 tcp:8000
```

如果电脑端 `8000` 端口被占用，可以两边统一换成其他端口：

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --vr-port 8001 \
  --ur-stub \
  --dexh13-protocol STUB
```

同时执行：

```bash
adb reverse tcp:8001 tcp:8001
```

## 8. WiFi 替代方案

第一次调试建议使用 USB + ADB reverse。

如果 Quest App 支持直接连接电脑 IP，也可以走 WiFi：

1. Quest 和电脑连接到同一个网络。
2. 查看电脑 IP：

   ```bash
   ip addr
   ```

3. 如果电脑开了防火墙，放行 TCP 端口：

   ```bash
   sudo ufw allow 8000/tcp
   ```

4. 在 Quest App 里配置连接地址：

   ```text
   COMPUTER_IP:8000
   ```

5. 启动脚本时加 `--no-adb`：

   ```bash
   python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
     --vr-port 8000 \
     --no-adb \
     --robot-ip 192.168.1.10 \
     --dexh13-protocol SDK \
     --dexh13-hand-port /dev/ttyUSB0
   ```

WiFi 模式会额外引入路由器、防火墙和 IP 配置变量；建议 USB 模式跑通后再切换。
