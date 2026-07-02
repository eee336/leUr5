# PICO 头显接入

这个目录提供一条用 PICO 替代 Quest 作为 VR 数据源的路线。

电脑端 LeFranX 脚本已经会在 `--vr-port` 指定端口监听简单 TCP 文本协议，默认端口是 `8000`。PICO 端只需要发送同样格式的 wrist 和 hand-landmark 消息，UR5e + DexH13 的 Python 代码就可以继续复用。

适用场景：

```text
PICO 头显 + PICO Unity 手部追踪
  -> WiFi TCP 数据流
  -> LeFranX VR message router
  -> UR5e 末端控制 + DexH13 retargeting
```

文件说明：

- `unity/PicoLeUr5HandStreamer.cs`：Unity 组件，读取 PICO 右手追踪数据，并发送 LeFranX 兼容 TCP 消息。
- `PICO_CONNECTION.md`：完整安装、连接、运行和排错教程。
