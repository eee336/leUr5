# 脚本说明

本目录包含不同任务的入口脚本，覆盖：

- 遥操作
- 轨迹回放
- 数据录制
- 训练
- 部署

UR5e 相关入口：

- `scripts/ur5e/ur5e_vr_teleoperator.py`
- `scripts/ur5e_xhand/ur5e_xhand_vr_teleoperator.py`
- `scripts/ur5e_xhand/record_ur5e_xhand_vr.py`
- `scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py`
- `scripts/ur5e_dexh13/record_ur5e_dexh13_vr.py`

DexH13 SDK 模式使用 `pxdex.dh13.DexH13Control`。安装厂商 DexHandSDK 后，运行时传入：

```bash
--dexh13-protocol SDK --dexh13-hand-port /dev/ttyUSB0
```

更多参数请查看对应脚本的 `--help` 输出。
