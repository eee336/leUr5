Example scripts under these directories:

- Teleoperate
- Trajectory replay
- Record
- Train
- Deploy

UR5e-specific entry points:

- `scripts/ur5e/ur5e_vr_teleoperator.py`
- `scripts/ur5e_xhand/ur5e_xhand_vr_teleoperator.py`
- `scripts/ur5e_xhand/record_ur5e_xhand_vr.py`
- `scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py`
- `scripts/ur5e_dexh13/record_ur5e_dexh13_vr.py`

DexH13 SDK mode uses `pxdex.dh13.DexH13Control`; pass `--dexh13-protocol SDK --dexh13-hand-port /dev/ttyUSB0` after installing the vendor DexHandSDK package.

Please refer to scripts for more details.
