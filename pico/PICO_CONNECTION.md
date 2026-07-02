# PICO Connection Guide

This guide replaces the Quest USB/ADB data source with a PICO headset.

The computer side is unchanged: LeFranX runs a TCP server through `vr_message_router` and consumes:

```text
Right wrist:, x, y, z, qx, qy, qz, qw, leftFist: state
Right landmarks: x0,y0,z0,x1,y1,z1,...,x20,y20,z20
```

The PICO side runs a Unity app that reads PICO hand tracking and sends those two text messages over WiFi.

## 1. What PICO Provides

Official PICO reference: <https://developer-cn.picoxr.com/document/unity/hand-tracking/>

PICO Unity XR SDK exposes hand tracking through:

- `PXR_HandTracking.GetSettingState()`
- `PXR_HandTracking.GetActiveInputDevice()`
- `PXR_HandTracking.GetJointLocations(HandType.HandRight, ref jointLocations)`
- `PXR_HandTracking.GetAimState(HandType.HandRight, ref aimState)`

The PICO API gives 26 hand joints. LeFranX expects 21 MediaPipe-style landmarks, so `PicoLeUr5HandStreamer.cs` maps PICO joints to:

```text
0 wrist
1-4 thumb
5-8 index
9-12 middle
13-16 ring
17-20 little
```

## 2. Network Layout

Recommended PICO setup:

```text
PICO headset and robot computer on same WiFi/LAN
PICO Unity app connects to COMPUTER_IP:8000
Computer script runs with --no-adb --vr-port 8000
```

Unlike Quest USB mode, this does not use `adb reverse`.

## 3. Prepare the Computer

Find the computer IP on the same network as the PICO:

```bash
ip addr
```

Allow the TCP port if firewall is enabled:

```bash
sudo ufw allow 8000/tcp
```

Start a software-only smoke test first:

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --ur-stub \
  --dexh13-protocol STUB \
  --no-adb \
  --no-home \
  --vr-port 8000 \
  --verbose
```

For real hardware:

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

Keep `--no-adb`; PICO connects over WiFi directly.

## 4. Prepare Unity for PICO

1. Create or open a Unity Android XR project.
2. Import the PICO Unity Integration SDK / PICO Unity XR SDK.
3. Enable PICO XR plugin in XR Plug-in Management.
4. Enable hand tracking in the PICO project settings and on the headset.
5. Add an empty GameObject named `LeUr5PicoStreamer`.
6. Add `pico/unity/PicoLeUr5HandStreamer.cs` to the Unity project.
7. Attach `PicoLeUr5HandStreamer` to `LeUr5PicoStreamer`.
8. Set:

```text
Computer Host = your computer IP
Computer Port = 8000
Stream Hz = 30
```

Build and install the app to the PICO headset.

## 5. Run Order

1. Put PICO and computer on the same network.
2. Start the LeFranX computer-side script.
3. Launch the PICO Unity app.
4. Show your right hand to the headset.
5. Watch the computer logs for `tcp_connected`, `wrist_valid`, and `landmarks_valid`.

## 6. Coordinate Calibration

The PICO script sends PICO/Unity world coordinates. The existing UR5e VR processor currently applies the same VR-to-robot transform used for Quest:

```text
Robot X = VR Z
Robot Y = -VR X
Robot Z = VR Y
```

This is a starting point, not a site calibration. If moving your PICO-tracked hand forward moves the UR5e in the wrong direction, adjust the axis conversion in one of these places:

- Unity side: `ConvertUnityPositionForLeFranX()` in `PicoLeUr5HandStreamer.cs`
- Python side: `_compute_target_matrix()` in `src/lerobot/teleoperators/ur5e_vr/arm_ik_processor.py`

Change only one side at a time.

## 7. Troubleshooting

Check that the computer is listening:

```bash
ss -ltnp | grep 8000
```

Check PICO can reach the computer:

```bash
ping COMPUTER_IP
```

If the computer logs show no VR connection:

- confirm `Computer Host` in Unity is the computer IP, not `localhost`
- confirm firewall allows `8000/tcp`
- confirm both devices are on the same subnet
- confirm the LeFranX script was started before the PICO app

If connected but no hand data:

- enable PICO hand tracking in headset settings
- make sure controllers are not taking over active input
- keep hands in the headset tracking volume
- log `PXR_HandTracking.GetSettingState()` and `GetActiveInputDevice()`

If hand landmarks are valid but retargeting looks mirrored or rotated:

- first switch to geometry backend for a simpler sanity check:

  ```bash
  --dexh13-hand-backend geometry
  ```

- then tune the Unity coordinate conversion or Python VR-to-robot transform.
