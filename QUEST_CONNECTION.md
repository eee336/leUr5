# Meta Quest Connection Guide

This guide explains how to connect a Meta Quest headset to the robot control computer for UR5e + DexH13 VR teleoperation.

The recommended setup is:

```text
Quest VR app
  -> USB cable
  -> adb reverse tcp:8000
  -> computer localhost:8000
  -> LeFranX VR message router
  -> UR5e + DexH13 teleoperation
```

## 1. Prepare Quest

On the Quest headset:

1. Enable Developer Mode for the headset.
2. Install the matching Quest VR teleoperation app.
3. Connect the Quest to the computer with a USB-C cable.
4. Put on the headset and allow USB debugging / RSA authorization when prompted.

## 2. Install ADB

Ubuntu:

```bash
sudo apt update
sudo apt install android-tools-adb -y
```

macOS:

```bash
brew install android-platform-tools
```

Check that the Quest is visible:

```bash
adb devices
```

Expected output:

```text
List of devices attached
1WMHHxxxxxxxx    device
```

If it says `unauthorized`, put on the headset and accept the debugging prompt.

## 3. Configure USB Port Forwarding

The project scripts configure ADB reverse automatically by default. Do not pass `--no-adb` unless you want to manage networking yourself.

Manual command:

```bash
adb reverse tcp:8000 tcp:8000
adb reverse --list
```

Expected output:

```text
1WMHHxxxxxxxx tcp:8000 tcp:8000
```

This means the Quest app can connect to its own `localhost:8000`, and ADB forwards that traffic to the computer's `localhost:8000`.

## 4. Start Computer-Side Teleoperation

Start the LeFranX script first:

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --robot-ip 192.168.1.10 \
  --dexh13-protocol SDK \
  --dexh13-hand-port /dev/ttyUSB0 \
  --dexh13-camera-port none \
  --dexh13-hand-backend retargeting \
  --vr-port 8000
```

For software-only startup testing:

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --ur-stub \
  --dexh13-protocol STUB \
  --no-home \
  --vr-port 8000
```

## 5. Start Quest App

After the computer-side script is running, open the Quest VR teleoperation app.

The app should stream:

- wrist position
- wrist quaternion
- 21 hand landmarks
- tracking validity/status fields

The computer script consumes wrist data for UR5e EE control and hand landmarks for DexH13 retargeting.

## 6. Verify Data Flow

In another terminal:

```bash
adb devices
adb reverse --list
```

If the computer-side script has verbose logging enabled:

```bash
--verbose
```

you should see VR connection/status messages.

## 7. Troubleshooting

Restart ADB:

```bash
adb kill-server
adb start-server
adb devices
adb reverse tcp:8000 tcp:8000
```

If the device is missing:

```bash
adb devices
```

Then reconnect the USB cable and accept the Quest debugging prompt.

If the device shows `unauthorized`, put on the headset and approve USB debugging.

If the script says there is no VR data:

```bash
adb reverse --list
```

If no `tcp:8000 tcp:8000` entry appears, run:

```bash
adb reverse tcp:8000 tcp:8000
```

If port `8000` is already in use on the computer, use another port consistently on both sides:

```bash
python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
  --vr-port 8001 \
  --ur-stub \
  --dexh13-protocol STUB
```

and:

```bash
adb reverse tcp:8001 tcp:8001
```

## 8. WiFi Alternative

USB + ADB reverse is recommended for first setup.

WiFi mode is possible if the Quest app can connect directly to the computer IP. In that case:

1. Put Quest and computer on the same network.
2. Find the computer IP:

   ```bash
   ip addr
   ```

3. Allow the TCP port through the firewall:

   ```bash
   sudo ufw allow 8000/tcp
   ```

4. Configure the Quest app to connect to:

   ```text
   COMPUTER_IP:8000
   ```

5. Start the script with:

   ```bash
   python scripts/ur5e_dexh13/ur5e_dexh13_vr_teleoperator.py \
     --vr-port 8000 \
     --no-adb \
     --robot-ip 192.168.1.10 \
     --dexh13-protocol SDK \
     --dexh13-hand-port /dev/ttyUSB0
   ```

Use WiFi only after USB mode works, because it adds router, firewall, and IP configuration variables.
