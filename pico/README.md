# PICO Headset Integration

This directory contains a PICO replacement path for the Quest-side VR data source.

The computer-side LeFranX scripts already listen for a simple TCP text protocol on `--vr-port` (default `8000`). The PICO side only needs to stream the same wrist and hand-landmark messages, so the UR5e + DexH13 Python code can stay unchanged.

Use this when you want:

```text
PICO headset + PICO Unity hand tracking
  -> WiFi TCP stream
  -> LeFranX VR message router
  -> UR5e EE control + DexH13 retargeting
```

Files:

- `unity/PicoLeUr5HandStreamer.cs`: Unity component that reads PICO right-hand tracking and streams LeFranX-compatible TCP messages.
- `PICO_CONNECTION.md`: complete setup and usage guide.

