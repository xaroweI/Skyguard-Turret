SkyGuard Object-Tracking Turret

A real-time object tracking system built with OpenCV and an ESP32. This project was developed in relation to my application for the Coursera course Modern Robotics: Foundations of Robot Motion.

The system uses background subtraction to detect motion, then drives a pan/tilt turret to aim at the target via serial commands to an ESP32.

Features

- Auto-lock onto the largest moving object in view
- Hold last known position when a locked target stops moving
- Sweep mode when no target is present
- Interactive calibration to correct for mechanical offsets and camera FOV

Usage

1. In `skyguard_tracker.py`, set `SERIAL_PORT` to your ESP32's COM port.
2. Connect your webcam (index 0 by default via `CAMERA_SOURCE`).
3. Run:

```
python skyguard_tracker.py
```

Two windows will open: `SkyGuard - Live` (camera feed) and `Mask` (motion view).

Troubleshooting

Camera Issues: Make sure your webcam is connected and not in use by another application. If it's on a different device index, change `CAMERA_SOURCE` in `skyguard_tracker.py`.

Serial Issues: Verify `SERIAL_PORT` is correct and no other program (like the Arduino Serial Monitor) is using it.

Servos Move Wrong Way: In `skyguard_turret.ino`, find the `updateContinuousServo` function and swap the `+` and `-` on the `speedValue` lines.
