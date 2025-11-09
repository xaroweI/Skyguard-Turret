Skyguard 🎯 Object-Tracking Turret

A real-time, camera-based object tracking system that uses OpenCV and an ESP32 to physically aim a pan/tilt turret at moving objects.

Features

Physical Tracking: Uses OpenCV to find moving objects and controls a pan/tilt turret via an ESP32.

Auto-Lock: Automatically locks onto the largest moving object in its view.

Hold Position: If a locked target is lost (e.g., it stops moving), the turret holds its last position.

Sweep Mode: When no target is locked, the turret sweeps 180° to 0° searching for one.

Usage

Configure skyguard_tracker.py:

Set the URL to your IP camera's video stream.

Set the SERIAL_PORT to your ESP32's COM port.

Run the application:

# Make sure your virtual environment is active
python skyguard_tracker.py


Two windows will open: Skyguard - Live (your camera) and Mask (the motion view).

Troubleshooting

Camera Issues: Verify the URL in skyguard_tracker.py is correct and your camera is on the same network.

Serial Issues: Verify the SERIAL_PORT is correct and no other program (like the Arduino Serial Monitor) is using it.

Servos Move Wrong Way: In skyguard_turret.ino, find the updateContinuousServo function and swap the + and - on the speedValue lines.