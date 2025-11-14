# AntiTipping – Stabilization Library for FRC (TechMaker)

AntiTipping is a lightweight and efficient library developed by TechMaker Robotics to help FRC and FTC teams prevent their robots from tipping over during operation. It uses IMU-based pitch and roll readings to compute a proportional corrective velocity that can be added to your drive commands, improving stability under rapid acceleration, collisions, or uneven terrain.

## Features
- Detects excessive tilt using pitch and roll.
- Computes correction direction and magnitude automatically.
- Outputs a `ChassisSpeeds` object ready to merge with your drive system.
- Fully compatible with WPILib.
- Easy to configure: proportional gain (kP), tipping threshold, and maximum correction speed.

## Installation
Simply copy the `AntiTipping.java` file into your project under:

