# Copilot Instructions for CodeBase

## Project Overview
This repository controls a 3-axis ball-balancing platform using computer vision, PID control, inverse kinematics, and stepper motor drivers. The system is designed for a Raspberry Pi (or similar SBC) with direct GPIO control and a Pi Camera.

### Major Components
- `UpdatedBallTrackingLite.py`: Real-time ball detection using OpenCV and PiCamera2. Provides normalized (u, v) coordinates for control.
- `PID_platform.py`: Contains PID controller logic and a `PlatformPID` class to map ball position to platform angles.
- `MattsIK.py`: Implements the `ThreeRRSRobot` class for inverse kinematics, converting platform angles to motor angles.
- `stepper_controller.py`: Synchronous control of 3 stepper motors via GPIO using the `gpiod` library. Provides absolute angle tracking and synchronized motion.
- `MainTest.py`: Main integration script. Connects vision, control, kinematics, and actuation in a real-time loop.

## Data Flow
1. Ball position is read from the camera (`BallTrackerLite.read()` → `u`, `v`).
2. PID controller computes desired platform roll/pitch angles.
3. Inverse kinematics converts platform angles to motor angles.
4. Stepper controller moves motors to target angles.

## Key Patterns & Conventions
- **Normalized Coordinates**: Ball position is always normalized to [-1, 1] for both axes before entering the control loop.
- **Safety Checks**: Kinematics and PID enforce hardware limits (angle, height) and will raise exceptions if unsafe commands are issued.
- **Absolute Positioning**: Stepper controller tracks software angles; always call `set_pos_deg()` after homing or power-up to align software and hardware.
- **Synchronous Motion**: All three motors move together using a Bresenham-style scheduler for coordinated platform movement.
- **Configurable Parameters**: HSV thresholds, PID gains, and geometry are settable at runtime or via constructor arguments.

## Developer Workflows
- **Run Main Loop**: Execute `MainTest.py` to start the full system. Ensure hardware is connected and PiCamera2 is available.
- **Testing Vision**: Run `UpdatedBallTrackingLite.py` standalone to debug ball detection (see example at file bottom).
- **Tuning**: Adjust PID gains and HSV thresholds in code or via provided setters for best performance.
- **Hardware Alignment**: After power-up, use `MultiStepperController.set_pos_deg()` to align software state with physical zero.

## External Dependencies
- `picamera2`, `opencv-python`, `numpy`, `gpiod` (for GPIO control)
- Install dependencies via `pip install -r Old/requirements.txt` (if present) or manually as needed.

## Example Integration
See `MainTest.py` for the canonical integration pattern:
```python
tracker = BallTrackerLite(); tracker.start()
ctrl = PlatformPID(...)
robot = ThreeRRSRobot(...)
motor_ctrl = MultiStepperController()
while True:
    data = tracker.read()
    roll, pitch = ctrl.update(data["u"], data["v"], data["timestamp"])
    motor_angles, _ = robot.inverse_kinematics(h, roll, pitch)
    motor_ctrl.goto_rad(motor_angles)
```

## Platform-Specific Notes
- GPIO and camera code is intended for Linux SBCs (e.g., Raspberry Pi). Not portable to Windows without hardware abstraction.
- All angles are in degrees for stepper controller, radians elsewhere.

---

**For AI agents:**
- Always respect hardware safety limits and initialization order.
- Use the main loop in `MainTest.py` as the reference for new features or debugging.
- When adding new modules, follow the normalized data flow and absolute positioning conventions.
