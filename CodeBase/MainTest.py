from PID_platform import PID, PIDGains, PlatformPID
from UpdatedBallTrackingWithTuneAndDemo import BallTracker
import time
from MattsIK import ThreeRRSRobot
import numpy as np

# NEW: import our controller
from stepper_controller import MultiStepperController

if __name__ == "__main__":
    roll_gains  = PIDGains(kp=1.0, ki=0.0, kd=0.0)
    pitch_gains = PIDGains(kp=1.0, ki=0.0, kd=0.0)

    ctrl = PlatformPID(roll_gains, pitch_gains, max_angle_deg=10.0,
                       sign_roll=-1.0, sign_pitch=-1.0)

    d, e, f, g = 116, 116, 50, 219
    h = 213
    robot = ThreeRRSRobot(d, e, f, g,
                          h_min=100, h_max=400,
                          roll_max_deg=15,
                          pitch_max_deg=15,
                          motor_limits=[(np.deg2rad(90.0-20.42), np.deg2rad(180))] * 3)

    tracker = BallTracker()
    tracker.start()

    # NEW: make the stepper controller; you can set initial software angles here if needed
    motor_ctrl = MultiStepperController()
    motor_ctrl.set_pos_deg([90.0-20.42, 90.0-20.42, 90.0-20.42])   # align software to your physical zero

    try:
        while True:
            data = tracker.read()
            tracker.show_image()
            if data is None:
                continue

            u, v, t = data["u"], data["v"], data["timestamp"]
            roll_cmd, pitch_cmd = ctrl.update(u, v, t)  # radians

            # IK returns motor angles (radians). We’ll send those straight to motors.
            motor_angles, passive_angles = robot.inverse_kinematics(
                h, roll=roll_cmd, pitch=pitch_cmd
            )

            # DEBUG prints (optional)
            # print("Pitch (deg):", np.rad2deg(pitch_cmd))
            # print("Roll  (deg):", np.rad2deg(roll_cmd))
            # print("Motor Angles (deg):", np.rad2deg(motor_angles))
            # print("Passive Angles (deg):", np.rad2deg(passive_angles))

            # NEW: drive steppers to those absolute angles (clamped 0–143°)
            motor_ctrl.goto_rad(motor_angles, delay_us=250)

            # small pacing so we don't command faster than the motion can complete
            time.sleep(0.02)

    finally:
        motor_ctrl.close()
