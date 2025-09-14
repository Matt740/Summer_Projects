from PID_platform import PID, PIDGains, PlatformPID
from UpdatedBallTrackingLite import BallTrackerLite
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

    tracker = BallTrackerLite()
    tracker.start()

    # NEW: make the stepper controller; you can set initial software angles here if needed
    motor_ctrl = MultiStepperController()
    motor_ctrl.set_pos_deg([90.0-20.42, 90.0-20.42, 90.0-20.42])   # align software to your physical zero

    try:
        while True:
            data = tracker.read()

            if data is None:
                continue

            u, v, t = data["u"], data["v"], data["timestamp"]
            roll_cmd, pitch_cmd = ctrl.update(u, v, t)  # radians

            # IK returns motor angles (radians)
            motor_angles, passive_angles = robot.inverse_kinematics(
                h, roll=roll_cmd, pitch=pitch_cmd
            )

            motor_ctrl.goto_rad(motor_angles, delay_us=250)

            # small pacing so we don't overload drivers/UI
            time.sleep(0.02)


    finally:
        motor_ctrl.close()
