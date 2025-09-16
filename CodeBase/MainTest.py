from collections import deque

# Modular imports
from PID_platform import PIDGains, PlatformPID
from UpdatedBallTrackingLite import BallTrackerLite
from MattsIK import ThreeRRSRobot
from stepper_controller import MultiStepperController
import time
import numpy as np

if __name__ == "__main__":
    # --- PID setup ---
    roll_gains = PIDGains(kp=1.0, ki=0.0, kd=0.0)
    pitch_gains = PIDGains(kp=1.0, ki=0.0, kd=0.0)
    # TEMP: Match roll gains to pitch gains for quick tuning
    roll_gains = pitch_gains
    ctrl = PlatformPID(roll_gains, pitch_gains, max_angle_deg=10.0, sign_roll=-1.0, sign_pitch=-1.0)

    # --- Kinematics setup ---
    d, e, f, g = 116, 116, 50, 219
    h = 213
    robot = ThreeRRSRobot(
        d, e, f, g,
        h_min=100, h_max=400,
        roll_max_deg=15,
        pitch_max_deg=15,
        motor_limits=[(np.deg2rad(90.0-20.42), np.deg2rad(180))] * 3
    )

    # --- Vision setup ---
    tracker = BallTrackerLite()
    tracker.start()

    # --- Stepper controller setup ---
    motor_ctrl = MultiStepperController()
    motor_ctrl.set_pos_deg([90.0-20.42, 90.0-20.42, 90.0-20.42])  # align software to your physical zero

    # Use a deque to store the last 3 valid tracker results
    recent_data = deque(maxlen=3)
    try:
        while True:
            data = tracker.read()
            if data is None:
                continue
            recent_data.append(data)
            filtered = BallTrackerLite.filter_recent_data(list(recent_data))
            u, v, t = filtered["u"], filtered["v"], filtered["timestamp"]
            roll_cmd, pitch_cmd = ctrl.update(u, v, t)
            motor_angles, _ = robot.inverse_kinematics(h, roll=roll_cmd, pitch=pitch_cmd)
            motor_ctrl.goto_rad(motor_angles, delay_us=250)
            time.sleep(0.02)
    finally:
        motor_ctrl.close()
