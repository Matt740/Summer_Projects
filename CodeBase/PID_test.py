from PID_platform import PID, PIDGains, PlatformPID
from UpdatedBallTrackingLite import BallTrackerLite
import time
from MattsIK import ThreeRRSRobot       # your IK code
import numpy as np

if __name__ == "__main__":

    roll_gains  = PIDGains(kp=3.0, ki=0.0, kd=0.0)
    pitch_gains = PIDGains(kp=3.0, ki=0.0, kd=0.0)

    ctrl = PlatformPID(roll_gains, pitch_gains, max_angle_deg=10.0,
                       sign_roll=-1.0, sign_pitch=-1.0)
    d, e, f, g = 116, 116, 50, 219
    h = 213
    robot = ThreeRRSRobot(d, e, f, g,
                      h_min=100, h_max=400,
                      roll_max_deg = 15,
                      pitch_max_deg= 15,
                      motor_limits=[(0.0, np.deg2rad(143))] * 3)
    tracker = BallTrackerLite()
    tracker.start()

    while True:
        data = tracker.read()
        if data is None:
             continue
        u, v, t = data["u"], data["v"], data["timestamp"]
        roll_cmd, pitch_cmd = ctrl.update(u, v, t) # in radians
        motor_angles, passive_angles = robot.inverse_kinematics(h,
                                                                roll=roll_cmd,
                                                                pitch=pitch_cmd)
        print("Pitch command (deg):", np.rad2deg(pitch_cmd))
        print("Roll command  (deg):", np.rad2deg(roll_cmd))
        print("Motor Angles (deg):", np.rad2deg(motor_angles))
        print("Passive Angles (deg):", np.rad2deg(passive_angles))