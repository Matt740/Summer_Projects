from PID_platform import PID, PIDGains, PlatformPID
from UpdatedBallTrackingLite import BallTrackerLite
from InverseKinematicsCalc import WristIKController
import numpy as np
import time

if __name__ == "__main__":
    # Example usage
    ctrl = WristIKController(L1=50, L2=285, Rb=116, Rp=116)
    q1, q2 = ctrl.compute_ik(pitch_deg=10, roll_deg=10, yaw_deg=0)

    for i in range(3):
        print(f"Limb {i+1}: q1 = {np.rad2deg(q1[i]):.2f} deg, q2 = {np.rad2deg(q2[i]):.2f} deg")