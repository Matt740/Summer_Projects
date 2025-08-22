# file: run_control.py
import numpy as np
from PID_platform import PID, PIDGains   # your PID controller code
from MattsIK import ThreeRRSRobot       # your IK code

# ---------------- Setup ----------------
# PID gains (example values, tune as needed)
pitch_pid = PID(PIDGains(kp=0.8, ki=0.05, kd=0.1),
                out_min=-np.deg2rad(15), out_max=np.deg2rad(15))  # limit ±15°
roll_pid  = PID(PIDGains(kp=0.8, ki=0.05, kd=0.1),
                out_min=-np.deg2rad(15), out_max=np.deg2rad(15))

# Robot geometry
d, e, f, g = 116, 116, 50, 219
robot = ThreeRRSRobot(d, e, f, g,
                      h_min=100, h_max=400,
                      roll_max=np.deg2rad(15),
                      pitch_max=np.deg2rad(15),
                      motor_limits=[(0.0, np.deg2rad(143))] * 3)

# ---------------- Control Loop Example ----------------
if __name__ == "__main__":
    # Example "ball error" inputs (replace with real tracking data)
    x_error = -20   # ball offset in x (pixels or mm)
    y_error = 10    # ball offset in y

    dt = 0.02  # 20 ms loop

    # Update PID → platform pitch/roll
    pitch_cmd = pitch_pid.update(y_error, dt)   # error in y controls pitch
    roll_cmd  = roll_pid.update(x_error, dt)    # error in x controls roll

    # Compute motor angles using IK
    h = 213   # fixed platform height (can adjust if needed)
    try:
        motor_angles, passive_angles = robot.inverse_kinematics(h,
                                                                roll=roll_cmd,
                                                                pitch=pitch_cmd)
        print("Pitch command (deg):", np.rad2deg(pitch_cmd))
        print("Roll command  (deg):", np.rad2deg(roll_cmd))
        print("Motor Angles (deg):", np.rad2deg(motor_angles))
        print("Passive Angles (deg):", np.rad2deg(passive_angles))

    except ValueError as e:
        print("SAFETY ERROR:", e)
