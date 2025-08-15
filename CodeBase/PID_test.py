from PID_platform import PID, PIDGains, PlatformPID
from UpdatedBallTrackingLite import BallTrackerLite
import time



if __name__ == "__main__":

    roll_gains  = PIDGains(kp=6.0, ki=0.0, kd=0.0)
    pitch_gains = PIDGains(kp=6.0, ki=0.0, kd=0.0)

    ctrl = PlatformPID(roll_gains, pitch_gains, max_angle_deg=10.0,
                       sign_roll=-1.0, sign_pitch=-1.0)

    tracker = BallTrackerLite()
    tracker.start()

    while True:
        data = tracker.read()
        if data is None:
             continue
        u, v, t = data["u"], data["v"], data["timestamp"]
        roll_deg, pitch_deg = ctrl.update(u, v, t)
        print(f"Roll: {roll_deg:.2f} deg, Pitch: {pitch_deg:.2f} deg")