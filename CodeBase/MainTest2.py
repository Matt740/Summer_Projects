from PID_platform import PID, PIDGains, PlatformPID
from UpdatedBallTrackingLite import BallTrackerLite
import time, cv2
from MattsIK import ThreeRRSRobot
import numpy as np

from stepper_controller import MultiStepperController

# ---------- PID tuner helpers ----------
_TUNER_WIN = "PID Tuner"
_SCALE = 1000  # slider unit -> gain = slider/_SCALE  (e.g., 750 -> 0.750)

def _make_pid_tuner(roll_init: PIDGains, pitch_init: PIDGains):
    cv2.namedWindow(_TUNER_WIN, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("Roll Kp",  _TUNER_WIN, int(roll_init.kp  * _SCALE), 5000, lambda x: None)
    cv2.createTrackbar("Roll Ki",  _TUNER_WIN, int(roll_init.ki  * _SCALE), 5000, lambda x: None)
    cv2.createTrackbar("Roll Kd",  _TUNER_WIN, int(roll_init.kd  * _SCALE), 5000, lambda x: None)
    cv2.createTrackbar("Pitch Kp", _TUNER_WIN, int(pitch_init.kp * _SCALE), 5000, lambda x: None)
    cv2.createTrackbar("Pitch Ki", _TUNER_WIN, int(pitch_init.ki * _SCALE), 5000, lambda x: None)
    cv2.createTrackbar("Pitch Kd", _TUNER_WIN, int(pitch_init.kd * _SCALE), 5000, lambda x: None)

def _get_pid_from_sliders():
    rkp = cv2.getTrackbarPos("Roll Kp",  _TUNER_WIN)/_SCALE
    rki = cv2.getTrackbarPos("Roll Ki",  _TUNER_WIN)/_SCALE
    rkd = cv2.getTrackbarPos("Roll Kd",  _TUNER_WIN)/_SCALE
    pkp = cv2.getTrackbarPos("Pitch Kp", _TUNER_WIN)/_SCALE
    pki = cv2.getTrackbarPos("Pitch Ki", _TUNER_WIN)/_SCALE
    pkd = cv2.getTrackbarPos("Pitch Kd", _TUNER_WIN)/_SCALE
    return PIDGains(rkp, rki, rkd), PIDGains(pkp, pki, pkd)

def _apply_gains(ctrl: PlatformPID, roll_g: PIDGains, pitch_g: PIDGains):
    """
    Be flexible with your PlatformPID API. We try common patterns.
    """
    # 1) explicit setter methods
    for meth in ("set_gains", "set_pid_gains"):
        if hasattr(ctrl, meth):
            try:
                getattr(ctrl, meth)(roll_g, pitch_g)  # (roll, pitch)
                return
            except TypeError:
                try:
                    getattr(ctrl, meth)(roll=roll_g, pitch=pitch_g)
                    return
                except Exception:
                    pass
    # 2) per-axis setters
    for meth in ("set_roll_gains",):
        if hasattr(ctrl, meth):
            getattr(ctrl, meth)(roll_g)
    for meth in ("set_pitch_gains",):
        if hasattr(ctrl, meth):
            getattr(ctrl, meth)(pitch_g)
    # 3) direct object attributes (ctrl.roll / ctrl.pitch / ctrl.roll_pid / ctrl.pitch_pid)
    for attr in ("roll", "roll_pid"):
        if hasattr(ctrl, attr):
            pid = getattr(ctrl, attr)
            for k in ("kp","ki","kd"):
                if hasattr(pid, k):
                    setattr(pid, k, getattr(roll_g, k))
    for attr in ("pitch", "pitch_pid"):
        if hasattr(ctrl, attr):
            pid = getattr(ctrl, attr)
            for k in ("kp","ki","kd"):
                if hasattr(pid, k):
                    setattr(pid, k, getattr(pitch_g, k))

# ---------- MAIN ----------
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

    motor_ctrl = MultiStepperController()
    motor_ctrl.set_pos_deg([90.0-20.42, 90.0-20.42, 90.0-20.42])

    # Create PID tuner sliders
    _make_pid_tuner(roll_gains, pitch_gains)

    try:
        while True:
            # 1) camera + key handling (keeps windows responsive)
            data = tracker.read()
            key = tracker.show_image()  # make sure show_image() returns cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            # 2) read sliders and apply gains live
            new_roll_g, new_pitch_g = _get_pid_from_sliders()
            _apply_gains(ctrl, new_roll_g, new_pitch_g)

            if data is None:
                continue

            # 3) control
            u, v, t = data["u"], data["v"], data["timestamp"]
            roll_cmd, pitch_cmd = ctrl.update(u, v, t)  # radians

            motor_angles, passive_angles = robot.inverse_kinematics(
                h, roll=roll_cmd, pitch=pitch_cmd
            )

            motor_ctrl.goto_rad(motor_angles, delay_us=250)

            time.sleep(0.02)  # pacing
    finally:
        motor_ctrl.close()
        cv2.destroyWindow(_TUNER_WIN)
