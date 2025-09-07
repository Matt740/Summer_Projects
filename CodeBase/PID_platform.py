import time
from dataclasses import dataclass
import numpy as np

@dataclass
class PIDGains: # Something that stores the PID gains
    kp: float
    ki: float
    kd: float

class PID:  # PID class, one for each axis of rotation
    """
    Single-axis PID with:
      - output limits (deg or whatever you choose)
      - integral windup clamp
      - derivative-on-measurement with simple low-pass filter
    Call update(error, dt) each cycle.
    """
    def __init__(self,
                 gains: PIDGains,
                 out_min: float = -15.0, # max and min platform angles
                 out_max: float = +15.0,
                 i_min: float = -10.0, # Max and min integral terms
                 i_max: float = +10.0,
                 d_alpha: float = 0.2  # 0..1 (higher = less filtering), 
                 ):
        self.g = gains
        self.out_min = out_min
        self.out_max = out_max
        self.i_min = i_min
        self.i_max = i_max
        self.d_alpha = d_alpha # how much to filter derivative term

        self._i = 0.0
        self._last_meas = None
        self._d_filt = 0.0

    def reset(self, integral: float = 0.0): # Reset integral and derivative state
        self._i = max(self.i_min, min(self.i_max, integral))
        self._last_meas = None
        self._d_filt = 0.0

    def update(self, error: float, meas: float, dt: float) -> float:
        """
        error = setpoint - measurement   (you compute upstream)
        meas  = current measurement (for derivative-on-measurement)
        dt    = seconds since last update
        returns: control output (same units as out_min/out_max)
        """
        if dt <= 0:
            # Guard: Just in case time step is zero.
            p = self.g.kp * error
            out = max(self.out_min, min(self.out_max, p))
            return out

        # P
        p = self.g.kp * error

        # I with clamp (anti-windup)
        self._i += self.g.ki * error * dt
        self._i = max(self.i_min, min(self.i_max, self._i))

        # D (on measurement): d = -kd * d(meas)/dt
        d_raw = 0.0
        if self._last_meas is not None:
            d_meas = (meas - self._last_meas) / dt
            d_raw = -self.g.kd * d_meas
            # low-pass filter derivative term (moving average)
            self._d_filt = (self.d_alpha * d_raw) + ((1 - self.d_alpha) * self._d_filt)
        else:
            self._d_filt = 0.0

        self._last_meas = meas

        out = p + self._i + self._d_filt
        # Output clamp, contrains limits to max and min angles

                # Clip to safe ranges
        out = np.clip(out, self.out_min, self.out_max)

        return out # Return the output value in radians

class PlatformPID:
    """
    Maps ball position -> platform angles.
    Inputs should be normalized offsets:
      u in [-1..+1]: right positive, left negative  (x-axis)
      v in [-1..+1]: down  positive, up   negative  (y-axis)
    Outputs (degrees):
      roll  (about x-axis, right-edge down is +)
      pitch (about y-axis, front-edge down is +)
    Adjust signs if your mechanics differ.
    """
    def __init__(self,
                 gains_roll: PIDGains,
                 gains_pitch: PIDGains,
                 max_angle_deg: float = 10.0, # Internally use radians, externally degrees
                 sign_roll: float = -1.0,   # flip to match your platform
                 sign_pitch: float = -1.0,  # flip to match your platform
                 d_alpha: float = 0.2):
        
        # convert to radians
        max_angle = np.deg2rad(max_angle_deg) 

        # one PID for each axis

        self.roll_pid = PID(gains_roll, out_min=-max_angle, out_max=+max_angle,
                            i_min=-max_angle, i_max=+max_angle, d_alpha=d_alpha)
        self.pitch_pid = PID(gains_pitch, out_min=-max_angle, out_max=+max_angle,
                             i_min=-max_angle, i_max=+max_angle, d_alpha=d_alpha)

        # setpoints (target center)
        self.u_sp = 0.0
        self.v_sp = 0.0

        self.sign_roll = sign_roll
        self.sign_pitch = sign_pitch

        self._t_last = None
        self._u_meas_last = None
        self._v_meas_last = None

    def set_setpoint(self, u_sp: float = 0.0, v_sp: float = 0.0): # Resets and sets new setpoint
        self.u_sp = u_sp
        self.v_sp = v_sp
        self.roll_pid.reset(0.0)
        self.pitch_pid.reset(0.0)
        self._t_last = None
        self._u_meas_last = None
        self._v_meas_last = None

    def update(self, u_meas: float, v_meas: float, t_now: float = None):
        """
        Provide current normalized position (u, v).
        Returns: (roll_deg, pitch_deg)
        """
        if t_now is None:
            t_now = time.time()

        if self._t_last is None:
            self._t_last = t_now
            self._u_meas_last = u_meas
            self._v_meas_last = v_meas
            # first call: no dt yet, return 0 angles
            return 0.0, 0.0

        dt = t_now - self._t_last
        self._t_last = t_now

        # Errors (setpoint - measurement)
        e_u = self.u_sp - u_meas
        e_v = self.v_sp - v_meas

        # Roll responds to u (x-axis), Pitch responds to v (y-axis)
        roll_cmd  = self.roll_pid.update(e_u, meas=u_meas, dt=dt)
        pitch_cmd = self.pitch_pid.update(e_v, meas=v_meas, dt=dt)

        # Sign corrections to match your platform geometry & motor wiring
        roll_out  = self.sign_roll  * roll_cmd
        pitch_out = self.sign_pitch * pitch_cmd

        return roll_out, pitch_out # Return the output values in radians


# ---------- Example wiring with your BallTrackerLite ----------
if __name__ == "__main__":
    # Example gains (start small; tune on hardware)
    roll_gains  = PIDGains(kp=6.0, ki=0.0, kd=0.8)
    pitch_gains = PIDGains(kp=6.0, ki=0.0, kd=0.8)

    ctrl = PlatformPID(roll_gains, pitch_gains, max_angle_deg=10.0,
                       sign_roll=-1.0, sign_pitch=-1.0)

    # Suppose you have: from tracker import BallTrackerLite
    # tracker = BallTrackerLite(); tracker.start()
    # while True:
    #     res = tracker.read()
    #     if res is None:
    #         continue
    #     u, v = res["u"], res["v"]
    #     roll_deg, pitch_deg = ctrl.update(u, v, t_now=res["timestamp"])
    #     # send roll_deg/pitch_deg to your actuators here
