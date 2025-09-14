
# PID_platform.py
# This module provides PID control logic for the ball-balancing platform.
# It includes a general PID controller and a PlatformPID class for mapping ball position to platform angles.

import time
from dataclasses import dataclass
import numpy as np


# Stores the PID gain values for a controller
@dataclass
class PIDGains:
    kp: float  # Proportional gain
    ki: float  # Integral gain
    kd: float  # Derivative gain


# Implements a single-axis PID controller (Proportional-Integral-Derivative)
# Used for either roll or pitch control
class PID:
    """
    Single-axis PID controller with output limits, anti-windup, and filtered derivative.
    """
    def __init__(self, gains: PIDGains, out_min=-15.0, out_max=15.0, i_min=-10.0, i_max=10.0, d_alpha=0.2):
        self.gains = gains  # PID gain values
        self.out_min = out_min  # Minimum output value
        self.out_max = out_max  # Maximum output value
        self.i_min = i_min      # Minimum integral value
        self.i_max = i_max      # Maximum integral value
        self.d_alpha = d_alpha  # Derivative filter smoothing (0..1)
        self._reset_state()

    def _reset_state(self, integral=0.0):
        # Reset the internal state (integral, last measurement, filtered derivative)
        self._i = max(self.i_min, min(self.i_max, integral))
        self._last_meas = None
        self._d_filt = 0.0

    def reset(self, integral=0.0):
        # Public method to reset the controller state
        self._reset_state(integral)

    def update(self, error, meas, dt):
        # Compute the PID output given the error, measurement, and time step
        if dt <= 0:
            # If time step is zero, just use proportional term
            return np.clip(self.gains.kp * error, self.out_min, self.out_max)
        # Proportional term
        p = self.gains.kp * error
        # Integral term (with anti-windup clamp)
        self._i += self.gains.ki * error * dt
        self._i = np.clip(self._i, self.i_min, self.i_max)
        # Derivative term (on measurement, with low-pass filter)
        d_raw = 0.0
        if self._last_meas is not None:
            d_meas = (meas - self._last_meas) / dt
            d_raw = -self.gains.kd * d_meas
            self._d_filt = self.d_alpha * d_raw + (1 - self.d_alpha) * self._d_filt
        else:
            self._d_filt = 0.0
        self._last_meas = meas
        # Sum all terms and clamp to output limits
        out = p + self._i + self._d_filt
        return np.clip(out, self.out_min, self.out_max)


# Maps ball position (u, v) to platform roll/pitch angles using two PID controllers
class PlatformPID:
    """
    Maps normalized ball position (u, v) to platform roll/pitch angles using two PID controllers.
    """
    def __init__(self, gains_roll: PIDGains, gains_pitch: PIDGains, max_angle_deg=5.0, sign_roll=-1.0, sign_pitch=-1.0, d_alpha=0.2):
        # Convert max angle from degrees to radians
        max_angle = np.deg2rad(max_angle_deg)
        # Create a PID controller for roll and pitch
        self.roll_pid = PID(gains_roll, out_min=-max_angle, out_max=+max_angle, i_min=-max_angle, i_max=+max_angle, d_alpha=d_alpha)
        self.pitch_pid = PID(gains_pitch, out_min=-max_angle, out_max=+max_angle, i_min=-max_angle, i_max=+max_angle, d_alpha=d_alpha)
        # Setpoints (target ball position, usually center)
        self.u_sp = 0.0
        self.v_sp = 0.0
        # Sign corrections (to match your platform's geometry)
        self.sign_roll = sign_roll
        self.sign_pitch = sign_pitch
        self._t_last = None

    def set_setpoint(self, u_sp=0.0, v_sp=0.0):
        # Set the desired ball position (default: center)
        self.u_sp = u_sp
        self.v_sp = v_sp
        self.roll_pid.reset(0.0)
        self.pitch_pid.reset(0.0)
        self._t_last = None

    def update(self, u_meas, v_meas, t_now=None):
        # Compute the required roll and pitch angles to move the ball to the setpoint
        if t_now is None:
            t_now = time.time()
        if self._t_last is None:
            # First call: just initialize time, return zero angles
            self._t_last = t_now
            return 0.0, 0.0
        dt = t_now - self._t_last
        self._t_last = t_now
        # Calculate errors (setpoint - measured)
        e_u = self.u_sp - u_meas
        e_v = self.v_sp - v_meas
        # Update each PID controller
        roll_cmd = self.roll_pid.update(e_u, meas=u_meas, dt=dt)
        pitch_cmd = self.pitch_pid.update(e_v, meas=v_meas, dt=dt)
        # Apply sign corrections
        roll_out = self.sign_roll * roll_cmd
        pitch_out = self.sign_pitch * pitch_cmd
        return roll_out, pitch_out
