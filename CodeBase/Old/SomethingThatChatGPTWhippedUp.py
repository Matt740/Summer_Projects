# ball_balance_pid.py
# Requirements: python3, gpiod (libgpiod), opencv-python
# On Raspberry Pi OS Bookworm: sudo apt install python3-opencv python3-libgpiod gpiod
# Then: python3 ball_balance_pid.py

import time
import math
import threading
from collections import deque

import cv2
import numpy as np
import gpiod

# =========================
# ====== CONFIG ZONE ======
# =========================

USE_THREE_MOTORS = False  # False = 2-motor tilt (X/Y). True = 3 motors at 120° around the plate.

# ---- CAMERA / VISION ----
CAM_INDEX = 0                  # Use 0 for default. If using PiCamera2 via OpenCV, this usually works once libcamera stack is bridged.
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 60
# HSV range for ORANGE ball (tune in your lighting!)
HSV_LOWER = np.array([5, 90, 80])    # TODO tune
HSV_UPPER = np.array([25, 255, 255]) # TODO tune
MIN_CONTOUR_AREA = 120               # ignore tiny noise blobs

# ---- CONTROL LOOP ----
CONTROL_HZ = 100.0             # PID update rate
PLATE_NEUTRAL_ANGLE_DEG = 0.0  # zero tilt
# Target = image center
X_SET = FRAME_WIDTH / 2
Y_SET = FRAME_HEIGHT / 2
PIXELS_TO_MM = 0.25            # TODO: mm per pixel at the plate plane (roughly; refine from calibration)
# Converts position error (mm) → desired plate angle (deg). Think of it as how aggressively you tilt for a given ball offset.
MM_TO_DEG = 0.03               # TODO: start small (0.02–0.08). Larger = more tilt per mm error.

# ---- PID GAINS (start conservative) ----
PID_KP = 0.9
PID_KI = 0.0
PID_KD = 0.12
PID_I_CLAMP = 10.0             # integral windup guard (deg)
DERIVATIVE_LPF_HZ = 15.0       # low-pass derivative filter cutoff (Hz), 0 to disable

# ---- MECHANICAL / STEPPER ----
CHIP_NAME = "gpiochip0"

# Motor steps: 200 step/rev * microsteps * gearbox ratio.
STEPS_PER_REV = 200
MICROSTEPS = 16               # TB6600 switches set to 1/16 per your note
GEAR_RATIO = 5.0              # TODO: set if gearbox present; 1.0 if direct drive
STEPS_PER_REV_EFF = STEPS_PER_REV * MICROSTEPS * GEAR_RATIO

# Screw pitch or linkage conversion from motor revolutions → linear height change at the plate joint (mm/rev).
# If you drive a leadscrew directly: mm_per_rev = pitch (e.g., 8 mm/rev).
# If you're using a linkage/arm, define angle-to-steps directly below instead.
MM_PER_REV_AT_JOINT = 8.0     # TODO: set correctly if using leadscrews. Otherwise ignore if using angle mapping.

# Plate geometry for 3‑motor mode (triangle, equal radii)
ARM_RADIUS_MM = 60.0          # radial distance from plate center to motor joint on plate
# Small-angle approximation: height ≈ radius * angle (in radians). Use this to convert desired pitch/roll → height at each joint.
# If you have 2‑motor mode: we map pitch to motor A, roll to motor B directly (simplified).

# ---- STEP LIMITS / MOTION SLEW ----
MAX_STEP_RATE = 2000           # steps/sec (per motor)
MAX_STEP_ACCEL = 8000          # steps/sec^2 (per motor)
STEP_PULSE_US = 2              # TB6600 needs short high pulses; datasheet says >2.2 µs typical → we’ll use a few µs

# GPIO pin map (BCM numbers) — EDIT for your wiring
# For 2-motor mode, M1 and M2 are used. For 3-motor mode, all three used.
MOTORS_CFG = [
    # step, dir, en
    (4, 3, 2),     # Motor 1: Pulse+, Dir+, Enable+
    (22, 27, 17),  # Motor 2
    (11, 9, 10),   # Motor 3 (only if USE_THREE_MOTORS = True)
]

# =========================
# ====== UTILITIES ========
# =========================

class PID:
    def __init__(self, kp, ki, kd, i_clamp=10.0, d_lpf_hz=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_clamp = abs(i_clamp)
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_time = None
        self.d_lpf_hz = d_lpf_hz
        self.d_state = 0.0  # filtered derivative

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_time = None
        self.d_state = 0.0

    def update(self, err, now=None):
        t = time.perf_counter() if now is None else now
        if self.prev_time is None:
            self.prev_time = t
            self.prev_err = err
            return self.kp * err

        dt = max(1e-5, t - self.prev_time)
        self.prev_time = t

        # Integral
        self.integral += err * dt
        self.integral = max(-self.i_clamp, min(self.integral, self.i_clamp))

        # Derivative
        raw_deriv = (err - self.prev_err) / dt
        self.prev_err = err
        if self.d_lpf_hz and self.d_lpf_hz > 0:
            # first-order low-pass on derivative
            rc = 1.0 / (2 * math.pi * self.d_lpf_hz)
            alpha = dt / (rc + dt)
            self.d_state = self.d_state + alpha * (raw_deriv - self.d_state)
            d_term = self.d_state
        else:
            d_term = raw_deriv

        return self.kp * err + self.ki * self.integral + self.kd * d_term


class StepperMotor:
    def __init__(self, chip, step_pin, dir_pin, en_pin, consumer="ballbal"):
        self.step_line = chip.get_line(step_pin)
        self.dir_line  = chip.get_line(dir_pin)
        self.en_line   = chip.get_line(en_pin)

        self.step_line.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self.dir_line.request( consumer=consumer, type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self.en_line.request(  consumer=consumer, type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])  # disabled (EN high if wired to Enable+)

        self.is_enabled = False
        self.lock = threading.Lock()

        # motion state
        self.position_steps = 0.0   # current (estimated) position
        self.target_steps   = 0.0   # commanded target
        self.vel_sps        = 0.0   # steps/sec

        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.running = True
        self.thread.start()

    def enable(self, on=True):
        with self.lock:
            self.en_line.set_value(0 if on else 1)  # TB6600 Enable active low (commonly)
            self.is_enabled = on

    def set_target(self, steps):
        with self.lock:
            self.target_steps = float(steps)

    def move_increment(self, dsteps):
        with self.lock:
            self.target_steps += float(dsteps)

    def _pulse(self):
        # minimal pulse for TB6600
        self.step_line.set_value(1)
        # busy-wait for ~STEP_PULSE_US microseconds
        end = time.perf_counter() + (STEP_PULSE_US / 1_000_000.0)
        while time.perf_counter() < end:
            pass
        self.step_line.set_value(0)

    def _worker(self):
        last_time = time.perf_counter()
        while self.running:
            now = time.perf_counter()
            dt = max(1e-4, now - last_time)
            last_time = now

            with self.lock:
                tgt = self.target_steps
                pos = self.position_steps
                vel = self.vel_sps
                enabled = self.is_enabled

            if not enabled:
                time.sleep(0.002)
                continue

            error = tgt - pos
            if abs(error) < 0.5 and abs(vel) < 1.0:
                # close enough; idle a bit
                time.sleep(0.001)
                continue

            # Plan velocity towards target with accel & max rate
            desired_vel = max(-MAX_STEP_RATE, min(MAX_STEP_RATE, error * 50.0))  # simple P on position to get a velocity target
            # accel limit
            dv = desired_vel - vel
            max_dv = MAX_STEP_ACCEL * dt
            if dv > max_dv: dv = max_dv
            if dv < -max_dv: dv = -max_dv
            vel += dv

            # integrate velocity; step discretely
            steps_to_take = vel * dt
            # take integer steps with correct direction
            n = int(abs(steps_to_take))
            direction = 1 if steps_to_take >= 0 else -1
            if n > 0:
                # set direction
                self.dir_line.set_value(0 if direction >= 0 else 1)
                for _ in range(n):
                    self._pulse()
                pos += direction * n

            with self.lock:
                self.position_steps = pos
                self.vel_sps = vel

            # small sleep to reduce CPU; stepping time dominates anyway
            time.sleep(0.0005)

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
        # disable outputs
        self.enable(False)
        self.step_line.release()
        self.dir_line.release()
        self.en_line.release()


class BallTracker:
    def __init__(self, cam_index=0, width=640, height=480, fps=60):
        self.cap = cv2.VideoCapture(cam_index, cv2.CAP_ANY)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open camera")

        self.width = width
        self.height = height

    def get_xy(self):
        ok, frame = self.cap.read()
        if not ok:
            return None, None, None

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
        mask = cv2.medianBlur(mask, 5)

        # Largest contour → centroid
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cx = cy = r = None
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) > MIN_CONTOUR_AREA:
                (x, y), radius = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = M["m10"] / M["m00"]
                    cy = M["m01"] / M["m00"]
                    r = radius

        # For visualization (optional)
        vis = frame.copy()
        if cx is not None:
            cv2.circle(vis, (int(cx), int(cy)), int(max(4, r or 4)), (0, 255, 0), 2)
        cv2.circle(vis, (int(X_SET), int(Y_SET)), 5, (255, 0, 0), -1)
        cv2.imshow("Ball / Mask", vis)
        cv2.waitKey(1)

        return cx, cy, r

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()

# =========================
# ====== KINEMATICS =======
# =========================

def deg_to_height_mm(pitch_deg, roll_deg, radius_mm):
    """For small angles: height at an edge joint = R * angle_rad projected onto its axis."""
    pitch_rad = math.radians(pitch_deg)
    roll_rad  = math.radians(roll_deg)
    # For 3-motor arrangement at 0°, 120°, 240° around +X axis
    angles = [0.0, 120.0 * math.pi/180.0, 240.0 * math.pi/180.0]
    heights = []
    for th in angles:
        # Unit direction vector on plate for that leg:
        ux = math.cos(th)
        uy = math.sin(th)
        # Height contribution ~ R * (pitch about X adds along +Y edge; roll about Y adds along +X)
        h = radius_mm * (pitch_rad * uy + roll_rad * ux)
        heights.append(h)
    return heights  # list of 3

def height_mm_to_steps(mm):
    revs = mm / MM_PER_REV_AT_JOINT
    return revs * STEPS_PER_REV_EFF

def angle_deg_to_steps_2motor(pitch_deg, roll_deg, scale_steps_per_deg):
    # In a 2-motor orthogonal layout, map pitch to Motor1 and roll to Motor2
    s1 = pitch_deg * scale_steps_per_deg
    s2 = roll_deg * scale_steps_per_deg
    return s1, s2

# =========================
# ======= MAIN APP ========
# =========================

def main():
    # Setup GPIO chip and motors
    chip = gpiod.Chip(CHIP_NAME)

    motors = []
    motor_pins = MOTORS_CFG[:3 if USE_THREE_MOTORS else 2]
    for (step, d, en) in motor_pins:
        m = StepperMotor(chip, step, d, en)
        motors.append(m)

    # Enable
    for m in motors:
        m.enable(True)

    # Home/zero (assumed already mechanically mid-stroke). If you have switches, add homing here.
    for m in motors:
        m.set_target(0.0)

    # PID for X and Y axes (ball position → desired plate tilt angle)
    pid_x = PID(PID_KP, PID_KI, PID_KD, i_clamp=PID_I_CLAMP, d_lpf_hz=DERIVATIVE_LPF_HZ)
    pid_y = PID(PID_KP, PID_KI, PID_KD, i_clamp=PID_I_CLAMP, d_lpf_hz=DERIVATIVE_LPF_HZ)

    tracker = BallTracker(CAM_INDEX, FRAME_WIDTH, FRAME_HEIGHT, FPS)

    # Precompute a scale if using 2‑motor angle mapping directly (deg → steps).
    # If you drive tilt directly via leadscrew heights, prefer the 3‑motor path.
    # Here we approximate: 1 deg tilt → height change ~ R * deg_rad at its joint, then mm → steps.
    deg_to_mm = ARM_RADIUS_MM * math.pi / 180.0
    steps_per_deg = height_mm_to_steps(deg_to_mm)

    dt = 1.0 / CONTROL_HZ
    print("Control running. Press Ctrl+C to exit.")
    try:
        while True:
            t0 = time.perf_counter()

            cx, cy, _ = tracker.get_xy()
            if cx is not None and cy is not None:
                # Position error in mm at the plate plane (approx)
                ex_mm = (cx - X_SET) * PIXELS_TO_MM
                ey_mm = (cy - Y_SET) * PIXELS_TO_MM

                # Convert mm error to desired tilt angles (deg). Positive ex_mm means ball is right of center → tilt plate right-to-left to roll ball left → sign choice below:
                pitch_cmd_deg = -pid_y.update(ey_mm * MM_TO_DEG, now=t0) + PLATE_NEUTRAL_ANGLE_DEG
                roll_cmd_deg  = -pid_x.update(ex_mm * MM_TO_DEG, now=t0) + PLATE_NEUTRAL_ANGLE_DEG

                # Limit maximum tilt for safety
                MAX_TILT = 8.0  # deg
                pitch_cmd_deg = max(-MAX_TILT, min(MAX_TILT, pitch_cmd_deg))
                roll_cmd_deg  = max(-MAX_TILT, min(MAX_TILT, roll_cmd_deg))

                if USE_THREE_MOTORS:
                    # 3-motor height solution
                    heights = deg_to_height_mm(pitch_cmd_deg, roll_cmd_deg, ARM_RADIUS_MM)  # 3 heights in mm
                    steps_targets = [height_mm_to_steps(h) for h in heights]
                    for i, m in enumerate(motors):
                        m.set_target(steps_targets[i])
                else:
                    # 2-motor simplified mapping
                    s1, s2 = angle_deg_to_steps_2motor(pitch_cmd_deg, roll_cmd_deg, steps_per_deg)
                    motors[0].set_target(s1)
                    motors[1].set_target(s2)
            else:
                # Lost ball: slowly return to neutral
                for m in motors:
                    m.set_target(0.0)

            # maintain loop timing
            t1 = time.perf_counter()
            sleep_time = dt - (t1 - t0)
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        tracker.release()
        for m in motors:
            m.stop()
        chip.close()

if __name__ == "__main__":
    main()
