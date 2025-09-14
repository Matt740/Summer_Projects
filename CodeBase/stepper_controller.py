

# stepper_controller.py
# This module controls three stepper motors in sync using GPIO on a Raspberry Pi (or similar SBC).
# It provides absolute angle tracking and ensures all motors move together for coordinated motion.

import gpiod  # Library for GPIO control on Linux
import time
from typing import Iterable, Dict

# --- Hardware and motion constants ---
CHIP_NAME = "gpiochip0"  # Name of the GPIO chip (default for Pi)
STEPS_PER_REV = 16000    # Steps per full revolution (depends on your driver/microstepping)
STEP_PER_DEG = STEPS_PER_REV / 360.0  # Steps per degree
DEFAULT_DELAY_US = 250   # Delay between steps (microseconds)
STEP_PULSE_US = 2        # Duration of a single step pulse (microseconds)
HOLD_AFTER_MOVE = True   # Whether to keep motor coils energized after moving
MOTOR_MIN_DEG = 90.0 - 20.42  # Minimum allowed angle (degrees)
MOTOR_MAX_DEG = 180          # Maximum allowed angle (degrees)


class StepperHardware:
    """
    Handles the low-level GPIO control for a single stepper motor.
    Each motor has STEP, DIR, and EN (enable) pins.
    """
    def __init__(self, chip: gpiod.Chip, step_pin: int, dir_pin: int, en_pin: int, consumer="stepper"):
        # Get the GPIO lines for each pin
        self.step_line = chip.get_line(step_pin)
        self.dir_line = chip.get_line(dir_pin)
        self.en_line = chip.get_line(en_pin)
        # Request control of the lines (set as outputs)
        self.step_line.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self.dir_line.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        # ENA LOW = enabled, HIGH = disabled (for TB6600 driver)
        self.en_line.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])
        self.enabled = False

    def enable(self, on: bool):
        # Enable or disable the motor driver
        self.en_line.set_value(0 if on else 1)
        self.enabled = on

    def set_dir(self, clockwise: bool):
        # Set the direction of rotation (True = CW, False = CCW)
        self.dir_line.set_value(1 if clockwise else 0)

    def step_pulse(self):
        # Send a single step pulse to the motor
        self.step_line.set_value(1)
        time.sleep(STEP_PULSE_US / 1_000_000.0)
        self.step_line.set_value(0)


class StepperMotionPlanner:
    """
    Plans the number of steps and direction for each motor so all axes finish at the same time.
    Uses a Bresenham-style algorithm for synchronized motion.
    """
    def __init__(self, step_per_deg: float):
        self.step_per_deg = step_per_deg

    def plan_motion(self, current: Dict[int, float], targets: Dict[int, float]):
        # For each axis, calculate how many steps to move and in which direction
        plan = {}
        max_steps = 0
        for aid in (1, 2, 3):
            delta = targets[aid] - current[aid]
            steps = int(round(abs(delta) * self.step_per_deg))
            cw = (delta > 0)  # Clockwise if target > current
            plan[aid] = {"steps": steps, "cw": cw}
            max_steps = max(max_steps, steps)
        return plan, max_steps


class StepperStateTracker:
    """
    Keeps track of the current (software) angle of each motor in degrees.
    This allows absolute positioning even if the motors are moved in steps.
    """
    def __init__(self, initial_angles=None):
        if initial_angles is None:
            initial_angles = [0.0, 0.0, 0.0]
        # Store the current angle for each axis (1, 2, 3)
        self.pos_deg = {1: initial_angles[0], 2: initial_angles[1], 3: initial_angles[2]}

    def set_pos_deg(self, angles_deg: Iterable[float]):
        # Set the software angle for each axis (used after homing or manual alignment)
        for aid, ang in zip((1, 2, 3), angles_deg):
            self.pos_deg[aid] = float(ang)

    def get_pos_deg(self):
        # Get the current software angles as a list
        return [self.pos_deg[1], self.pos_deg[2], self.pos_deg[3]]

    def update_after_move(self, plan, step_per_deg):
        # After a move, update the software angles based on steps taken
        for aid in (1, 2, 3):
            signed_steps = plan[aid]["steps"] if plan[aid]["cw"] else -plan[aid]["steps"]
            self.pos_deg[aid] += signed_steps / step_per_deg
            # Clamp to allowed range
            self.pos_deg[aid] = max(MOTOR_MIN_DEG, min(MOTOR_MAX_DEG, self.pos_deg[aid]))


class MultiStepperController:
    """
    Controls three stepper motors in sync, using the hardware, motion planner, and state tracker classes.
    Provides high-level methods to move to absolute angles (in degrees or radians).
    """
    def __init__(self, chip_name=CHIP_NAME, pins=None, step_per_deg=STEP_PER_DEG):
        # Set up the GPIO chip and all three motors
        self.chip = gpiod.Chip(chip_name)
        if pins is None:
            pins = {
                1: {"STEP": 4, "DIR": 3, "EN": 2},
                2: {"STEP": 22, "DIR": 27, "EN": 17},
                3: {"STEP": 11, "DIR": 9, "EN": 10},
            }
        # Create hardware objects for each axis
        self.hw = {aid: StepperHardware(self.chip, p["STEP"], p["DIR"], p["EN"]) for aid, p in pins.items()}
        self.motion_planner = StepperMotionPlanner(step_per_deg)
        self.state_tracker = StepperStateTracker()
        self.step_per_deg = float(step_per_deg)

    def set_pos_deg(self, angles_deg: Iterable[float]):
        # Set the software angles for all axes (call after homing or manual alignment)
        self.state_tracker.set_pos_deg(angles_deg)

    def get_pos_deg(self):
        # Get the current software angles for all axes
        return self.state_tracker.get_pos_deg()

    def goto_rad(self, angles_rad: Iterable[float], delay_us: int = DEFAULT_DELAY_US):
        # Move to the given angles (in radians)
        import math
        self.goto_deg([math.degrees(a) for a in angles_rad], delay_us=delay_us)

    def goto_deg(self, angles_deg: Iterable[float], delay_us: int = DEFAULT_DELAY_US):
        # Move all axes to the given absolute angles (in degrees)
        # Clamp to allowed range
        targets = {}
        for aid, tgt in zip((1, 2, 3), angles_deg):
            t = max(MOTOR_MIN_DEG, min(MOTOR_MAX_DEG, float(tgt)))
            targets[aid] = t
        # Get current angles
        current = {aid: self.state_tracker.pos_deg[aid] for aid in (1, 2, 3)}
        # Plan the motion (steps and direction for each axis)
        plan, max_steps = self.motion_planner.plan_motion(current, targets)
        if max_steps == 0:
            return  # No movement needed
        # Enable motors and set direction
        for aid in (1, 2, 3):
            hw = self.hw[aid]
            if not hw.enabled:
                hw.enable(True)
            hw.set_dir(plan[aid]["cw"])
        # Synchronized stepping loop
        acc = {1: 0, 2: 0, 3: 0}
        for i in range(max_steps):
            for aid in (1, 2, 3):
                steps_needed = plan[aid]["steps"]
                if steps_needed == 0:
                    continue
                acc[aid] += steps_needed
                if acc[aid] >= max_steps:
                    self.hw[aid].step_pulse()
                    acc[aid] -= max_steps
            time.sleep(delay_us / 1_000_000.0)
        # Update software angles after move
        self.state_tracker.update_after_move(plan, self.step_per_deg)
        # Optionally de-energize motors
        if not HOLD_AFTER_MOVE:
            for aid in (1, 2, 3):
                self.hw[aid].enable(False)

    def close(self):
        # Release the GPIO chip (cleanup)
        self.chip.close()
