# stepper_controller.py
import gpiod
import time
from typing import Iterable, Dict

CHIP_NAME = "gpiochip0"

STEPS_PER_REV = 16000          # match your driver microstep setting (16k/rev per your working code)
STEP_PER_DEG  = STEPS_PER_REV / 360.0  # ≈ 44.44 steps/deg

# Motion + driver behavior
DEFAULT_DELAY_US  = 250        # composite scheduler delay (bigger = slower)
STEP_PULSE_US     = 2          # high-time for one STEP pulse
HOLD_AFTER_MOVE   = True      # True => keep coils energized after motion

# Mechanical limits (degrees)
# From your robot init: motor_limits=[(0.0, np.deg2rad(143))] * 3
MOTOR_MIN_DEG = 0.0
MOTOR_MAX_DEG = 143.0

class _StepperHW:
    """Low-level single-motor driver lines."""
    def __init__(self, chip: gpiod.Chip, step_pin: int, dir_pin: int, en_pin: int, consumer="stepper"):
        self.step_line = chip.get_line(step_pin)
        self.dir_line  = chip.get_line(dir_pin)
        self.en_line   = chip.get_line(en_pin)

        self.step_line.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self.dir_line.request(consumer=consumer,  type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        # TB6600: ENA LOW = enabled, HIGH = disabled
        self.en_line.request(consumer=consumer,   type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])

        self.enabled = False

    def enable(self, on: bool):
        self.en_line.set_value(0 if on else 1)
        self.enabled = on

    def set_dir(self, clockwise: bool):
        self.dir_line.set_value(1 if clockwise else 0)

    def step_pulse(self):
        self.step_line.set_value(1)
        time.sleep(STEP_PULSE_US / 1_000_000.0)
        self.step_line.set_value(0)

class MultiStepperController:
    """
    3-axis synchronous stepper controller with absolute angle tracking (degrees).
    Public API:
      - goto_rad(np.ndarray[3])  : radians
      - goto_deg(Iterable[3])    : degrees
      - set_pos_deg(Iterable[3]) : set software absolute without moving (e.g., after homing)
      - get_pos_deg() -> list[3]
    """
    def __init__(self,
                 chip_name: str = CHIP_NAME,
                 pins: Dict[int, Dict[str, int]] = None,
                 step_per_deg: float = STEP_PER_DEG):
        """
        pins: { axis_id: {"STEP": int, "DIR": int, "EN": int} }
              default matches your working code:
                1: STEP=4,  DIR=3,  EN=2
                2: STEP=22, DIR=27, EN=17
                3: STEP=11, DIR=9,  EN=10
        """
        self.chip = gpiod.Chip(chip_name)
        if pins is None:
            pins = {
                1: {"STEP": 4,  "DIR": 3,  "EN": 2},
                2: {"STEP": 22, "DIR": 27, "EN": 17},
                3: {"STEP": 11, "DIR": 9,  "EN": 10},
            }
        self.hw = {
            aid: _StepperHW(self.chip, p["STEP"], p["DIR"], p["EN"])
            for aid, p in pins.items()
        }
        self.step_per_deg = float(step_per_deg)
        self.pos_deg = {1: 0.0, 2: 0.0, 3: 0.0}  # software absolute angles

    # -------------- public helpers --------------
    def set_pos_deg(self, angles_deg: Iterable[float]):
        for aid, ang in zip((1,2,3), angles_deg):
            self.pos_deg[aid] = float(ang)

    def get_pos_deg(self):
        return [self.pos_deg[1], self.pos_deg[2], self.pos_deg[3]]

    # -------------- main APIs --------------
    def goto_rad(self, angles_rad: Iterable[float], delay_us: int = DEFAULT_DELAY_US):
        # radians -> degrees
        import math
        self.goto_deg([math.degrees(a) for a in angles_rad], delay_us=delay_us)

    def goto_deg(self, angles_deg: Iterable[float], delay_us: int = DEFAULT_DELAY_US):
        """
        Absolute move. Clamps to [MOTOR_MIN_DEG, MOTOR_MAX_DEG].
        Synchronous Bresenham-style scheduler so all 3 finish together.
        """
        targets = {}
        for aid, tgt in zip((1,2,3), angles_deg):
            # clamp to hard limits
            t = max(MOTOR_MIN_DEG, min(MOTOR_MAX_DEG, float(tgt)))
            targets[aid] = t

        # plan steps + directions
        plan = {}
        max_steps = 0
        for aid in (1,2,3):
            cur = self.pos_deg[aid]
            delta = targets[aid] - cur
            steps = int(round(abs(delta) * self.step_per_deg))
            cw = (delta > 0)  # define +deg => CW
            plan[aid] = {"steps": steps, "cw": cw}
            max_steps = max(max_steps, steps)

        if max_steps == 0:
            return  # nothing to do

        # enable + set direction
        for aid in (1,2,3):
            hw = self.hw[aid]
            if not hw.enabled:
                hw.enable(True)
            hw.set_dir(plan[aid]["cw"])

        # synchronized DDA
        acc = {1: 0, 2: 0, 3: 0}
        for i in range(max_steps):
            for aid in (1,2,3):
                steps_needed = plan[aid]["steps"]
                if steps_needed == 0:
                    continue
                acc[aid] += steps_needed
                if acc[aid] >= max_steps:
                    self.hw[aid].step_pulse()
                    acc[aid] -= max_steps
            time.sleep(delay_us / 1_000_000.0)

        # update software absolute
        for aid in (1,2,3):
            signed_steps = plan[aid]["steps"] if plan[aid]["cw"] else -plan[aid]["steps"]
            self.pos_deg[aid] += signed_steps / self.step_per_deg
            # numerically keep in bounds
            self.pos_deg[aid] = max(MOTOR_MIN_DEG, min(MOTOR_MAX_DEG, self.pos_deg[aid]))

        # de-energize if desired
        if not HOLD_AFTER_MOVE:
            for aid in (1,2,3):
                self.hw[aid].enable(False)

    def close(self):
        self.chip.close()
