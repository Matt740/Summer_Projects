import gpiod
import time
import threading

CHIP_NAME = "gpiochip0"
STEPS_PER_REV = 16000
STEP_PER_DEG = STEPS_PER_REV / 360.0  # ≈ 44.44 steps/deg

# PID controller helper
class PIDController:
    def __init__(self, kp, ki, kd, dt=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output


class StepperMotor:
    def __init__(self, chip, step_pin, dir_pin, en_pin,
                 kp=1.0, ki=0.0, kd=0.0, consumer="stepper"):
        # GPIO setup
        self.step_line = chip.get_line(step_pin)
        self.dir_line  = chip.get_line(dir_pin)
        self.en_line   = chip.get_line(en_pin)
        for line in (self.step_line, self.dir_line, self.en_line):
            line.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        # disabled = 1
        self.en_line.set_value(1)

        # state
        self.is_enabled = False
        self.position = 0.0  # current absolute position (degrees)
        # PID
        self.pid = PIDController(kp, ki, kd)

    def enable(self, on):
        self.en_line.set_value(0 if on else 1)
        self.is_enabled = on

    def set_direction(self, clockwise):
        self.dir_line.set_value(1 if clockwise else 0)

    def step(self, count, delay_us=200):
        for _ in range(count):
            self.step_line.set_value(1)
            time.sleep(delay_us / 1_000_000)
            self.step_line.set_value(0)
            time.sleep(delay_us / 1_000_000)

    def rotate_degrees(self, degrees):
        """ Relative move """
        if degrees == 0:
            return
        if not self.is_enabled:
            self.enable(True)
        self.set_direction(degrees > 0)
        steps = abs(int(degrees * STEP_PER_DEG))
        self.step(steps)
        # update tracked position
        self.position += degrees
        self.enable(False)

    def goto_position(self, target_deg, tolerance=0.1):
        """ PID-controlled absolute move """
        if not self.is_enabled:
            self.enable(True)
        while True:
            error = target_deg - self.position
            if abs(error) <= tolerance:
                break
            # PID output = desired delta degrees this cycle
            delta = self.pid.compute(error)
            # clamp to reasonable step size
            max_step_deg = 1.0
            delta = max(-max_step_deg, min(max_step_deg, delta))
            self.set_direction(delta > 0)
            steps = abs(int(delta * STEP_PER_DEG))
            if steps > 0:
                self.step(steps)
                self.position += (steps / STEP_PER_DEG) * (1 if delta > 0 else -1)
        self.enable(False)

    def manual_enable(self, on):
        self.enable(on)


def move_motors_concurrent(motors, moves, absolute=False):
    """
    Move multiple motors concurrently.
      - motors: dict {id: StepperMotor}
      - moves:  dict {id: degrees}
      - absolute: if True, use PID goto_position; else relative rotate_degrees
    """
    threads = []
    for m_id, deg in moves.items():
        if m_id in motors:
            target = motors[m_id].goto_position if absolute else motors[m_id].rotate_degrees
            t = threading.Thread(target=target, args=(deg,))
            threads.append(t)
        else:
            print(f"[Warning] invalid motor id {m_id}")
    for t in threads:
        t.start()
    for t in threads:
        t.join()


def main():
    chip = gpiod.Chip(CHIP_NAME)
    motors = {
        1: StepperMotor(chip,  4,  3,  2, kp=2.0, ki=0.1, kd=0.05),
        2: StepperMotor(chip, 22, 27, 17, kp=2.0, ki=0.1, kd=0.05),
        3: StepperMotor(chip, 11,  9, 10, kp=2.0, ki=0.1, kd=0.05)
    }

    # set initial absolute positions
    print("Enter initial absolute position (°) for each motor:")
    for m_id in motors:
        try:
            pos = float(input(f" Motor {m_id} initial pos: "))
            motors[m_id].position = pos
        except ValueError:
            motors[m_id].position = 0.0

    print("\nCommands:")
    print("  motor <id>:<deg>           → relative move")
    print("  motors: 1:deg,2:deg,...    → concurrent relative moves")
    print("  abs <id>:<deg>             → absolute PID move")
    print("  abs motors: 1:deg,2:deg..  → concurrent absolute moves")
    print("  home <id> <deg>            → set current pos without moving")
    print("  enable <id> / disable <id>")
    print("  exit")

    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "exit":
                break

            elif cmd.startswith("home"):
                parts = cmd.split()
                try:
                    m_id = int(parts[1])
                    deg  = float(parts[2])
                    motors[m_id].position = deg
                    print(f"Motor {m_id} homed to {deg}°")
                except:
                    print("Invalid. Use 'home <id> <deg>'")

            elif cmd.startswith("abs motors:"):
                try:
                    parts = cmd.split(":", 1)[1].split(",")
                    moves = {int(mid): float(deg) for mid, deg in (p.split(":") for p in parts)}
                    print(f"Absolute move concurrently to {moves}")
                    move_motors_concurrent(motors, moves, absolute=True)
                except:
                    print("Invalid. Use 'abs motors: 1:deg,2:deg,...'")

            elif cmd.startswith("abs "):
                try:
                    left, deg = cmd.split(":")
                    m_id = int(left.split()[1])
                    deg  = float(deg)
                    print(f"Moving motor {m_id} to {deg}° absolute")
                    motors[m_id].goto_position(deg)
                except:
                    print("Invalid. Use 'abs <id>:<deg>'")

            elif cmd.startswith("motors:"):
                try:
                    parts = cmd.split(":", 1)[1].split(",")
                    moves = {int(mid): float(deg) for mid, deg in (p.split(":") for p in parts)}
                    print(f"Relative moves concurrently: {moves}")
                    move_motors_concurrent(motors, moves, absolute=False)
                except:
                    print("Invalid. Use 'motors: 1:deg,2:deg,...'")

            elif cmd.startswith("motor"):
                try:
                    left, deg = cmd.split(":")
                    m_id = int(left.split()[1])
                    deg  = float(deg)
                    print(f"Relative move motor {m_id} by {deg}°")
                    motors[m_id].rotate_degrees(deg)
                except:
                    print("Invalid. Use 'motor <id>:<deg>'")

            elif cmd.startswith("enable") or cmd.startswith("disable"):
                parts = cmd.split()
                try:
                    m_id = int(parts[1])
                    on   = (parts[0] == "enable")
                    motors[m_id].manual_enable(on)
                except:
                    print("Invalid. Use 'enable <id>' / 'disable <id>'")

            else:
                print("Unknown command.")

    finally:
        chip.close()


if __name__ == "__main__":
    main()
