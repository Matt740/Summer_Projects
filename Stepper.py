import gpiod
import time
import threading

CHIP_NAME = "gpiochip0"
STEPS_PER_REV = 16000
STEP_PER_DEG = STEPS_PER_REV / 360.0  # ≈ 44.44 steps/deg

class StepperMotor:
    def __init__(self, chip, step_pin, dir_pin, en_pin, consumer="stepper"):
        self.step_line = chip.get_line(step_pin)
        self.dir_line  = chip.get_line(dir_pin)
        self.en_line   = chip.get_line(en_pin)

        self.step_line.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self.dir_line.request(consumer=consumer,  type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self.en_line.request(consumer=consumer,   type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])  # disabled initially

        self.is_enabled = False

    def enable(self, on):
        self.en_line.set_value(0 if on else 1)
        self.is_enabled = on
        print(f"{'Enabled' if on else 'Disabled'} motor")

    def set_direction(self, clockwise):
        self.dir_line.set_value(1 if clockwise else 0)

    def step(self, count, delay_us=200):
        for _ in range(count):
            self.step_line.set_value(1)
            time.sleep(delay_us / 1_000_000)
            self.step_line.set_value(0)
            time.sleep(delay_us / 1_000_000)

    def rotate_degrees(self, degrees):
        if degrees == 0:
            return
        if not self.is_enabled:
            self.enable(True)
        self.set_direction(degrees > 0)
        steps = abs(int(degrees * STEP_PER_DEG))
        self.step(steps)
        self.enable(False)

    def manual_enable(self, on):
        self.enable(on)


def move_motors_concurrent(motors, moves):
    """
    Rotate multiple motors concurrently.
      - motors: dict {id: StepperMotor}
      - moves:  dict {id: degrees}
    """
    threads = []
    for m_id, deg in moves.items():
        if m_id in motors:
            t = threading.Thread(
                target=motors[m_id].rotate_degrees,
                args=(deg,)
            )
            threads.append(t)
        else:
            print(f"[Warning] invalid motor id {m_id}")
    # start all threads
    for t in threads:
        t.start()
    # wait for all to finish
    for t in threads:
        t.join()


def main():
    chip = gpiod.Chip(CHIP_NAME)
    motors = {
        1: StepperMotor(chip,  4,  3,  2),
        2: StepperMotor(chip, 22, 27, 17),
        3: StepperMotor(chip, 11,  9, 10)
    }

    print("Commands:")
    print("  motor <id>:<deg>         → rotate one motor")
    print("  motors: 1:deg,2:deg,3:deg → rotate multiple motors concurrently")
    print("  enable <id> / disable <id>")
    print("  exit")

    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "exit":
                break

            elif cmd.startswith("motors:"):
                # e.g. "motors: 1:90,2:-45,3:30"
                try:
                    parts = cmd.split(":", 1)[1].split(",")
                    moves = {int(mid): float(deg) for mid, deg in (p.split(":") for p in parts)}
                    print(f"Rotating motors concurrently: {moves}")
                    move_motors_concurrent(motors, moves)
                except Exception:
                    print("Invalid format. Use 'motors: 1:deg,2:deg,3:deg'")

            elif cmd.startswith("motor"):
                # e.g. "motor 2:45"
                try:
                    left, deg = cmd.split(":")
                    m_id = int(left.split()[1])
                    deg  = float(deg)
                    print(f"Rotating motor {m_id} by {deg}°")
                    motors[m_id].rotate_degrees(deg)
                except Exception:
                    print("Invalid input. Use 'motor <id>:<deg>'")

            elif cmd.startswith("enable") or cmd.startswith("disable"):
                # e.g. "enable 1"
                parts = cmd.split()
                try:
                    m_id = int(parts[1])
                    on   = (parts[0] == "enable")
                    motors[m_id].manual_enable(on)
                except Exception:
                    print("Invalid command. Use 'enable <id>' or 'disable <id>'")

            else:
                print("Unknown command.")

    finally:
        chip.close()


if __name__ == "__main__":
    main()
