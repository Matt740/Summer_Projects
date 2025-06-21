import gpiod
import time

CHIP_NAME = "gpiochip0"
STEPS_PER_REV = 16000
STEP_PER_DEG = STEPS_PER_REV / 360.0  # ≈ 44.44 steps/deg

class StepperMotor:
    def __init__(self, chip, step_pin, dir_pin, en_pin, consumer="stepper"):
        self.step_line = chip.get_line(step_pin)
        self.dir_line = chip.get_line(dir_pin)
        self.en_line = chip.get_line(en_pin)

        self.step_line.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self.dir_line.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self.en_line.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])  # disabled initially

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

def main():
    chip = gpiod.Chip(CHIP_NAME)
    motors = {
        1: StepperMotor(chip, 4, 3, 2),
        2: StepperMotor(chip, 22, 27, 17),
        3: StepperMotor(chip, 11, 9, 10)
    }

    print("Commands:")
    print("- 'motor <id>: <degrees>' to rotate")
    print("- 'enable <id>' to enable")
    print("- 'disable <id>' to disable")
    print("- 'exit' to quit")

    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "exit":
                break
            elif cmd.startswith("motor"):
                try:
                    parts = cmd.split(":")
                    motor_id = int(parts[0].split()[1])
                    degrees = float(parts[1])
                    if motor_id in motors:
                        print(f"Rotating motor {motor_id} by {degrees}°")
                        motors[motor_id].rotate_degrees(degrees)
                    else:
                        print("Invalid motor number")
                except Exception:
                    print("Invalid input format. Use 'motor <id>: <deg>'")
            elif cmd.startswith("enable"):
                try:
                    motor_id = int(cmd.split()[1])
                    if motor_id in motors:
                        motors[motor_id].manual_enable(True)
                    else:
                        print("Invalid motor number")
                except:
                    print("Invalid input. Use 'enable <id>'")
            elif cmd.startswith("disable"):
                try:
                    motor_id = int(cmd.split()[1])
                    if motor_id in motors:
                        motors[motor_id].manual_enable(False)
                    else:
                        print("Invalid motor number")
                except:
                    print("Invalid input. Use 'disable <id>'")
            else:
                print("Invalid command.")
    finally:
        chip.close()

if __name__ == "__main__":
    main()
