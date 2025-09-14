# test.py
import time
from stepper_controller import MultiStepperController

if __name__ == "__main__":
    # create controller (pins & settings from stepper_controller.py)
    ctrl = MultiStepperController()

    # align software positions to 0° (adjust if your hardware zero differs)
    ctrl.set_pos_deg([0.0, 0.0, 0.0])

    # adjustable speed: microseconds between scheduler "ticks"
    delay_us = 400   # bigger = slower, safer. Try 800, 400, 200

    presets = [
        [15.0, 15.0, 15.0],
        [25.0, 25.0, 25.0],
        [35.0, 35.0, 35.0],
        [0.0,  0.0,  0.0],   # return to home
    ]

    try:
        while True:
            for target in presets:
                print(f"Moving to {target} deg at delay {delay_us} us...")
                ctrl.goto_deg(target, delay_us=delay_us)
                print("Current positions:", ctrl.get_pos_deg())
                time.sleep(1.0)  # pause between moves
    except KeyboardInterrupt:
        print("Test stopped by user.")
    finally:
        ctrl.close()
