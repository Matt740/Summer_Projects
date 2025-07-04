# Packages
import gpiod
import time

# Codebase# Packages
from Stepper import StepperMotor, CHIP_NAME
import BallTracker


def main():
    chip = gpiod.Chip(CHIP_NAME)
    # Initialize the stepper motors

    stepper1 = StepperMotor(chip, 4, 3, 2)  # Step, direction, enable
    stepper2 = StepperMotor(chip, 22, 27, 17)
    stepper3 = StepperMotor(chip, 11, 9, 10)

    stepper1.enable(True)
    stepper2.enable(True) 
    stepper3.enable(True)

    while True:
        stepper1.rotate_degrees(10)
        stepper2.rotate_degrees(10)
        stepper3.rotate_degrees(10)
        time.sleep(2)
        stepper1.rotate_degrees(-10)
        stepper2.rotate_degrees(-10)
        stepper3.rotate_degrees(-10)
        time.sleep(2) 




if __name__ == "__main__":
    main()