#include <gpiod.h>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <map>
using namespace std;

#define CHIP_NAME "gpiochip0"
#define STEPS_PER_REV 16000 // motor pulse/rev (*) micro step count (*)
#define STEP_PER_DEG (STEPS_PER_REV / 360.0)  // ≈ 44.44 steps/deg



class StepperMotor {
private:
    gpiod_chip* chip;
    gpiod_line* step_line;
    gpiod_line* dir_line;
    gpiod_line* en_line;
    const char* consumer;
    bool is_enabled = false;

public:
    StepperMotor(gpiod_chip* chip, int step_pin, int dir_pin, int en_pin, const char* consumer = "stepper")
        : chip(chip), consumer(consumer) {

        step_line = gpiod_chip_get_line(chip, step_pin);
        dir_line  = gpiod_chip_get_line(chip, dir_pin);
        en_line   = gpiod_chip_get_line(chip, en_pin);

        if (!step_line || !dir_line || !en_line)
            throw runtime_error("Failed to get one or more GPIO lines");

        if (gpiod_line_request_output(step_line, consumer, 0) < 0 ||
            gpiod_line_request_output(dir_line,  consumer, 0) < 0 ||
            gpiod_line_request_output(en_line,   consumer, 1) < 0)  // Disabled initially
            throw runtime_error("Failed to request one or more lines as output");
    }

    ~StepperMotor() {
        gpiod_line_release(step_line);
        gpiod_line_release(dir_line);
        gpiod_line_release(en_line);
    }

    void enable(bool on) {
        gpiod_line_set_value(en_line, on ? 0 : 1); // LOW = enabled
        is_enabled = on;
        cout << (on ? "Enabled" : "Disabled") << " motor\n";
    }

    void setDirection(bool clockwise) {
        gpiod_line_set_value(dir_line, clockwise ? 1 : 0);
    }

    void step(int count, int delay_us = 200) {
        for (int i = 0; i < count; ++i) {
            gpiod_line_set_value(step_line, 1);
            usleep(delay_us);
            gpiod_line_set_value(step_line, 0);
            usleep(delay_us);
        }
    }

    void rotateDegrees(double degrees) {
        if (degrees == 0) return;
        if (!is_enabled) enable(true);
        setDirection(degrees > 0);
        int steps = abs(int(degrees * STEP_PER_DEG));
        step(steps);
        enable(false);
    }

    void manualEnable(bool on) {
        enable(on);
    }
};

int main() {
    gpiod_chip* chip = gpiod_chip_open_by_name(CHIP_NAME);
    if (!chip) {
        cerr << "Failed to open GPIO chip\n";
        return 1;
    }

    try {
        map<int, StepperMotor*> motors = {
            {1, new StepperMotor(chip, 4, 3, 2)},
            {2, new StepperMotor(chip, 22, 27, 17)},
            {3, new StepperMotor(chip, 11, 9, 10)}
        };

        cout << "Commands:\n";
        cout << "- 'motor <id>: <degrees>' to rotate\n";
        cout << "- 'enable <id>' to enable\n";
        cout << "- 'disable <id>' to disable\n";
        cout << "- 'exit' to quit\n";

        string input;
        while (true) {
            cout << "> ";
            getline(cin, input);

            if (input == "exit") break;

            int motor_id;
            double deg;

            if (sscanf(input.c_str(), "motor %d: %lf", &motor_id, &deg) == 2) {
                if (motors.find(motor_id) != motors.end()) {
                    cout << "Rotating motor " << motor_id << " by " << deg << "°\n";
                    motors[motor_id]->rotateDegrees(deg);
                } else {
                    cerr << "Invalid motor number\n";
                }
            } else if (sscanf(input.c_str(), "enable %d", &motor_id) == 1) {
                if (motors.find(motor_id) != motors.end()) {
                    motors[motor_id]->manualEnable(true);
                } else {
                    cerr << "Invalid motor number\n";
                }
            } else if (sscanf(input.c_str(), "disable %d", &motor_id) == 1) {
                if (motors.find(motor_id) != motors.end()) {
                    motors[motor_id]->manualEnable(false);
                } else {
                    cerr << "Invalid motor number\n";
                }
            } else {
                cerr << "Invalid input\n";
            }
        }

        for (auto& [_, m] : motors) delete m;
        gpiod_chip_close(chip);
        return 0;
    }
    catch (const exception& e) {
        cerr << "Error: " << e.what() << "\n";
        gpiod_chip_close(chip);
        return 1;
    }
}
