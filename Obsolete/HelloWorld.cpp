#include <gpiod.h>
#include <iostream>
#include <unistd.h>
using namespace std;

#define CHIP_NAME "gpiochip0"  // Replace if yours is different (check gpiodetect)
#define LINE_OFFSET 17         // GPIO 17

int main() { 
    const char* consumer = "blink_gpiod";

    // Open GPIO chip
    gpiod_chip* chip = gpiod_chip_open_by_name(CHIP_NAME);
    if (!chip) {
        cerr << "Failed to open GPIO chip\n";
        return 1;
    }

    // Get GPIO line
    gpiod_line* line = gpiod_chip_get_line(chip, LINE_OFFSET);
    if (!line) {
        cerr << "Failed to get GPIO line\n";
        gpiod_chip_close(chip);
        return 1;
    }

    // Request line as output
    if (gpiod_line_request_output(line, consumer, 0) < 0) {
        cerr << "Failed to request line as output\n";
        gpiod_chip_close(chip);
        return 1;
    }

    cout << "Blinking LED on GPIO 17 using libgpiod...\n";

    for (int i = 0; i < 10; ++i) {
        gpiod_line_set_value(line, 1);
        sleep(1);
        gpiod_line_set_value(line, 0);
        sleep(1);
    }

    gpiod_line_release(line);
    gpiod_chip_close(chip);
    return 0;
}
