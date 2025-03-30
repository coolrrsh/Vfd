#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>

#define GPIO_CHIP "gpiochip0"
#define GPIO_PIN 17  // Any GPIO
#define PWM_FREQ 50  // Hz
#define DUTY_CYCLE 50  // %

int main() {
    struct gpiod_chip *chip;
    struct gpiod_line *line;

    chip = gpiod_chip_open_by_name(GPIO_CHIP);
    if (!chip) {
        perror("Error opening chip");
        return 1;
    }

    line = gpiod_chip_get_line(chip, GPIO_PIN);
    if (!line) {
        perror("Error getting line");
        gpiod_chip_close(chip);
        return 1;
    }

    if (gpiod_line_request_output(line, "pwm-example", 0) < 0) {
        perror("Error setting output");
        gpiod_line_release(line);
        gpiod_chip_close(chip);
        return 1;
    }

    int period_us = 1000000 / PWM_FREQ;
    int high_time_us = (period_us * DUTY_CYCLE) / 100;

    while (1) {
        gpiod_line_set_value(line, 1);
        usleep(high_time_us);
        gpiod_line_set_value(line, 0);
        usleep(period_us - high_time_us);
    }

    gpiod_line_release(line);
    gpiod_chip_close(chip);
    return 0;
}
