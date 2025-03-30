#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <stdint.h>

#define GPIO_CHIP "gpiochip0"  // Default GPIO chip on Pi
#define GPIO_LINE_OFFSET 17    // BCM GPIO17 (physical pin 11)
#define CONSUMER "pwm-reader"

struct pwm_measurement {
    uint64_t high_time_ns;
    uint64_t low_time_ns;
    uint64_t period_ns;
    double duty_cycle;
    double frequency_hz;
};

int main() {
    struct gpiod_chip *chip;
    struct gpiod_line *line;
    int ret;
    
    chip = gpiod_chip_open_by_name(GPIO_CHIP);
    if (!chip) {
        fprintf(stderr, "Error opening chip: %s\n", strerror(errno));
        return 1;
    }

    line = gpiod_chip_get_line(chip, GPIO_LINE_OFFSET);
    if (!line) {
        fprintf(stderr, "Error getting line: %s\n", strerror(errno));
        gpiod_chip_close(chip);
        return 1;
    }

    ret = gpiod_line_request_both_edges_events(line, CONSUMER);
    if (ret < 0) {
        fprintf(stderr, "Error requesting line: %s\n", strerror(errno));
        gpiod_line_release(line);
        gpiod_chip_close(chip);
        return 1;
    }

    printf("Monitoring PWM on GPIO %d (offset %d)...\n", GPIO_LINE_OFFSET, GPIO_LINE_OFFSET);

    struct timespec prev_time, curr_time;
    uint64_t prev_edge_ns = 0, curr_edge_ns = 0;
    int prev_value = 0;
    struct pwm_measurement pwm;

    prev_value = gpiod_line_get_value(line);
    clock_gettime(CLOCK_MONOTONIC, &prev_time);
    prev_edge_ns = prev_time.tv_sec * 1000000000ULL + prev_time.tv_nsec;

    while (1) {
        struct gpiod_line_event event;

        ret = gpiod_line_event_wait(line, NULL);
        if (ret < 0) {
            fprintf(stderr, "Error waiting for event: %s\n", strerror(errno));
            break;
        }

        ret = gpiod_line_event_read(line, &event);
        if (ret < 0) {
            fprintf(stderr, "Error reading event: %s\n", strerror(errno));
            continue;
        }

        clock_gettime(CLOCK_MONOTONIC, &curr_time);
        curr_edge_ns = curr_time.tv_sec * 1000000000ULL + curr_time.tv_nsec;

        uint64_t time_since_last_edge = curr_edge_ns - prev_edge_ns;

        if (prev_value == 0 && event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
            pwm.low_time_ns = time_since_last_edge;
        } else if (prev_value == 1 && event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
            pwm.high_time_ns = time_since_last_edge;

            pwm.period_ns = pwm.high_time_ns + pwm.low_time_ns;
            pwm.duty_cycle = (double)pwm.high_time_ns / (double)pwm.period_ns * 100.0;
            pwm.frequency_hz = 1000000000.0 / (double)pwm.period_ns;

            printf("PWM: %.2f%% duty, %.2f Hz (High: %llu ns, Low: %llu ns)\n",
                   pwm.duty_cycle, pwm.frequency_hz,
                   (unsigned long long)pwm.high_time_ns,
                   (unsigned long long)pwm.low_time_ns);
        }

        prev_value = (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) ? 1 : 0;
        prev_edge_ns = curr_edge_ns;
    }

    gpiod_line_release(line);
    gpiod_chip_close(chip);

    return 0;
}
