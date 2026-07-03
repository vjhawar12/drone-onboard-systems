#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

typedef enum sensor_err_t {
    ERR_NONE,
    ERR_ULTRASONIC
} sensor_err_t;

void ultrasonic_trigger();
sensor_err_t __ultrasonic_distance_cm(uint16_t *buffer);
void ultrasonic_distance_cm(uint16_t *buffer);

#endif
