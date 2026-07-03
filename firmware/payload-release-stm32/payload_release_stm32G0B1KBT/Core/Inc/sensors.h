#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include "errors.h"

void ultrasonic_trigger();
app_err_t __ultrasonic_distance_cm(uint16_t *buffer);
app_err_t ultrasonic_distance_cm(uint16_t *buffer);

#endif
