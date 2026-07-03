#include "state_machine.h"
#include "servo.h"
#include "uart.h"
#include <stdint.h>
#include <string.h>
#include "sensors.h"

#define DIST_THRESH_CM 30
#define MAX_TRIES 5

state_machine_t state;
commands_t command;
int override = 0; // ultrasonic override

void fault() {
    const char *buffer = "MCU: Fault";
    UART_tx_line(buffer, strlen(buffer));
}

void state_machine_loop() {
    char buffer[256];
    uint16_t samples[MAX_TRIES] = {0};
    uint16_t distance_avg = 0;

    while (1) {
        if (UART_rx_line(buffer, -1) != NONE) {
            continue;
        } else if (!strncmp(buffer, "LOCK", 4)) {
            state = LOCK;
        }  else if (state == LOCK && !strncmp(buffer, "ARM", 3)) {
            state = ARM;
        }  else if (state == ARM && !strncmp(buffer, "RELEASE", 7)) {
            distance_avg = 0;
            if (!override) {
                for (int i = 0; i < MAX_TRIES; i++) {
                    ultrasonic_trigger();
                    if (ultrasonic_distance_cm(&samples[i]) != NONE) {
                        state = FAULT;
                        break;
                    }
                    distance_avg += samples[i];
                }
                if (state == FAULT) continue;
                distance_avg /= MAX_TRIES;
                if (distance_avg <= DIST_THRESH_CM) {
                    state = RELEASE;
                }
            } else {
                state = RELEASE;
            }
        }  else if (state == RELEASE && !strncmp(buffer, "RETRACT", 7)) {
            state = RETRACT;
        }  
        // lock
        switch (state) {
            case LOCK:
                lock();
                break;
            case ARM:
                arm();
                break;
            case RELEASE:
                if (!release_busy) {
                    release();
                }
                break;
            case RETRACT:
                if (!retract_busy) {
                    retract();
                }
                state = LOCK;
                break;
            case FAULT:
                fault();
        }
    }
}