#include "state_machine.h"
#include "servo.h"
#include "uart.h"
#include <stdint.h>
#include <string.h>

state_machine_t state;
commands_t command;

void fault() {
    const char *buffer = "MCU: Fault";
    if (UART_tx(*buffer) != ERR_TX) {}
}

void state_machine_loop() {
    char *buffer = 0;
    UART_rx((uint8_t*)buffer);
    if (!strncmp(buffer, "LOCK", 4)) {
        state = LOCK;
    }  else if (!strncmp(buffer, "ARM", 3)) {
        state = ARM;
    }  else if (!strncmp(buffer, "RELEASE", 7)) {
        state = RELEASE;
    }  else if (!strncmp(buffer, "RETRACT", 7)) {
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