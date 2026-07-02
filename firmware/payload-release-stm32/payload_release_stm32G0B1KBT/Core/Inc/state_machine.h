#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

typedef enum state_machine_t {
    LOCK,
    ARM,
    RELEASE,
    RETRACT,
    FAULT
} state_machine_t;

typedef enum commands_t {
    CMD_LOCK,
    CMD_ARM,
    CMD_RELEASE,
    CMD_RETRACT,
} commands_t;

extern state_machine_t state;
extern commands_t command;

void state_machine_loop();
void fault(); 

#endif