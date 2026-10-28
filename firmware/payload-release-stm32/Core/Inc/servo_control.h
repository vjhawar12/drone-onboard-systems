#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>

int servo_move_to(uint16_t degrees); 
int servo_retract();
int servo_release()

#endif