#ifndef SERVO_H
#define SERVO_H

void lock();
void arm();
void release();
void retract();

extern int release_busy, retract_busy;

#endif