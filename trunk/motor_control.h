#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "basic_types.h"

int motor_init(void);
int motor_setAngle(int angle, int motorIndex);

#endif
