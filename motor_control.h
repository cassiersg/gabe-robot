#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "basic_types.h"

// distance between motor2 and motor3 (mm)
#define LEN1 70
//distance between motor3 and the tip (mm)
#define LEN2 60
// distance between motor1 and motor2 (mm)
#define LEN3 20

int motor_init(void);
int motor_setAngle(int angle, int motorIndex);
int pod_setPosition(int motor1, int motor2, int motor3, int x, int y, int z);

#endif
