#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "basic_types.h"


/* Must be called one (and only one) time before any use of motors. */
void motor_init(void);


/* Return SUCCESS if the given angle is valid for the motor
 *
 * angle : Rangle
 */
int testMotor(int angle, unsigned int motorIdx);


/* Set the motor <motorIdx> to the <angle> position.
 * The mouvement last <duration>.
 * Return FAILURE if the requested angle cannot be applied, else return SUCCESS
 *
 * angle : RAngle
 * duration : ms
 */
int setMotor(int angle, unsigned int motorIdx, uint32 duration);


#endif // MOTOR_CONTROL_H
