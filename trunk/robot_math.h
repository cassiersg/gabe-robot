#ifndef MATH_ROBOT_H
#define MATH_ROBOT_H

#include "basic_types.h"

/*
This is a specific math-library with integers.
The angles are between 0 (=0rad) and 8191 (8192= 2*pi rad ): Name of this unit r_angle
The sinus and cosinus are between -4096 (=-1) and 4096 (=1).
The tangents are between -4096 (=-1) and 4096 (=1).
*/

/*R_PIB16>>16=M_PI */
#define R_PIB16 0x3243F

#define DEG_RANGLE(degree) (((degree)<<13)/360)
#define RANGLE_DEG(r_angle) (((r_angle)*360)>>13)

int16 r_asin(int x);
int16 r_acos(int x);
int16 r_atan(int x);

uint16 r_sqrt(uint32 x);

#endif
