#ifndef MATH_ROBOT_H
#define MATH_ROBOT_H

#include "basic_types.h"

#define R_SQRT_2_8 362

uint32 r_square(int x);

/*
This is a specific math-library with integers.
The angles are between 0 (=0rad) and 8191 (8192= 2*pi rad ): Name of this unit : r_angle
*/

#define RANGLE_BITS  13
#define RANGLE_UNIT  (1<<RANGLE_BITS)     // 8192
#define RANGLE_PI   (1<<(RANGLE_BITS-1))  // 4096
#define RANGLE_PI2  (1<<(RANGLE_BITS-2))  // 2048
#define RANGLE_PI4  (1<<(RANGLE_BITS-3))  // 1024


#define DEG2RANGLE(degrees) (((degrees)*(1<<13))/360)
#define RANGLE2DEG(r_angle) (((r_angle)*360)/(1<<13))

// return the same value, normalized between -PI2 and PI2 (-4096 and 4096)
int16 rangle_normalize(int x);

int16 r_asin(double x);
int16 r_acos(double x);
int16 r_atan(double x);

uint16 r_sqrt(double x);

#endif
