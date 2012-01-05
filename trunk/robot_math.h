#ifndef MATH_ROBOT_H
#define MATH_ROBOT_H

#include "basic_types.h"

/*
This is a specific math-library with integers.
The angles are between 0 (=0rad) and 8191 (8192= 2*pi rad ): Name of this unit r_angle
r_asinD(x, y) = asin(x/y); idem for acos and atan
*/

#define ANGLE_BITS  13
#define ANGLE_UNIT  (1<<ANGLE_BITS)      // 8192
#define RANGLE_PI   (1<<(ANGLE_BITS-1))  // 4096
#define RANGLE_PI2  (1<<(ANGLE_BITS-2))  // 2048
#define RANGLE_PI4  (1<<(ANGLE_BITS-3))  // 1024


#define DEG_RANGLE(degrees) (((degrees)*(1<<13))/360)
#define RANGLE_DEG(r_angle) (((r_angle)*360)/(1<<13))

int16 r_asinD(int x, int y);
int16 r_acosD(int x, int y);
int16 r_atanD(int x, int y);

uint16 r_sqrt(uint32 x);

#endif
