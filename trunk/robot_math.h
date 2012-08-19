#ifndef MATH_ROBOT_H
#define MATH_ROBOT_H

#include "basic_types.h"

#define SQUARE(x) ((x)*(x))
#define R_ABS(x)  ((x)>=0 ? (x) : (-(x)))

#define R_SQRT_2_8 362

/*
This is a specific math-library with integers.
The angles are between 0 (=0rad) and 8191 (8192= 2*pi rad ): Name of this unit r_angle
r_asinD(x, y) = asin(x/y); idem for acos and atan
*/

#define RANGLE_BITS  13
#define RANGLE_UNIT  (1<<RANGLE_BITS)      // 8192
#define RANGLE_PI   (1<<(RANGLE_BITS-1))  // 4096
#define RANGLE_PI2  (1<<(RANGLE_BITS-2))  // 2048
#define RANGLE_PI4  (1<<(RANGLE_BITS-3))  // 1024


#define DEG_RANGLE(degrees) (((degrees)*(1<<13))/360)
#define RANGLE_DEG(r_angle) (((r_angle)*360)/(1<<13))

#define RANGLE_NORMALIZE(x) while ((x) > RANGLE_PI) {(x) -= RANGLE_UNIT;} while ((x) <= -RANGLE_PI) {(x) += RANGLE_UNIT;}

int16 r_asinD(int x, int y);
int16 r_acosD(int x, int y);
int16 r_atanD(int x, int y);

uint16 r_sqrt(uint32 x);

#endif
