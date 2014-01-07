#include "robot_math.h"
#include <math.h>

#define RAD2RANGLE(rad) (rad/2/M_PI*RANGLE_UNIT)

int16 rangle_normalize(int x)
{
    while (x > RANGLE_PI) 
    {
        x -= RANGLE_UNIT;
    }
    while (x <= -RANGLE_PI)
    {
        x += RANGLE_UNIT;
    }
    return x;
}

int16 r_asin(double x)
{
    return (int)RAD2RANGLE(asin(x));
}

int16 r_acos(double x)
{
    return (int)RAD2RANGLE(acos(x));
}

int16 r_atan(double x)
{
    return (int)RAD2RANGLE(atan(x));
}

uint16 r_sqrt(double x)
{
    return (uint16)(sqrt(x) + 0.5);
}
    
uint32 r_square(int x)
{
    return x*x;
}
