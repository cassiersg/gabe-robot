#include "robot_math.h"
#include <math.h>

#define RAD_RANGLE(rad) (rad/2/M_PI*ANGLE_UNIT)

int16 r_asinD(int x, int y)
{
	return (int)RAD_RANGLE(asin(((float)x)/y));
}

int16 r_acosD(int x, int y)
{
	return (int)RAD_RANGLE(acos(((float)x)/y));
}

int16 r_atanD(int x, int y)
{
	return (int)RAD_RANGLE(atan(((float)x)/y));
}

uint16 r_sqrt(uint32 x)
{
    if (x==0) return 0;
    else
    {
        int i;
        uint32 x2 = x;
        for (i=0; x2!=0; i++) { x2=x2>>1;}
        uint32 val1;
        val1 = 1<<(i/2);
        uint32 val2 = x/val1;
        while (val1!=val2)
        {
            if (val1-val2 == 1) return val2;
            if (val2-val1 == 1) return val1;
            val1 = (val1 + val2) / 2;
            val2 = x/val1;
        }
        return val1;
    }
}
