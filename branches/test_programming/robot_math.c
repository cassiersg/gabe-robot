#include "robot_math.h"

/*r_asin, r_acos and r_atan can be negative in order to exprees FAILURE */

#define COS_SIN_BITS 12
#define TANGENT_BITS 12
#define ANGLE_BITS   13

#define COS_SIN_UNIT (1<<COS_SIN_BITS) // 4096
#define TANGENT_UNIT (1<<TANGENT_BITS) // 4096
#define ANGLE_UNIT   (1<<ANGLE_BITS)   // 8192

#define SIN_PI4 ((46400*COS_SIN_UNIT)>>16)  // 46400.0>>16 *2*PI >= sin(pi/4)

static int sinCos(int x);

int16 r_asin(int x)
{
    if (x>COS_SIN_UNIT || x< -COS_SIN_UNIT)
        return FAILURE;
    else
    {
        if (x>SIN_PI4 || x<-SIN_PI4)
        {
            return x<0 ? r_asin(sinCos(x))-ANGLE_UNIT/4 : ANGLE_UNIT/4-r_asin(sinCos(x));
        }
        else
        {
            int x_square = (x*x)>>COS_SIN_BITS;
            int i = 1;
            int coeff = 1<<COS_SIN_BITS;
            int res = 1<<COS_SIN_BITS;
            for (; i<7; i++)
            {
                coeff*= (x_square * (2*i-1)) / (2*i);
                coeff = coeff>>COS_SIN_BITS;
                res+= coeff / (2*i+1);
            }
            res = (res*x)>>COS_SIN_BITS;
            res = ((res<<16) / (R_PIB16));
            return res;
        }
    }
}

int16 r_acos(int x)
{
   if (x>COS_SIN_UNIT || x< -COS_SIN_UNIT)
        return FAILURE;
    else
    {
        int angle = ANGLE_UNIT/4 - r_asin(x);
        return angle;
    }
}

int16 r_atan(int x)
{
    if (x < -TANGENT_UNIT || x > TANGENT_UNIT)
        return FAILURE;
    else
    {
        int x_square = (x*x)>>TANGENT_BITS;
        int coeff = 1<<TANGENT_BITS;
        int res = 1<<TANGENT_BITS;
        int i = 1;
        for (;i<20; i++)
        {
            coeff*= x_square;
            coeff = -(coeff>>TANGENT_BITS);
            res+= coeff / (2*i+1);
        }
        res = (res*x)>>TANGENT_BITS;
        res = ( (res<<16) / (R_PIB16) );
        return res;
    }
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

int r_abs(int x)
{
	return x<0 ? -x : x;
}

static int sinCos(int x)
{
    return r_sqrt(COS_SIN_UNIT*COS_SIN_UNIT-x*x);
}
