#include "robot_math.h"

/*r_asin, r_acos and r_atan can be negative in order to exprees FAILURE */

uint16 r_sqrt(uint32 x);
static int sinCos(int x);

int16 r_asin(int x)
{
    if (x>4096 || x< -4096)
        return FAILURE;
    else
    {
        if (x>2897 || x<-2897)
        {
            return x<0 ? r_asin(sinCos(x))-8192/4 : 8192/4-r_asin(sinCos(x));
        }
        else
        {
            int x_square = (x*x)>>12;
            int i = 1;
            int coeff = 1<<12;
            int res = 1<<12;
            for (; i<7; i++)
            {
                coeff*= (x_square * (2*i-1)) / (2*i);
                coeff = coeff>>12;
                res+= coeff / (2*i+1);
            }
            res = (res*x)>>12;
            res = ((res<<16) / (R_PIB16));
            return res;
        }
    }
}

int16 r_acos(int x)
{
   if (x>4096 || x< -4096)
        return FAILURE;
    else
    {
        int angle = 8192/4 - r_asin(x);
        return angle;
    }
}

int16 r_atan(int x)
{
    if (x < -4096 || x > 4096)
        return FAILURE;
    else
    {
        int x_square = (x*x)>>12;
        int coeff = 1<<12;
        int res = 1<<12;
        int i = 1;
        for (;i<32; i++)
        {
            coeff*= x_square;
            coeff = -(coeff>>12);
            res+= coeff / (2*i+1);
        }
        res = (res*x)>>12;
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

static int sinCos(int x)
{
    return r_sqrt(4096*4096-x*x);
}
