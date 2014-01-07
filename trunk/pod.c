#include <stdio.h>
#include <stdlib.h>

#include "pod_control.h"
#include "motor_control.h"
#include "robot_console.h"

#define NB_PODS 4


/* NOTES :
* An absolute position (abs, absPos) represents a position in a coordinates 
  system depending on the pod's INDEX (0 to 3), which don't change during execution.
* Countrairwise, a relative position (rel) is depending on the pod's location,
  and thus on the robot's direction
* The default direction is those with the front side between pods 0 and 1
*/

/*  Physical parameters of pods */
// distance between motor1 and motor2 (mm)
#define LEN1 15
// distance between motor2 and motor3 (mm)
#define LEN2 40
//distance between motor3 and the tip (mm)
#define LEN3 110
//correction angle motor 1
#define ANGLE1 DEG2RANGLE( 0 )
//correction angle motor 2
#define ANGLE2 DEG2RANGLE( -23 )
//correction angle motor 3
#define ANGLE3 DEG2RANGLE( -72 )
//motor's directions (-1 or 1)
#define DIRECTION1 1 
#define DIRECTION2 -1
#define DIRECTION3 1


typedef struct Pod Pod;
struct Pod
{
    int m1Idx;
    int m2Idx;
    int m3Idx;
};

static Pod pods[NB_PODS] = 
{     { 0,  1,  2},
      { 3,  4,  5},
      { 6,  7,  8},
      { 9, 10, 11},
};

/* Correspondence between relative and absolute positions */
/* Indexed by PodLocation */
static Pod *podsLocation[NB_PODS] = {&pods[0], &pods[1], &pods[2], &pods[3]};


#define R_SQRT_2_7 R_SQRT_2_8 >> 1 // (1/sqrt(2)) << 8
/* Transfrom relative to absolute coordinates 
    new_x = mat[0][0]*old_x + mat[0][1]*old_y
    new_y = mat[1][0]*old_x + mat[1][1]*old_y
*/
typedef int RotationMatrix[2][2];
/* Indexed by PodLocation */
static RotationMatrix mat_rel2absPos[NB_PODS] = {
// Values are shifted of 8 bits 
    // Front Right
    {{ R_SQRT_2_7, -R_SQRT_2_7}, {R_SQRT_2_7, R_SQRT_2_7}},
    // Front Left
    {{ -R_SQRT_2_7, R_SQRT_2_7}, {R_SQRT_2_7, R_SQRT_2_7}},
    // Hind Left
    {{ R_SQRT_2_7, R_SQRT_2_7}, {R_SQRT_2_7, -R_SQRT_2_7}},
    // Hind Right
    {{ -R_SQRT_2_7, -R_SQRT_2_7}, {R_SQRT_2_7, -R_SQRT_2_7}}
};


static int podAbsPos2Angles(int x, int y, int z, int *m1, int *m2, int *m3);
static int testPodAbsPos(int x, int y, int z, const Pod *pod);
static int setPodAbsPos(int x, int y, int z, const Pod *pod, uint32 duration);

static void relative2absPos(int *x, int *y, int *z, PodLocation pod);

// Console interface
static int c_setAbsPosition(char *args[], int argc);


//~~~~~~~~~~ PODS MANGEMENT ~~~~~~~~~~

static int podAbsPos2Angles(int x, int y, int z, int *m1, int *m2, int *m3)
{
    int lenM2Tip = z*z + r_square(r_sqrt(x*x + y*y) - LEN1);
    if (lenM2Tip <= 0)
    {
        return FAILURE;
    }
    lenM2Tip = r_sqrt(lenM2Tip);
    if (y == 0)
    {
        *m1 = rangle_normalize(DIRECTION1 * ANGLE1);
    }
    else
    {
        *m1 = rangle_normalize(DIRECTION1 * (r_atan(((float)x) / y) + ANGLE1));
    }
    int num = LEN2*LEN2 - LEN3*LEN3 + r_square(lenM2Tip);
    float beta_cos = ((float)(num)) /((float)(2 * LEN2 * lenM2Tip));
    if (beta_cos > 1 || beta_cos < -1)
    {
        return FAILURE;
    }
    *m2 = rangle_normalize(DIRECTION2 * (r_acos(((float)z) / lenM2Tip) + r_acos(beta_cos) - RANGLE_PI2 + ANGLE2));
    num = LEN2*LEN2 + LEN3*LEN3 - r_square(lenM2Tip);
    float gamma_cos = ((float)(num)) / (2*LEN2*LEN3);
    if (gamma_cos > 1 || gamma_cos < -1)
    {
        return FAILURE;
    }
    *m3 = rangle_normalize(DIRECTION3 * (r_acos(gamma_cos) + ANGLE3));
    return SUCCESS;
}   

static int testPodAbsPos(int x, int y, int z, const Pod *pod)
{
    int m1, m2, m3;
    if (podAbsPos2Angles(x, y, z, &m1, &m2, &m3) == FAILURE)
    {
        return FAILURE;
    }
    if (testMotor(m1, pod->m1Idx) == FAILURE || 
        testMotor(m2, pod->m2Idx) == FAILURE || 
        testMotor(m3, pod->m3Idx) == FAILURE)
    {
        return FAILURE;
    }
    return SUCCESS;
}       

static int setPodAbsPos(int x, int y, int z, const Pod *pod, uint32 duration)
{
    int m1, m2, m3;
    if (podAbsPos2Angles(x, y, z, &m1, &m2, &m3) == FAILURE)
    {
        return FAILURE;
    }
    if (testMotor(m1, pod->m1Idx) == FAILURE || 
        testMotor(m2, pod->m2Idx) == FAILURE || 
        testMotor(m3, pod->m3Idx) == FAILURE)
    {
        return FAILURE;
    }
    setMotor(m1, pod->m1Idx, duration);  
    setMotor(m2, pod->m2Idx, duration);
    setMotor(m3, pod->m3Idx, duration);
    return SUCCESS;
}

static void relative2absPos(int *x, int *y, int *z, PodLocation pod)
{
    RotationMatrix *rotMatrix = &mat_rel2absPos[pod];
    int v_x = (*rotMatrix[0][0] * (*x) + *rotMatrix[0][1] * (*y)) >> 8;
    int v_y = (*rotMatrix[1][0] * (*x) + *rotMatrix[1][1] * (*y)) >> 8;
    *x = v_x;
    *y = v_y;
}

int testPodPos(int x, int y, int z, PodLocation pod)
{
    relative2absPos(&x, &y, &z, pod);
    return testPodAbsPos(x, y, z, podsLocation[pod]);
}

int setPodPos(int x, int y, int z, PodLocation pod, uint32 duration)
{
    relative2absPos(&x, &y, &z, pod);
    return setPodAbsPos(x, y, z, podsLocation[pod], duration);
}

/*DEPRECATED*/
int pod_setPosition(int x, int y, int z, int time, int podIdx)
{
    return setPodAbsPos(x, y, z, podsLocation[podIdx], time);
}       
      

//~~~~~~~~~~ CONSOLE APPLICATION ~~~~~~~~~~

CmdEntry podConsoleMenu[] = {
	{ "setabsposition", c_setAbsPosition,  4, "setabsposition <x> <y> <z> <podIdx>[=0] <duration ms>=[500]\r\n\tSet absolute position"},
	{ "sap",            c_setAbsPosition,  4, "alias of setabsposition"},
	{ NULL,       NULL,      0, NULL},
};

static int c_setAbsPosition(char *args[], int argc)
{
	int x = atoi(args[1]);
	int y = atoi(args[2]);
	int z = atoi(args[3]);
	unsigned int podIdx = 0;
	unsigned int duration = 500;
	if (argc>4)
	{
		podIdx = atoi(args[4]);
    }
	if (argc>5)
	{
		duration = atoi(args[5]);
	}
	printf("setAbsPosition fuction x:%i y:%i z:%i pod:%i duration: %i\r\n", x, y, z, podIdx, duration);
	if (podIdx >= NB_PODS || setPodAbsPos(x, y, z, podsLocation[podIdx], duration)!=SUCCESS)
		printf("\tcould not apply requested position\r\n");
	return 0;
}


/*~~~~~~~~~~~~~ INITIALIZATION ~~~~~~~~~~~~~~ */

void pod_init(void)
{
    // pod control by console
	console_addCommandsList(podConsoleMenu);
}