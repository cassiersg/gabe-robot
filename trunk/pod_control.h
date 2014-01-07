#ifndef POD_CONTROL_H
#define POD_CONTROL_H

#include "robot_math.h"

/* 
   A POD'S MANAGING SYSTEM

There are four physical pods, each one made of three motors.

A global robot's direction is defined: it's the main advance direction, relative
to the default direction (direction at starting).
The direction can be changed relative to the default direction or relative to 
the current direction (a rotation).

The pods are designated by their location with respect to the current front side.

For each pod location, a coordinates system is defined:
The origin of the coordinate is located at the basis of the pod (axis of the first motor),
Axis directions are shown below.

 
 * POSITION COORDINATES SYSTEM
 *
 * Origin is the axis of first motor
 *
 *      y      y
 *      ^      ^
 *      |      |
 *   x<-+------+->x
 *      |FL  FR|
 *      |      |
 *      | ROBOT|
 *      y      y
 *      ^      ^
 *      |HL  HR|
 *   x<-+------+->x
 *
 *   z : vertical, points to the ground
 *   x, y, z in mm
*/


typedef enum {PodFrontRight=0, PodFrontLeft=1, PodHindLeft=2, PodHindRight=3} PodLocation;

void pod_init(void);

/* Test if the position (x, y, z) is valid for the <pod> */
int testPodPos(int x, int y, int z, PodLocation pod);

/* Set the tip of the pod at the (x,y,z) point, the move will take <duration> */
int setPodPos(int x, int y, int z, PodLocation pod, uint32 duration);

/* DEPRECATED
Use absolute positions and pod Indexes
*/
int pod_setPosition(int x, int y, int z, int time, int podIdx);

#endif // POD_CONTROL_H