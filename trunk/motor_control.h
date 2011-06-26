#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "basic_types.h"


int motor_init(void);
// speed : °/sec
int motor_setAngle(int angle, int motorIndex, int speed);

/* ---------- for pod_setPosition ---------- */
// distance between motor1 and motor2 (mm)
#define LEN1 50
// distance between motor2 and motor3 (mm)
#define LEN2 50
//distance between motor3 and the tip (mm)
#define LEN3 50

//correction angle motor 1
#define ANGLE1 0
//correction angle motor 2
#define ANGLE2 -135
//correction angle motor 2
#define ANGLE3 -135

//directions motors (-1 or 1)
#define DIRECTION1 -1 //not defined
#define DIRECTION2 1
#define DIRECTION3 -1

/*
amène l extrémité d une patte au point désigné
centre du repère: axe du moteur1;
x: horizontal, perpendiculaire au premier segment quand angle moteur1 = 0, sens: vers la droite en regardant du centre du robot;
y: horizontal, vers l'extérieur du robot;
z: vertical, vers le bas.
numéros de moteurs: du centre vers l'extrémité de la patte, SELON LE MODELE POSTE : v1
longueur des segments : LEN1 LEN2 LEN3, centre->extrémité
moteur1: croissant vers la droite (en regardant du centre du robot)
moteur2: position "0" a 45° vers le bas, augmentation vers le haut
moteur3: position "0" vertical, augmentation vers l exterieur
!!! y>0 ; z>0
*/
int pod_setPosition(float x, float y, float z, int motor1, int motor2, int motor3);

#endif
