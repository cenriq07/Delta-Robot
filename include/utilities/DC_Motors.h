/*
 * DC_Motors.h
 *
 *  Created on: 24 jul. 2022
 *      Author: Enrique
 */

/**
 * Motor 1
 *  CH A :  GIO 0
 *  CH B :  GIO 1
 *  PWM  :  HET 0
 *  CB 1 :  HET 2
 *  CB 2 :  HET 4
 *
 * Motor 2
 *  CH A :  GIO 4
 *  CH B :  GIO 5
 *  PWM  :  HET 6
 *  CB 1 :  HET 10
 *  CB 2 :  HET 12
 *
 * Motor 3
 *  CH A :  GIO 6
 *  CH B :  HET 22
 *  PWM  :  HET 14
 *  CB 1 :  HET 16
 *  CB 2 :  HET 18
 *
 * */
#include "stdio.h"
#include "stdlib.h"
#include "gio.h"
#include "het.h"
#include "math.h"

#ifndef INCLUDE_UTILITIES_DC_MOTORS_H_
#define INCLUDE_UTILITIES_DC_MOTORS_H_

#define M1_CB1   4       /* HET */
#define M1_CB2   6       /* HET */

#define PULSE_PER_REV   224.0
#define DEG_CNT         360.0/PULSE_PER_REV
#define RAD_CNT         6.2832/PULSE_PER_REV

#define M1_PWM pwm0     /* HET 0 */
#define M2_PWM pwm1     /* HET 1 */
#define M3_PWM pwm2     /* HET 2 */

#define CTRL_TIME       50

#define pi      3.14159

enum
{
    HORARIO,ANTIHORARIO,PARO
};

enum
{
    actual, prev
};

struct Motor
{
    int pwm;
    int controlBit_1;
    int controlBit_2;
    int Up;
    int Ui;
    int Ud;
    float Kp;
    float Ki;
    float Kd;
    int counter;
    int errCheck;
};

float countToRads(int counter);
void setDirection(int dir, struct Motor MX);
int motorPID(struct Motor *xMotor, float error[], float refPos, float actPos);


#endif /* INCLUDE_UTILITIES_DC_MOTORS_H_ */
