/*
 * DC_Motors.c
 *
 *  Created on: 24 jul. 2022
 *      Author: Enrique
 */

#include "utilities/DC_Motors.h"

float countToDegree(int counter)
{
   return ((float)counter)*DEG_CNT;
}

void setDirection(int dir, struct Motor xMotor)
{
    /*  BITS DE CONTROL L298D : Deben tener diferente estado  */
    if(dir == HORARIO)
    {
        gioSetBit(hetPORT1, xMotor.controlBit_1, 0);
        gioSetBit(hetPORT1, xMotor.controlBit_2, 1);
    }
    else if(dir == ANTIHORARIO)
    {
        gioSetBit(hetPORT1, xMotor.controlBit_1, 1);
        gioSetBit(hetPORT1, xMotor.controlBit_2, 0);
    }
    else
    {
        gioSetBit(hetPORT1, xMotor.controlBit_1, 0);
        gioSetBit(hetPORT1, xMotor.controlBit_2, 0);
    }
}

int motorPID(struct Motor *xMotor, float error[], float refPos, float actPos)
{
    int PID = 0;

    error[actual] = refPos - actPos;

    if(fabs(error[actual]) < 3.0)
        xMotor->errCheck = 1;
    else
        xMotor->errCheck = 0;

    xMotor->Up = (int)(error[actual]*(xMotor->Kp));
    xMotor->Ui = xMotor->Ui +  (int)(error[actual]*(xMotor->Ki));
//    xMotor->Ud = (int)((xMotor->Kd)*(error[actual] - error[prev])/((float)CTRL_TIME));
    PID = xMotor->Up + xMotor->Ui + xMotor->Ud; // TODO: A*cos(degree)

    if(PID>0)
        setDirection(ANTIHORARIO, *xMotor);

    else if(PID<0)
        setDirection(HORARIO, *xMotor);

    else
        setDirection(PARO, *xMotor);

    if(abs(PID) > 1000)
        PID = 1000;

    error[prev] = error[actual];

    return abs(PID);
}
