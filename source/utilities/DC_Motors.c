/*
 * DC_Motors.c
 *
 *  Created on: 24 jul. 2022
 *      Author: Enrique
 */

#include "utilities/DC_Motors.h"

float countToRads(int counter)
{
   return ((float)counter)*DEG_CNT;
}

void setDirection(int dir, struct Motor xMotor)
{
    /*  BITS DE CONTROL L298D : Deben tener diferente estado  */
    if(dir == HORARIO)
    {
        gioSetBit(hetPORT1, xMotor.controlBit_1, 1);
        gioSetBit(hetPORT1, xMotor.controlBit_2, 0);
    }
    else if(dir == ANTIHORARIO)
    {
        gioSetBit(hetPORT1, xMotor.controlBit_1, 0);
        gioSetBit(hetPORT1, xMotor.controlBit_2, 1);
    }
    else
    {
        gioSetBit(hetPORT1, xMotor.controlBit_1, 0);
        gioSetBit(hetPORT1, xMotor.controlBit_2, 0);
    }
}

int motorPID(struct Motor *xMotor, float error[], float refPos, float actPos)
{
    float PID = 0;
    error[prev] = error[actual];
    error[actual] = refPos - actPos;

    if(fabs(error[actual]) < 2.5)
        xMotor->errCheck = 1;
    else
        xMotor->errCheck = 0;

    xMotor->Up = error[actual]*(xMotor->Kp);
    xMotor->Ud = (xMotor->Kd)*(error[actual] - error[prev]);
    if(fabs(error[actual]) < 20.0)
    {
        xMotor->Ui = xMotor->Ui +  error[actual]*(xMotor->Ki);
    }
    else
    {

    }
    //            xMotor->Ui = 0.0;

    PID = xMotor->Up + xMotor->Ui + xMotor->Ud;// + (int)(2*cos(actPos));

    if(PID>0.0)
        setDirection(ANTIHORARIO, *xMotor);

    else if(PID<0.0)
    {
        setDirection(HORARIO, *xMotor);
        PID = PID - 400.0*cos(actPos*pi/180.0);
    }

    else
        setDirection(PARO, *xMotor);

    if(fabs(PID) > 3000.0)
        PID = 3000.0;

    if(fabs(PID) < 300.0)
        PID = 300.0;

    return abs((int)PID);

}
