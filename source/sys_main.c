/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
#include "FreeRTOS.h"
#include "os_task.h"
#include "sys_core.h"
#include "gio.h"
#include "sci.h"
#include "spi.h"
#include "het.h"
#include "os_queue.h"

#include <utilities/PWM.h>
#include <utilities/MCI.h>
#include <utilities/DC_Motors.h>

/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */

/*
 * UPDOWN
 * CUADRADO
 * TRIANGULO
 *
 * */

/*------------------------------- USER PARAMETERS ------------------------------------*/
#define GARRA   TRUE
/*==============================>*/ #define     TRIANGULO /* <=========================*/

#ifdef UPDOWN
float pDeltaPath[N_PNTS][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
#endif
#ifdef TRIANGULO
float pDeltaPath[N_PNTS][3] = {{-1.6, -4.9, 3.0}, {-3.2, 5.5, 3.0}, {5.5, 3.0, 3.0}};
#endif
#ifdef CUADRADO
float pDeltaPath[N_PNTS][3] = {{-1.6, -4.9, 3.0}, {-4.9, -1.1, 3.0},{2.2, 7.5, 3.0}, {5.5, 3.0, 3.0}};
#endif

float pDeltaHome[3] = {0.0, 0.0, 0.0};

#ifdef GARRA
                  /*   PWM CB1 CB2  Up  Up  Ui   Kp     Ki      Kd   cont erChk */
struct Motor Motor_1 = {0,  2,  4,  0,  0,  0,  132.0,   2.0,   54.0,   0,   0};
struct Motor Motor_2 = {6,  10, 12, 0,  0,  0,  135.0,   2.0,   54.0,   0,   0};
struct Motor Motor_3 = {14, 16, 18, 0,  0,  0,  138.0,   2.0,   54.0,   0,   0};

#else
/*   PWM CB1 CB2  Up  Up  Ui   Kp     Ki      Kd   cont erChk */
struct Motor Motor_1 = {0,  2,  4,  0,  0,  0,  44.0,   0.8,   18.0,   0,   0};
struct Motor Motor_2 = {6,  10, 12, 0,  0,  0,  45.0,   0.8,   18.0,   0,   0};
struct Motor Motor_3 = {14, 16, 18, 0,  0,  0,  46.0,   0.8,   18.0,   0,   0};
#endif


/*------------------------------------------------------------------------------------*/

/*  CONTROL */
int ISRbit = 0;
hetSIGNAL_t M1;
hetSIGNAL_t M2;
hetSIGNAL_t M3;

/*  SCI Variables   */
char sciBuffer[12];

struct posActStruc
{
    float theta1;
    float theta2;
    float theta3;
};
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
void vPosition(void *pvParameters);
void vMotorCtrl(void *pvParameters);
void vPlanner(void *pvParameters);

xQueueHandle Int2Pos_QHandle = 0;
xQueueHandle Pos2MCtrl_QHandle = 0;
xQueueHandle MCtrl2Plan_QHandle = 0;
xQueueHandle MCI2MCtrl_QHandle = 0;

#define M1_CH_A  0       /* GIO */
#define M1_CH_B  1       /* GIO */

#define M2_CH_A  4       /* GIO */
#define M2_CH_B  5       /* GIO */

#define M3_CH_A  6       /* GIO */
#define M3_CH_B  22       /* GIO */

/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
   gioInit();
   sciInit();
   hetInit();

   _enable_interrupt_();
   _enable_IRQ();

   Int2Pos_QHandle = xQueueCreate(1, sizeof(uint32));
   Pos2MCtrl_QHandle = xQueueCreate(1, 3*sizeof(float));
   MCI2MCtrl_QHandle = xQueueCreate(10, 3*sizeof(float));
   MCtrl2Plan_QHandle = xQueueCreate(10,3*sizeof(float));

   /*   INTERRUPCION GIO    */
   gioEnableNotification(gioPORTA,M1_CH_A);
   gioEnableNotification(gioPORTA,M2_CH_A);
   gioEnableNotification(gioPORTA,M3_CH_A);

   xTaskCreate(vMotorCtrl, "PID", 512, NULL, 2, NULL);
   xTaskCreate(vPlanner, "MCI", 512, NULL, 1, NULL);
   xTaskCreate(vPosition, "Cuentas", 128, NULL, 3, NULL);

   vTaskStartScheduler();

   while(1);
/* USER CODE END */

}


/* USER CODE BEGIN (4) */

void gioNotification(gioPORT_t *port, uint32 bit)
{
    portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xQueueOverwriteFromISR(Int2Pos_QHandle,&bit,&xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void vPosition(void *pvParameters)
{
    uint32 ISRbit = 0;
    float aActualPosition[3] = {0.0, 0.0, 0.0};  // Posici�n actual en grados

    while(1)
    {
        if(xQueueReceive(Int2Pos_QHandle, &ISRbit, portMAX_DELAY) == pdTRUE)
        {
            if(ISRbit == M1_CH_A)
            {
                if(gioGetBit(gioPORTA,M1_CH_B))
                    Motor_1.counter++;

                else
                    Motor_1.counter--;

                aActualPosition[0] = countToRads(Motor_1.counter);
            }

            if(ISRbit == M2_CH_A)
            {
                if(gioGetBit(gioPORTA,M2_CH_B))
                    Motor_2.counter++;

                else
                    Motor_2.counter--;

                aActualPosition[1] = countToRads(Motor_2.counter);
            }

            if(ISRbit == M3_CH_A)
            {
                if(gioGetBit(hetPORT1,M3_CH_B))
                    Motor_3.counter++;

                else
                    Motor_3.counter--;

                aActualPosition[2] = countToRads(Motor_3.counter);
            }
            xQueueOverwrite(Pos2MCtrl_QHandle, aActualPosition);
        }
    }
}

void vMotorCtrl(void *pvParameters)
{
    portTickType xLastWakeTime = xTaskGetTickCount();

    /* ---- CONTROL ---*/

    float error[2] = {0.0, 0.0};

    float aActualPosition[3] = {0.0, 0.0, 0.0};  // Posici�n actual en grados
    float aRefPosition[3] = {0.0, 0.0, 0.0};  // Posici�n deseada en grados

    /* ------- MOTOR DRIVE ------- */
    M1.period = 20000;
    M1.duty = 0;
    pwmSetSignal10e3(hetRAM1, M1_PWM, M1);

    M2.period = 20000;
    M2.duty = 0;
    pwmSetSignal10e3(hetRAM1, M2_PWM, M2);

    M3.period = 20000;
    M3.duty = 0;
    pwmSetSignal10e3(hetRAM1, M3_PWM, M3);

    setDirection(HORARIO, Motor_1);

    while (1)
    {

        xQueueReceive(Pos2MCtrl_QHandle, aActualPosition, 0);
        xQueueReceive(MCI2MCtrl_QHandle, aRefPosition, 0);

        M1.duty = motorPID(&Motor_1, error, aRefPosition[0], aActualPosition[0]);
        pwmSetSignal10e3(hetRAM1, M1_PWM, M1);

        M2.duty = motorPID(&Motor_2, error, aRefPosition[1], aActualPosition[1]);
        pwmSetSignal10e3(hetRAM1, M2_PWM, M2);

        M3.duty = motorPID(&Motor_3, error, aRefPosition[2], aActualPosition[2]);
        pwmSetSignal10e3(hetRAM1, M3_PWM, M3);

//        sciSend(scilinREG, sprintf(sciBuffer,"%.2f %.2f %.2f - %.2f %.2f %.2f\n\r",aRefPosition[0],aRefPosition[1],aRefPosition[2], aActualPosition[0], aActualPosition[1], aActualPosition[2]), (uint8*)sciBuffer);

        if(Motor_1.errCheck == 1 && Motor_2.errCheck == 1 && Motor_3.errCheck == 1)
        {
            xQueueSend(MCtrl2Plan_QHandle,&aActualPosition,100);    // TODO: Checar delays
        }

        vTaskDelayUntil(&xLastWakeTime,(CTRL_TIME/portTICK_RATE_MS));
    }
}

void vPlanner(void *pvParameters)
{
    int idxPath = 0;
    int motorCheck[3] = {0,0,0};
    float aActualPosition[3] = {0.0, 0.0, 0.0};  // Posici�n actual en grados
    float aFinalPosition[3] = {0.0, 0.0, 0.0};  // Posici�n final deseada en grados
    float aNewPosition[3] = {0.0, 0.0, 0.0};  // Nueva posici�n parcial deseada en grados
    float pDPathAux[3] = {0.0, 0.0, 0.0};

    while(1)
    {
        if(xQueueReceive(MCtrl2Plan_QHandle,&aActualPosition,portMAX_DELAY))
        {
            gioToggleBit(gioPORTA, 2);

            point2point(pDeltaPath, pDPathAux, idxPath);                            // Selecciona un punto destino de la rutina
            getMotorsAngle(pDPathAux, aFinalPosition);                              // Entra punto y salen �ngulos de motores)
            setAngleIncr(aActualPosition, aFinalPosition, aNewPosition, motorCheck);  // Incrementa angulo actual

            if(fabs(aActualPosition[0] - aFinalPosition[0]) < 1.0 &&
               fabs(aActualPosition[1] - aFinalPosition[1]) < 1.0 &&
               fabs(aActualPosition[2] - aFinalPosition[2]) < 1.0)
            {
                if(idxPath < (N_PNTS - 1))
                    idxPath++;
                else
                    idxPath = 0;
            }
            xQueueSend(MCI2MCtrl_QHandle, aNewPosition, 10);
        }
    }
}
/* USER CODE END */
