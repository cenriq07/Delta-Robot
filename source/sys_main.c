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


hetSIGNAL_t M1;
hetSIGNAL_t M2;
hetSIGNAL_t M3;

                  /*   PWM CB1 CB2  Up  Up  Ui   Kp     Ki      Kd   cont erChk */
struct Motor Motor_1 = {0,  2,  4,  0,  0,  0,  40.0,   0.4,   8.0,   0,   0};
struct Motor Motor_2 = {6,  10, 12, 0,  0,  0,  40.0,   0.4,   8.0,   0,   0};
struct Motor Motor_3 = {14, 16, 18, 0,  0,  0,  40.0,   0.4,   8.0,   0,   0};

/*  CONTROL */
int ISRbit = 0;

/*  SCI Variables   */
char sciBuffer[12];

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
void vMCI(void *pvParameters);
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

   Pos2MCtrl_QHandle = xQueueCreate(10, 3*sizeof(float));
   MCI2MCtrl_QHandle = xQueueCreate(10, 3*sizeof(float));
   Int2Pos_QHandle = xQueueCreate(10, sizeof(int));
   MCtrl2Plan_QHandle = xQueueCreate(10,sizeof(int));

   /*   INTERRUPCION GIO    */
   gioEnableNotification(gioPORTA,M1_CH_A);
   gioEnableNotification(gioPORTA,M2_CH_A);
   gioEnableNotification(gioPORTA,M3_CH_A);

   xTaskCreate(vMotorCtrl, "PID", 512, NULL, 2, NULL);
   xTaskCreate(vPlanner, "MCI", 512, NULL, 1, NULL);
   xTaskCreate(vPosition, "Int2Pos", 512, NULL, 3, NULL);

//   xTaskCreate(vMCI, "MCI", 512, NULL, 1, NULL);


   vTaskStartScheduler();

   while(1);
/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */

void gioNotification(gioPORT_t *port, uint32 bit)   // TODO: Aqui solo se manda el bit de gio que realiza interruocion
{
    xQueueSendFromISR(Int2Pos_QHandle, &bit, 0);
}

void vPosition(void *pvParameters)
{
    float aPosition[3] = {0.0, 0.0, 0.0};  // Posición actual en grados
    uint32 ISRBit = 0;

    while(1)
    {
        if(xQueueReceive(Int2Pos_QHandle,&ISRBit,portMAX_DELAY))
        {
            if(ISRBit == M1_CH_A)
            {
                if(gioGetBit(gioPORTA,M1_CH_B))
                    Motor_1.counter++;

                else
                    Motor_1.counter--;

                aPosition[0] = countToRads(Motor_1.counter);
            }

            if(ISRBit == M2_CH_A)
            {
                if(gioGetBit(gioPORTA,M2_CH_B))
                    Motor_2.counter++;

                else
                    Motor_2.counter--;

                aPosition[1] = countToRads(Motor_2.counter);
            }

            if(ISRBit == M3_CH_A)
            {
                if(gioGetBit(hetPORT1,M3_CH_B))
                    Motor_3.counter++;

                else
                    Motor_3.counter--;

                aPosition[2] = countToRads(Motor_3.counter);
            }

            xQueueSend(Pos2MCtrl_QHandle, aPosition,0);
        }
    }
}

void vMotorCtrl(void *pvParameters)
{
    portTickType xLastWakeTime = xTaskGetTickCount();
    int enaPlan = 0;

    /* ---- CONTROL ---*/

    float error[2] = {0.0, 0.0};

    float aPosition[3] = {0.0, 0.0, 0.0};  // Posición actual en grados
    float dPosition[3] = {0.0, 0.0, 0.0};  // Posición deseada en grados

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
        xQueueReceive(Pos2MCtrl_QHandle, aPosition, 0);
        xQueueReceive(MCI2MCtrl_QHandle, dPosition, 0);

        M1.duty = motorPID(&Motor_1, error, dPosition[0], aPosition[0]);
        pwmSetSignal10e3(hetRAM1, M1_PWM, M1);

        M2.duty = motorPID(&Motor_2, error, dPosition[1], aPosition[1]);
        pwmSetSignal10e3(hetRAM1, M2_PWM, M2);

        M3.duty = motorPID(&Motor_3, error, dPosition[2], aPosition[2]);
        pwmSetSignal10e3(hetRAM1, M3_PWM, M3);

        if(Motor_1.errCheck == 1 && Motor_2.errCheck == 1 && Motor_3.errCheck == 1)
        {
            enaPlan = 1;
            xQueueSend(MCtrl2Plan_QHandle,&enaPlan,100);
        }

        vTaskDelayUntil(&xLastWakeTime,(CTRL_TIME/portTICK_RATE_MS));
    }
}

void vMCI(void *pvParameters)
{
    float theta[3] = {0.0, 0.0, 0.0};               // Angulos calculados
    float deltaPos[3] = {3.0, 0.0, 1.0};            // Punto en el espacio x,y,z


    while(1)
    {
        getMotorsAngle(theta, deltaPos);

        xQueueSend(MCI2MCtrl_QHandle, theta, 1000);
    }
}

void vPlanner(void *pvParameters)
{
//    float posiciones[8] = {0.0,45.0,90.0,135.0,180.0,225.0,270.0,315.0};
//    float posiciones[4] = {0.0,90.0,180.0,270.0};
    float posiciones[10] = {0.0,5.0,10.0,15.0,20.0,25.0,30.0,35.0,40.0,45.0};
//    float posiciones[10] = {0.0,10.0,20.0,30.0,40.0,50.0,60.0,70.0,80.0,90.0};

    int index = 0;
    float dPosition[3] = {0.0, 0.0, 0.0};  // Posición deseada en grados
    int enableFlag = 0;

    bool select = 0;

    portTickType xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        if(xQueueReceive(MCtrl2Plan_QHandle,&enableFlag,portMAX_DELAY))
        {
            gioToggleBit(gioPORTA, 2);

            dPosition[0] = posiciones[index];
            dPosition[1] = posiciones[index];
            dPosition[2] = posiciones[index];

            xQueueSend(MCI2MCtrl_QHandle, dPosition, 10);

            if(select == 0)
                index++;
            else
                index--;

            if(index == 9 )
            {
                select = 1;
            }
            if(index == 0)
            {
                select = 0;
            }
        }
    }
}
/* USER CODE END */
