/*
 * MCI.h
 *
 *  Created on: 3 jul. 2022
 *      Author: Enrique
 */

#ifndef INCLUDE_UTILITIES_MCI_H_
#define INCLUDE_UTILITIES_MCI_H_

#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#define N_PNTS    3                           // Numero de puntos destino

void getMotorsAngle(float p[], float theta_m[]);
void point2point(float pOrg[][3], float pDest[], int rowIdx);
void setAngleIncr(float Actual[], float Final[], float New[], int mCheck[]);

#endif /* INCLUDE_UTILITIES_MCI_H_ */
