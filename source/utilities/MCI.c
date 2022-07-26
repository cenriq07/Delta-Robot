/*
 * MCI.c
 *
 *  Created on: 3 jul. 2022
 *      Author: Enrique
 */
#include "utilities/MCI.h"

int i=0, j=0;

float a=9.0;
float b=18.0;
float h=6.0;
float r=12.0;

float p[3] = {1.0,2.0,0.0};
float c[3][3] = {{0.0,0.0,0.0},
                 {0.0,0.0,0.0},
                 {0.0,0.0,0.0}};

float phi[3] = {0,2.0944,4.1888};     // [0, 120, 240]'*(2*pi)/360;
float theta[3][3] = {{0.0,0.0,0.0},
                     {0.0,0.0,0.0},
                     {0.0,0.0,0.0}};

float cos_phi = 0.0, sin_phi = 0.0;
float sin_theta3i = 0.0;

float num = 0.0, den = 0.0;
float M = 0.0, N = 0.0;

void getMotorsAngle(float theta_m[], float p[])
{
    for(i=0; i<3; i++)
    {
        cos_phi = cos(phi[i]);
        sin_phi = sin(phi[i]);
        c[0][i] = cos_phi*p[0] + sin_phi*p[1] + (h-r);
        c[1][i] = -sin_phi*p[0] + cos_phi*p[1];
        c[2][i] = p[2] + sqrt(pow(b,2) - pow(r+a - h,2));

        theta[2][i] = acos(c[1][i]/b);
        sin_theta3i = sin(theta[2][i]);             // sin(theta(3,:))

        num = pow(c[0][i],2) + pow(c[1][i],2) + pow(c[2][i],2) - pow(a,2) - pow(b,2);
        den = 2*a*b*sin_theta3i;                    // 2*a*b*sin(theta(3,:))
        theta[1][i] = acos(num/den);                // theta(2, i) = acos(num/den);

        M = a + b*sin_theta3i*cos(theta[1][i]);     // a + b*sin(theta(3, i))*cos(theta(2, i))
        N = b*sin_theta3i*sin(theta[1][i]);         // b*sin(theta(3, i))*sin(theta(2, i));
        theta[0][i] = atan2(-c[0][i]*N + c[2][i]*M, c[2][i]*N + c[0][i]*M);

        theta_m[i] = theta[0][i];
    }
}
