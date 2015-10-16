/*
    Quadcopter -- Quaternion.c
    Copyright 2015 Wan-Ting CHEN (wanting@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
//#include "I2CControl.h"
#include "Device.h"
#include "Common.h"
#define DEG_TO_RAD      (M_PI/180)
#define RAD_TO_DEG	(180/M_PI)
#define G_VALUE		9.79
#define ACCL_UNIT	0.004
//#define FILTER_YAW	1
static float q[4];
static float half_dt;
static float norm;
static float anorm[3], mnorm[3];
static float v[3], h[3], b[3], w[3];
static float e[3];
static float eInt[3];
static float EulerAngle[3];

static int i,j;

extern float Kp, Ki, Kd;

float getAngle(float *val) {
    return acos(val[1]/sqrtf(pow(val[0],2) + pow(val[1],2)));
}

void Rotate_Vector(float A[][3], float* x, float *b) {
    for (i=0; i<3; ++i) {
	b[i] = 0.0;
	for (j=0; j<3; ++j) b[i] += A[i][j] * x[j];
    }
}

void Quaternion_Euler(Drone_Status *stat) {
//    EulerAngle[0] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1); // roll
//    EulerAngle[1] = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2]); // pitch
//    EulerAngle[2] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3]*q[3] + 1); // yaw
    stat->angle[0] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1) * RAD_TO_DEG; // roll
    stat->angle[1] = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2]) * RAD_TO_DEG; // pitch
//    stat->angle[2] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3]*q[3] + 1); // yaw
    stat->angle[2] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3]*q[3] + 1) * RAD_TO_DEG; // yaw
}

void Quaternion_From_EulerAngle(void) {
    float dCos[3], dSin[3];
    for (i=0; i<3; ++i) {
	dCos[i] = cos(EulerAngle[i]*DEG_TO_RAD * 0.5);
	dSin[i] = sin(EulerAngle[i]*DEG_TO_RAD * 0.5);
    }
    q[0] = dCos[0] * dCos[1] * dCos[2] + dSin[0] * dSin[1] * dSin[2];
    q[1] = dSin[0] * dCos[1] * dCos[2] - dCos[0] * dSin[1] * dSin[2];
    q[2] = dCos[0] * dSin[1] * dCos[2] + dSin[0] * dCos[1] * dSin[2];
    q[3] = dCos[0] * dCos[1] * dSin[2] - dSin[0] * dSin[1] * dCos[2];
}

void Quaternion_From_Stat(Drone_Status *stat) {
    float dCos[3], dSin[3];
    for (i=0; i<3; ++i) {
        dCos[i] = cos(stat->angle[i] * 0.5);
        dSin[i] = sin(stat->angle[i] * 0.5);
    }
    q[0] = dCos[0] * dCos[1] * dCos[2] + dSin[0] * dSin[1] * dSin[2];
    q[1] = dSin[0] * dCos[1] * dCos[2] - dCos[0] * dSin[1] * dSin[2];
    q[2] = dCos[0] * dSin[1] * dCos[2] + dSin[0] * dCos[1] * dSin[2];
    q[3] = dCos[0] * dCos[1] * dSin[2] - dSin[0] * dSin[1] * dCos[2];

    for (i=0; i<3; ++i) mnorm[i] = stat->i2c_cali.magn_offset[i] / stat->i2c_cali.magn_abs;
}

void Euler_direct(Drone_Status *stat) {
    stat->angle[0] = atan2(stat->i2c_var.accl[1], stat->i2c_var.accl[2]);               // roll
    stat->angle[1]  = -asin(stat->i2c_var.accl[0]/stat->acc_magnitude);                     // pitch
//    stat->angle[0] = atan2(stat->i2c_var.accl[1], sqrt(pow(stat->i2c_var.accl[0],2)+pow(stat->i2c_var.accl[2],2)) );
//    stat->angle[1]  = atan2(stat->i2c_var.accl[0], Common_GetNorm(&stat->i2c_var.accl[1], 2));

//    stat->angle[2] = acos(stat->i2c_cali.magn_offset[0]/Common_GetNorm(stat->i2c_cali.magn_offset, 2)); // yaw
}
void Quaternion_calculate_MagField_Earth(Drone_Status *stat) {
    for (i=0; i<3; ++i) mnorm[i] = stat->i2c_var.magn[i] / stat->mag_magnitude;

    h[0] = 2* (mnorm[0]*(0.5 - q[2]*q[2] - q[3]*q[3]) + mnorm[1]*(q[1]*q[2] - q[0]*q[3]) + mnorm[2]*(q[1]*q[3] + q[0]*q[2]) );
    h[1] = 2* (mnorm[0]*(q[1]*q[2] + q[0]*q[3]) + mnorm[1]*(0.5 - q[1]*q[1] - q[3]*q[3]) + mnorm[2]*(q[2]*q[3] - q[0]*q[1]) );
    h[2] = 2* (mnorm[0]*(q[1]*q[3] - q[0]*q[2]) + mnorm[1]*(q[2]*q[3] + q[0]*q[1]) + mnorm[2]*(0.5 - q[1]*q[1] - q[2]*q[2]) );
    //b[0] = sqrtf((h[0]*h[0]) + (h[1]*h[1]));
    b[0] = Common_GetNorm(h, 2);
    b[2] = h[2];
}

void Quaternion_renew_Drone(Drone_Status *stat, float* deltaT) {
    half_dt = *deltaT/2;
    // Part 1: verify if acc data can be used

//    if ( ((stat->status>>7)&1)==0 && ((stat->status>>8)&1)==0 ) {
    if ( (((stat->status)>>8)&1)==0 && (((stat->status)>>6)&1)==0) {
	for (i=0; i<3; ++i) anorm[i] = stat->i2c_var.accl[i] / stat->acc_magnitude;
    	v[0] = 2*(q[1]*q[3] - q[0]*q[2]);
    	v[1] = 2*(q[0]*q[1] + q[2]*q[3]);
//    	v[2] = pow(q[0],2) - pow(q[1],2) - pow(q[2],2) + pow(q[3],2);
	v[2] = 1 - 2*(q[1]*q[1] - q[2]*q[2]);
    	e[0] = (anorm[1] * v[2] - anorm[2] * v[1]);
    	e[1] = (anorm[2] * v[0] - anorm[0] * v[2]);
    	e[2] = (anorm[0] * v[1] - anorm[1] * v[0]);
    } else {
	for (i=0; i<3; ++i) e[i] = 0.0;
    }

    // Part 2 : verify if mag data can be used
    if ( ((stat->status>>10)&1)==0 ) {
	Quaternion_calculate_MagField_Earth(stat);
    	w[0] = 2*( b[0]*(0.5 - q[2]*q[2] - q[3]*q[3]) + b[2]*(q[1]*q[3] - q[0]*q[2]) );
    	w[1] = 2*( b[0]*(q[1]*q[2] - q[0]*q[3]) + b[2]*(q[0]*q[1] + q[2]*q[3]) );
    	w[2] = 2*( b[0]*(q[0]*q[2] + q[1]*q[3]) + b[2]*(0.5 - q[1]*q[1] - q[2]*q[2]) );

//	printf("mnorm:w : %f, %f, %f, %f, %f, %f\n", mnorm[0],w[0],mnorm[1],w[1],mnorm[2],w[2]);
    	e[0] += (mnorm[1]*w[2] - mnorm[2]*w[1]);
    	e[1] += (mnorm[2]*w[0] - mnorm[0]*w[2]);
    	e[2] += (mnorm[0]*w[1] - mnorm[1]*w[0]);
    }

    for (i=0; i<3; ++i) {
        eInt[i] += e[i] * Ki;
        stat->angVel[i] = stat->gyro_corr[i] + Kp * e[i] + eInt[i];
    }

    q[0] = q[0] - half_dt*(q[1]*stat->angVel[0] + q[2]*stat->angVel[1] + q[3]*stat->angVel[2]);
    q[1] = q[1] + half_dt*(q[0]*stat->angVel[0] + q[2]*stat->angVel[2] - q[3]*stat->angVel[1]);
    q[2] = q[2] + half_dt*(q[0]*stat->angVel[1] - q[1]*stat->angVel[2] + q[3]*stat->angVel[0]);
    q[3] = q[3] + half_dt*(q[0]*stat->angVel[2] + q[1]*stat->angVel[1] - q[2]*stat->angVel[0]);

    norm = Common_GetNorm(q, 4);

    for (i=0; i<4; ++i) q[i] /= norm;

//    stat->angle[0] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1); // roll
//    stat->angle[1] = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2]); // pitch
//    stat->angle[2] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3]*q[3] + 1); // yaw

    Quaternion_Euler(stat);
//    Euler_direct(stat);

    stat->accl_ref[0] = 2*( stat->i2c_var.accl[0]*(0.5-q[2]*q[2]-q[3]*q[3]) + stat->i2c_var.accl[1]*(q[1]*q[2]-q[0]*q[3]) + stat->i2c_var.accl[2]*(q[1]*q[3]+q[0]*q[2]) );
    stat->accl_ref[1] = 2*( stat->i2c_var.accl[0]*(q[1]*q[2]+q[0]*q[3]) + stat->i2c_var.accl[1]*(0.5-q[1]*q[1]-q[3]*q[3]) + stat->i2c_var.accl[2]*(q[2]*q[3]-q[0]*q[1]) );
    stat->accl_ref[2] = 2*( stat->i2c_var.accl[0]*(q[1]*q[3]-q[0]*q[2]) + stat->i2c_var.accl[1]*(q[2]*q[3]+q[0]*q[1]) + stat->i2c_var.accl[2]*(0.5-q[1]*q[1]-q[2]*q[2]) );

    for (i=0; i<2; ++i) {
	stat->a[i] = trunc(stat->accl_ref[i]/ACCL_UNIT) * ACCL_UNIT * G_VALUE;
	//stat->a[i] = stat->accl_ref[i] * G_VALUE;
    }
    stat->a[2] = trunc((stat->accl_ref[2]-stat->i2c_cali.accl_abs)/ACCL_UNIT) * ACCL_UNIT * G_VALUE;
    //stat->a[2] = (stat->accl_ref[2]-stat->i2c_cali.accl_abs) * G_VALUE + 0.001;
//    stat->a[0] += 0.001;
//    stat->a[1] -= 0.012;

    for (i=0; i<3; ++i){
	stat->x[i] += *deltaT * stat->v[i] + 0.5 * *deltaT * *deltaT * stat->a[i];
	stat->v[i] += *deltaT * stat->a[i];
    }

    stat->x[2] = stat->x[2] * (1-Ki*10) + stat->altitude_corr * Ki*10;
}

