/*
    Quadcopter -- AHRS.c
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
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include "Common.h"
#include "AHRS.h"
#define DEG_TO_RAD      (M_PI/180)
#define RAD_TO_DEG	(180/M_PI)
//#define G_VALUE		9.79
//#define ACCL_UNIT	0.004
//#define FILTER_YAW	1

void AHRS_init(AHRS* ahrs, float* angle, float* pi) {
    int i;
    memset(ahrs, 0, sizeof(AHRS));
    float dCos[3], dSin[3];
    for (i=0; i<3; ++i) {
        dCos[i] = cos(angle[i]*DEG_TO_RAD * 0.5);
        dSin[i] = sin(angle[i]*DEG_TO_RAD * 0.5);
    }

    ahrs->q[0] = dCos[0] * dCos[1] * dCos[2] + dSin[0] * dSin[1] * dSin[2];
    ahrs->q[1] = dSin[0] * dCos[1] * dCos[2] - dCos[0] * dSin[1] * dSin[2];
    ahrs->q[2] = dCos[0] * dSin[1] * dCos[2] + dSin[0] * dCos[1] * dSin[2];
    ahrs->q[3] = dCos[0] * dCos[1] * dSin[2] - dSin[0] * dSin[1] * dCos[2];

    ahrs->Kp = pi[0];
    ahrs->Ki = pi[1];
}

void AHRS_getAngle(AHRS* ahrs, float* angle) {
    angle[0] = atan2(2 * ahrs->q[2]*ahrs->q[3] + 2 * ahrs->q[0]*ahrs->q[1], -2 * ahrs->q[1]*ahrs->q[1] - 2 * ahrs->q[2]*ahrs->q[2] + 1) * RAD_TO_DEG; // roll
    angle[1] = asin(-2 * ahrs->q[1]*ahrs->q[3] + 2 * ahrs->q[0]*ahrs->q[2]) * RAD_TO_DEG; // pitch
    angle[2] = atan2(2 * ahrs->q[1]*ahrs->q[2] + 2 * ahrs->q[0]*ahrs->q[3], -2 * ahrs->q[2]*ahrs->q[2] - 2 * ahrs->q[3]*ahrs->q[3] + 1) * RAD_TO_DEG; // yaw
}

void AHRS_calculate_MagField_Earth(AHRS* ahrs, float* magn) {
    int i;
    ahrs->norm = Common_GetNorm(magn, 3);
    for (i=0; i<3; ++i) ahrs->mnorm[i] = magn[i] / ahrs->norm;

    ahrs->h[0] = 2* (ahrs->mnorm[0]*(0.5 - ahrs->q[2]*ahrs->q[2] - ahrs->q[3]*ahrs->q[3]) + ahrs->mnorm[1]*(ahrs->q[1]*ahrs->q[2] - ahrs->q[0]*ahrs->q[3])
		+ ahrs->mnorm[2]*(ahrs->q[1]*ahrs->q[3] + ahrs->q[0]*ahrs->q[2]) );
    ahrs->h[1] = 2* (ahrs->mnorm[0]*(ahrs->q[1]*ahrs->q[2] + ahrs->q[0]*ahrs->q[3]) + ahrs->mnorm[1]*(0.5 - ahrs->q[1]*ahrs->q[1] - ahrs->q[3]*ahrs->q[3])
		+ ahrs->mnorm[2]*(ahrs->q[2]*ahrs->q[3] - ahrs->q[0]*ahrs->q[1]) );
    ahrs->h[2] = 2* (ahrs->mnorm[0]*(ahrs->q[1]*ahrs->q[3] - ahrs->q[0]*ahrs->q[2]) + ahrs->mnorm[1]*(ahrs->q[2]*ahrs->q[3] + ahrs->q[0]*ahrs->q[1])
		+ ahrs->mnorm[2]*(0.5 - ahrs->q[1]*ahrs->q[1] - ahrs->q[2]*ahrs->q[2]) );

    ahrs->b[0] = Common_GetNorm(ahrs->h, 2);
    ahrs->b[2] = ahrs->h[2];
}

void AHRS_renew(AHRS* ahrs, float* deltaT, float* accl, float* gyro, float* magn) {
    int i;

    ahrs->half_dt = *deltaT/2;

    ahrs->norm = Common_GetNorm(accl, 3);
    for (i=0; i<3; ++i) ahrs->anorm[i] = accl[i] / ahrs->norm;
    ahrs->v[0] = 2*(ahrs->q[1]*ahrs->q[3] - ahrs->q[0]*ahrs->q[2]);
    ahrs->v[1] = 2*(ahrs->q[0]*ahrs->q[1] + ahrs->q[2]*ahrs->q[3]);
    ahrs->v[2] = 1 - 2*(ahrs->q[1]*ahrs->q[1] - ahrs->q[2]*ahrs->q[2]);
    ahrs->e[0] = (ahrs->anorm[1] * ahrs->v[2] - ahrs->anorm[2] * ahrs->v[1]);
    ahrs->e[1] = (ahrs->anorm[2] * ahrs->v[0] - ahrs->anorm[0] * ahrs->v[2]);
    ahrs->e[2] = (ahrs->anorm[0] * ahrs->v[1] - ahrs->anorm[1] * ahrs->v[0]);

    AHRS_calculate_MagField_Earth(ahrs, magn);

    ahrs->w[0] = 2*( ahrs->b[0]*(0.5 - ahrs->q[2]*ahrs->q[2] - ahrs->q[3]*ahrs->q[3]) + ahrs->b[2]*(ahrs->q[1]*ahrs->q[3] - ahrs->q[0]*ahrs->q[2]) );
    ahrs->w[1] = 2*( ahrs->b[0]*(ahrs->q[1]*ahrs->q[2] - ahrs->q[0]*ahrs->q[3]) + ahrs->b[2]*(ahrs->q[0]*ahrs->q[1] + ahrs->q[2]*ahrs->q[3]) );
    ahrs->w[2] = 2*( ahrs->b[0]*(ahrs->q[0]*ahrs->q[2] + ahrs->q[1]*ahrs->q[3]) + ahrs->b[2]*(0.5 - ahrs->q[1]*ahrs->q[1] - ahrs->q[2]*ahrs->q[2]) );

    ahrs->e[0] += (ahrs->mnorm[1]*ahrs->w[2] - ahrs->mnorm[2]*ahrs->w[1]);
    ahrs->e[1] += (ahrs->mnorm[2]*ahrs->w[0] - ahrs->mnorm[0]*ahrs->w[2]);
    ahrs->e[2] += (ahrs->mnorm[0]*ahrs->w[1] - ahrs->mnorm[1]*ahrs->w[0]);

    for (i=0; i<3; ++i) {
        ahrs->eInt[i] += ahrs->e[i] * ahrs->Ki;
        ahrs->angVel[i] = gyro[i] + ahrs->Kp * ahrs->e[i] + ahrs->eInt[i];
    }

    ahrs->qp[0] = ahrs->q[0] - ahrs->half_dt*(ahrs->q[1]*ahrs->angVel[0] + ahrs->q[2]*ahrs->angVel[1] + ahrs->q[3]*ahrs->angVel[2]);
    ahrs->qp[1] = ahrs->q[1] + ahrs->half_dt*(ahrs->q[0]*ahrs->angVel[0] + ahrs->q[2]*ahrs->angVel[2] - ahrs->q[3]*ahrs->angVel[1]);
    ahrs->qp[2] = ahrs->q[2] + ahrs->half_dt*(ahrs->q[0]*ahrs->angVel[1] - ahrs->q[1]*ahrs->angVel[2] + ahrs->q[3]*ahrs->angVel[0]);
    ahrs->qp[3] = ahrs->q[3] + ahrs->half_dt*(ahrs->q[0]*ahrs->angVel[2] + ahrs->q[1]*ahrs->angVel[1] - ahrs->q[2]*ahrs->angVel[0]);

    for (i=0; i<4; ++i) ahrs->q[i] = ahrs->qp[i];

    ahrs->norm = Common_GetNorm(ahrs->q, 4);

    for (i=0; i<4; ++i) ahrs->q[i] /= ahrs->norm;
/*
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
*/
}

