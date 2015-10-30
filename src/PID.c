/*
    Quadcopter -- PID.c
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

#include <string.h>
#include <math.h>
#include "PID.h"

static int i;
//static float gyro_mean[] = {0,0,0};
void PID_init(PIDControl *pid, float* pid_setting) {
    memset(pid, 0, sizeof(PIDControl));
    memcpy(pid, pid_setting, sizeof(float)*8);
}

void PID_update(PIDControl *pid, float *angle_expect, float *angle_measured, float* gyro, int *pwm, float *dt, int* power) {
    for (i=0; i<3; ++i) {
	//gyro_mean[i] = gyro_mean[i] * 0.5 + gyro[i] * 0.5;
	pid->angle_err[i] = angle_expect[i] - angle_measured[i];
	//pid->angle_integ[i] += gyro[i] * *dt;
	pid->angle_integ[i] += pid->angle_err[i] * *dt;
/*
	if (pid->angle_integ[i] > INTEG_LIMIT) {
    	    pid->angle_integ[i] = INTEG_LIMIT;
  	} else if (pid->angle_integ[i] < -INTEG_LIMIT) {
    	    pid->angle_integ[i] = -INTEG_LIMIT;
  	}
*/
	pid->angle_deriv[i] = -gyro[i];
	pid->outP[i] = pid->Kp_out * pid->angle_err[i];
        pid->outI[i] = pid->Ki_out * pid->angle_integ[i];
        pid->outD[i] = pid->Kd_out * pid->angle_deriv[i];
        pid->output[i] = (int) round(pid->outP[i] + pid->outI[i] + pid->outD[i]);
    }
/*
    pwm[0] = (*power + pid->output[1] - pid->output[0] + pid->output[2] );    //M0
    pwm[1] = (*power - pid->output[1] - pid->output[0] - pid->output[2] );    //M1
    pwm[2] = (*power - pid->output[1] + pid->output[0] + pid->output[2] );    //M2
    pwm[3] = (*power + pid->output[1] + pid->output[0] - pid->output[2] );    //M3

*/
    pwm[0] = (*power - pid->output[0] + pid->output[1]);    //M0
    pwm[1] = (*power - pid->output[0] - pid->output[1]);    //M1
    pwm[2] = (*power + pid->output[0] - pid->output[1]);    //M2
    pwm[3] = (*power + pid->output[0] + pid->output[1]);    //M3
/*
    pwm[0] = (*power + pid->output[1]);    //M0
    pwm[1] = (*power - pid->output[1]);    //M1
    pwm[2] = (*power - pid->output[1]);    //M2
    pwm[3] = (*power + pid->output[1]);    //M3

    pwm[0] = (*power - pid->output[0]);    //M0
    pwm[1] = (*power - pid->output[0]);    //M1
    pwm[2] = (*power + pid->output[0]);    //M2
    pwm[3] = (*power + pid->output[0]);    //M3
*/
    for (i=0; i<4; ++i) {
	if (pwm[i] > POWER_MAX) {
	    pwm[i] = POWER_MAX;
	} else if (pwm[i] < POWER_MIN) {
	    pwm[i] = POWER_MIN;
	}
    }
}

