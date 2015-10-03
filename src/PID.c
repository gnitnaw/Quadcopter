#include <string.h>
#include <math.h>
#include "PID.h"

static int i;
void PID_init(PIDControl *pid, float* pid_setting) {
    memset(pid, 0, sizeof(PIDControl));
    memcpy(pid, pid_setting, sizeof(float)*8);
}

void PID_update(PIDControl *pid, float *angle_expect, float *angle_measured, float* gyro, int *pwm, float *dt, int* power) {
    for (i=0; i<3; ++i) {
	pid->angle_err[i] = angle_expect[i] - angle_measured[i];
	pid->angle_integ[i] += gyro[i] * *dt;
	if (pid->angle_integ[i] > INTEG_LIMIT) {
    	    pid->angle_integ[i] = INTEG_LIMIT;
  	} else if (pid->angle_integ[i] < -INTEG_LIMIT) {
    	    pid->angle_integ[i] = -INTEG_LIMIT;
  	}
	pid->angle_deriv[i] = -gyro[i];

	pid->outP[i] = pid->Kp_out * pid->angle_err[i];                                                                 //方便独立观察
        pid->outI[i] = pid->Ki_out * pid->angle_integ[i];
        pid->outD[i] = pid->Kd_out * pid->angle_deriv[i];

        pid->output[i] = (int) round(pid->outP[i] + pid->outI[i] + pid->outD[i]);
    }

//    pwm[2] = (*power - pid->output[0] - pid->output[1] - pid->output[2] );    //M3
//    pwm[0] = (*power + pid->output[0] + pid->output[1] - pid->output[2] );    //M1
//    pwm[3] = (*power - pid->output[0] + pid->output[1] + pid->output[2] );    //M4
//    pwm[1] = (*power + pid->output[0] - pid->output[1] + pid->output[2] );    //M2

    pwm[2] = (*power + pid->output[0] - pid->output[1] - pid->output[2] );    //M3
    pwm[0] = (*power - pid->output[0] + pid->output[1] - pid->output[2] );    //M1
    pwm[3] = (*power + pid->output[0] + pid->output[1] + pid->output[2] );    //M4
    pwm[1] = (*power - pid->output[0] - pid->output[1] + pid->output[2] );    //M2

}

