#ifndef H_PID
#define	H_PID

#define	POWER_MIN	1640
#define	POWER_MAX	3280
#define	POWER_LIMIT	3280
#define INTEG_LIMIT	40
typedef struct {
    float Kp_in, Ki_in, Kd_in;
    float Kp_out, Ki_out, Kd_out;
    float Kp_filter, Ki_filter;

    float angle_before[3];
    float angle_err[3];
    float angle_deriv[3];
    float angle_integ[3];
    float outP[3], outI[3], outD[3];
    int output[3];
} PIDControl;

void PID_init(PIDControl * pid, float* pid_setting);
void PID_update(PIDControl *pid, float *angle_expect, float *angle_measured, float* gyro, int *pwm, float *dt, unsigned int* power);

#endif
