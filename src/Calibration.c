#include <stdio.h>
#include <unistd.h>
#include <gsl/gsl_statistics.h>
#define N_SAMPLE_CALIBRATION	2000

int Calibration_getSD(float* var, float* var_mean, float* var_sd, int (*f)(float*)) {
    int i, j, ret;
    float var_cali[3][N_SAMPLE_CALIBRATION];

    for (i=0; i<N_SAMPLE_CALIBRATION; ++i) {
	usleep(3000);
        if ( (ret = f(var)) !=0 ) return ret;
        for (j=0; j<3; ++j) {
            var_cali[j][i] = (float) var[j];
        }
    }

    for (j=0; j<3; ++j) {
        var_mean[j] = gsl_stats_float_mean(&var_cali[j][0], 1, N_SAMPLE_CALIBRATION);
        var_sd[j] = gsl_stats_float_sd(&var_cali[j][0], 1, N_SAMPLE_CALIBRATION);
    }

    return 0;

}

int Calibration_getSD_withK(float* var, float* var_mean, float* var_sd, int (*f)(float*), char c) {
    int i, j, ret;
    float var_cali[3][N_SAMPLE_CALIBRATION], var_est[3];

    for (i=0; i<N_SAMPLE_CALIBRATION; ++i) {
	usleep(3000);
        if ( (ret = f(var)) !=0 ) return ret;
	Kalman_estimate(var, var_est, c);
        for (j=0; j<3; ++j) {
            var_cali[j][i] = var_est[j];
        }
    }

    for (j=0; j<3; ++j) {
        var_mean[j] = gsl_stats_float_mean(&var_cali[j][0], 1, N_SAMPLE_CALIBRATION);
        var_sd[j] = gsl_stats_float_sd(&var_cali[j][0], 1, N_SAMPLE_CALIBRATION);
    }

    return 0;

}

/*
int Calibration_getSD(float* accl, float* gyro, double* accl_mean, double* gyro_mean, double* accl_sd, double* gyro_sd) {
    int i, j, ret;
    double accl_cali[3][N_SAMPLE_CALIBRATION], gyro_cali[3][N_SAMPLE_CALIBRATION];
    for (i=0; i<N_SAMPLE_CALIBRATION; ++i) {
    	if ( (ret = getAccGyro(accl, gyro)) !=0 ) return ret;
	for (j=0; j<3; ++j) {
	    accl_cali[j][i] = (double) accl[j];
	    gyro_cali[j][i] = (double) gyro[j];
	}
    }

    for (j=0; j<3; ++j) {
	accl_mean[j] = gsl_stats_mean(&accl_cali[j][0], 1, N_SAMPLE_CALIBRATION);
	gyro_mean[j] = gsl_stats_mean(&gyro_cali[j][0], 1, N_SAMPLE_CALIBRATION);
	accl_sd[j] = gsl_stats_sd(&accl_cali[j][0], 1, N_SAMPLE_CALIBRATION);
	gyro_sd[j] = gsl_stats_sd(&gyro_cali[j][0], 1, N_SAMPLE_CALIBRATION);
    }

    return 0;
}

*/
int Calibration_HMC5883L(short* mag, float* mag_offset, float* mag_gain) {
    int i, j, ret;
    float mag_cali[3][N_SAMPLE_CALIBRATION], max[3], min[3];
    for (i=0; i<N_SAMPLE_CALIBRATION; ++i) {
	if ( (ret=HMC5883L_getRawValue(mag))!=0 ) return ret;
	printf("%d, %d, %d\n", mag[0], mag[1], mag[2]);
	for (j=0; j<3; ++j) mag_cali[j][i] = (float) mag[j];
	usleep(6600);
    }

    for (i=0; i<3; ++i) {
	gsl_stats_float_minmax (&min[i], &max[i], &mag_cali[i][0], 1, N_SAMPLE_CALIBRATION);
	mag_offset[i] = (max[i]+min[i])/2 ;
	printf("Max = %f, min = %f\n", max[i], min[i]);
	if (i==0) mag_gain[i] = 1.0;
	else {
	    mag_gain[i] = (max[0]-min[0])/(max[i]-min[i]);
	}
    }

    return 0;
}
