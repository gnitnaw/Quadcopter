/*
    Quadcopter -- Calibration.c
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


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fgsl/gsl_statistics.h>
#include "Common.h"
#include "Calibration.h"
#define N_SAMPLE_CALIBRATION	2000
#define NUM_CALI_THREADS 3

static unsigned int thread_count;

void Calibration_getSD_singlethread(void *cal) {
    int i, j, nItem=1, nSample=N_SAMPLE_CALIBRATION, SD_Check=0;
    volatile int ret;
    float* var=0;
    int (*f)(I2CVariables *) = 0;
    I2CCaliThread* i2c_caliThread = (I2CCaliThread*) cal;
    if (i2c_caliThread->c == 'A') {
        f = Renew_acclgyro;
	var = (float*) &i2c_caliThread->i2c_var->accl;
	nItem = 6;
	nSample = N_SAMPLE_CALIBRATION;
    } else if (i2c_caliThread->c == 'M') {
        f = Renew_magn;
	var = (float*) &i2c_caliThread->i2c_var->magn;
	nItem = 3;
	nSample = N_SAMPLE_CALIBRATION / 2;
    } else if (i2c_caliThread->c == 'B') {
        f = Renew_baro;
	var = (float*) &i2c_caliThread->i2c_var->altitude;
	nItem = 1;
	nSample = N_SAMPLE_CALIBRATION / 20;
	SD_Check = 1;
    }

    float **var_cali = (float**) malloc(sizeof(float*)*nItem);
    for (j=0; j<nItem; ++j) var_cali[j] = (float*) malloc(sizeof(float)*nSample);
    for (i=0; i<nSample; ++i) {
        if ( (ret= f(i2c_caliThread->i2c_var)) !=0 ) {
	    printf("%dth IO error: %c, %d\n", i, i2c_caliThread->c, ret);
	}

        for (j=0; j<nItem; ++j) {
            var_cali[j][i] = var[j];
        }

	if (ret!=0) --i;
    }

    for (j=0; j<nItem; ++j) {
        i2c_caliThread->mean[j] = (float) gsl_stats_mean(&var_cali[j][0], 1, nSample);
        i2c_caliThread->sd[j] = (float) gsl_stats_sd(&var_cali[j][0], 1, nSample);
	free(var_cali[j]);
    }

    free(var_cali);

    if (SD_Check) {
	if (i2c_caliThread->sd[0]/i2c_caliThread->mean[0] > 0.01) {
	    puts("Need to do it again");
	    Calibration_getSD_singlethread(cal);
	    return;
	}
    }
    __sync_fetch_and_sub(&thread_count,1);
}

void Calibration_getSD_multithread(I2CVariblesCali* i2c_valCali) {
    I2CVariables i2c_var;
    I2CVariables_init(&i2c_var);
    thread_count = NUM_CALI_THREADS;
    pthread_t thread_i2c[NUM_CALI_THREADS];
    I2CCaliThread cali[NUM_CALI_THREADS];
    pthread_mutex_init (&i2c_var.mutex, NULL);

    cali[0].i2c_var = &i2c_var;
    cali[0].mean = (float*) i2c_valCali->accl_offset;
    cali[0].sd = (float*) i2c_valCali->accl_sd;
    cali[0].c = 'A';
    pthread_create(&thread_i2c[0], NULL, (void*) Calibration_getSD_singlethread, (void*) &cali[0]);
//    pthread_join(thread_i2c[0],NULL);

    cali[1].i2c_var = &i2c_var;
    cali[1].mean = (float*) i2c_valCali->magn_offset;
    cali[1].sd = (float*) i2c_valCali->magn_sd;
    cali[1].c = 'M';
    pthread_create(&thread_i2c[1], NULL, (void*) Calibration_getSD_singlethread, (void*) &cali[1]);

    cali[2].i2c_var = &i2c_var;
    cali[2].mean = (float*) &i2c_valCali->altitude_offset;
    cali[2].sd = (float*) &i2c_valCali->altitude_sd;
    cali[2].c = 'B';
    pthread_create(&thread_i2c[2], NULL, (void*) Calibration_getSD_singlethread, (void*) &cali[2]);
//    pthread_join(thread_i2c[2],NULL);

    do {
	__sync_synchronize();
	_usleep(1000000);
    } while (thread_count);

//    i2c_valCali->accl_abs = 	sqrtf(i2c_valCali->accl_offset[0]*i2c_valCali->accl_offset[0]
//				+i2c_valCali->accl_offset[1]*i2c_valCali->accl_offset[1]
//				+i2c_valCali->accl_offset[2]*i2c_valCali->accl_offset[2]);

    i2c_valCali->accl_abs = Common_GetNorm(i2c_valCali->accl_offset, 3);
    i2c_valCali->magn_abs = Common_GetNorm(i2c_valCali->magn_offset, 3);

    I2CVariables_end(&i2c_var);
}

int Calibration_HMC5883L_singleThread(float* mag_offset, float* mag_gain) {
    I2CVariables i2c_var;
    int i, j, ret;
    I2CVariables_init(&i2c_var);

    float max[3], min[3];
    for (i=0; i<3; ++i) {
	max[i] = 0.0;
	min[i] = 0.0;
    }
    for (i=0; i<N_SAMPLE_CALIBRATION; ++i) {
        if ( (ret=Renew_magn_Origin(&i2c_var))!=0 ) return ret;
        printf("%f, %f, %f\n", i2c_var.magn[0], i2c_var.magn[1], i2c_var.magn[2]);
        for (j=0; j<3; ++j) {
	    if (i2c_var.magn[j] > max[j]) max[j] = i2c_var.magn[j];
	    else if (i2c_var.magn[j] < min[j]) min[j] = i2c_var.magn[j];
	}
	_usleep(100000);
    }

    for (i=0; i<3; ++i) {
        mag_offset[i] = (max[i]+min[i])/2 ;
        printf("Max = %f, min = %f\n", max[i], min[i]);
        if (i==0) mag_gain[i] = 1.0;
        else {
            mag_gain[i] = (max[0]-min[0])/(max[i]-min[i]);
        }
    }

    I2CVariables_end(&i2c_var);
    return 0;

}

int Calibration_getSD(float* var, float* var_mean, float* var_sd, int (*f)(float*)) {
    int i, j, ret;
    float var_cali[3][N_SAMPLE_CALIBRATION];

    for (i=0; i<N_SAMPLE_CALIBRATION; ++i) {
	_usleep(3000);
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
/*
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
*/
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

