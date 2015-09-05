#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "Filter.h"
#define N_SAMPLE_CALIBRATION    1000

static Kalman accl_K[3], gyro_K[3], magn_K[3];

void Kalman_init(Kalman* k, float var, float noise) {
    int i;
    k->data_measured = var;
    k->data_noise = noise;
    k->data_updated = 0;
    k->P_EC = var * 0.1;
    k->K_factor = k->P_EC / (k->P_EC+k->data_noise);
    k->data_estimated = k->data_updated + k->K_factor * (k->data_measured-k->data_updated);
    k->P_EC_estimated = (1-k->K_factor)*k->P_EC;
}

void Kalman_init_all(float* accl, float* accl_sd, float* gyro, float* gyro_sd, float* magn, float* magn_sd){
    int i;
    for (i=0; i<3; ++i) {
        Kalman_init(&accl_K[i], accl[i], accl_sd[i]);
        Kalman_init(&gyro_K[i], gyro[i], gyro_sd[i]);
	Kalman_init(&magn_K[i], magn[i], magn_sd[i]);
    }
}

void Kalman_renew(Kalman* k, float* var_m, float* var_e ) {
    k->data_measured = *var_m;
    k->data_updated = k->data_estimated;
    k->P_EC = k->P_EC_estimated;
    k->K_factor = k->P_EC / (k->P_EC+k->data_noise);
    k->data_estimated = k->data_updated + k->K_factor * (k->data_measured-k->data_updated);
    k->P_EC_estimated = (1-k->K_factor)*k->P_EC;
    *var_e = k->data_estimated;
}

void Kalman_estimate(float* var, float* var_est, char c) {
    int i;
    Kalman* k;
    if (c == 'A') {
	k = &accl_K[0];
    } else if (c == 'G') {
	k = &gyro_K[0];
    } else if (c == 'M') {
	k = &magn_K[0];
    }

    for (i=0; i<3; ++i) Kalman_renew(&k[i], &var[i], &var_est[i]);
}

void Kalman_estimateAll(float* accl, float* gyro, float* magn, float* accl_est, float* gyro_est, float* magn_est) {
/*
    int i;
    for (i=0; i<3; ++i) {
        Kalman_renew(&accl_K[i], &accl[i], &accl_est[i]);
        Kalman_renew(&gyro_K[i], &gyro[i], &gyro_est[i]);
        Kalman_renew(&magn_K[i], &magn[i], &magn_est[i]);
    }
*/
    Kalman_estimate(accl, accl_est, 'A');
    Kalman_estimate(gyro, gyro_est, 'G');
    Kalman_estimate(magn, magn_est, 'M');
}

int Kalman_init_Calibration(float* var, float* var_offset, char c){
    int i, ret;
    float var_mean[3], var_sd[3], var_est[3];
    int (*f)(float*) = 0;
    Kalman* k;
    if (c == 'A') {
        k = &accl_K[0];
	f = ADXL345_getRealData;
    } else if (c == 'G') {
        k = &gyro_K[0];
	f = L3G4200D_getRealData;
    } else if (c == 'M') {
        k = &magn_K[0];
	f = HMC5883L_getRealData;
    }

    if ( (ret=Calibration_getSD(var, var_mean, var_sd, f))!=0 ) return ret ;
    for (i=0; i<3; ++i) Kalman_init(&k[i], var[i], var_sd[i]);

    for (i=0; i<1000; ++i) {
	if ( (ret = f(var))!=0 ) return ret;
	Kalman_estimate(var, var_est, c);
	//printf("%f, %f, %f\n", var_est[0], var_est[1], var_est[2]);
    }

    if ( (ret=Calibration_getSD_withK(var, var_mean, var_sd, f, c))!=0 ) return ret ;

    for (i=0; i<3; ++i) {
        var_offset[i] = var_mean[i];
    }
    printf("OFFSET : %f, %f, %f \n", var_offset[0], var_offset[1], var_offset[2]);
    return 0;
}

int Kalman_init_all_Calibration(float* accl, float* gyro, float* magn, float* accl_offset, float* gyro_offset, float* magn_offset){
    int ret;
    if ( (ret=Kalman_init_Calibration(accl, accl_offset, 'A'))!=0 ) return ret;
    if ( (ret=Kalman_init_Calibration(gyro, gyro_offset, 'G'))!=0 ) return ret;
    if ( (ret=Kalman_init_Calibration(magn, magn_offset, 'M'))!=0 ) return ret;
    return 0;
}

