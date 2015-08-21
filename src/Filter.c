#include <stdio.h>
#include <math.h>
#include "Filter.h"
#define N_SAMPLE_CALIBRATION    1000

static Kalman accl_K[3], gyro_K[3];

void Kalman_init(Kalman* k, float var, double noise) {
    int i;
    k->data_measured = var;
    k->data_noise = noise;
    k->data_updated = 0;
    k->P_EC = var * 0.1;
    k->K_factor = k->P_EC / (k->P_EC+k->data_noise);
    k->data_estimated = k->data_updated + k->K_factor * (k->data_measured-k->data_updated);
    k->P_EC_estimated = (1-k->K_factor)*k->P_EC;
}

void Kalman_init_all(float* accl, double* accl_sd, float* gyro, double* gyro_sd){
    int i;
    for (i=0; i<3; ++i) {
        Kalman_init(&accl_K[i], accl[i], accl_sd[i]);
        Kalman_init(&gyro_K[i], gyro[i], gyro_sd[i]);
    }
}

void Kalman_renew(Kalman* k, float* var_m, double* var_e ) {
    k->data_measured = *var_m;
    k->data_updated = k->data_estimated;
    k->P_EC = k->P_EC_estimated;
    k->K_factor = k->P_EC / (k->P_EC+k->data_noise);
    k->data_estimated = k->data_updated + k->K_factor * (k->data_measured-k->data_updated);
    k->P_EC_estimated = (1-k->K_factor)*k->P_EC;
    *var_e = k->data_estimated;
}

void Kalman_estimateAll(float* accl, float* gyro, double* accl_est, double* gyro_est) {
    int i;
    for (i=0; i<3; ++i) {
        Kalman_renew(&accl_K[i], &accl[i], &accl_est[i]);
        Kalman_renew(&gyro_K[i], &gyro[i], &gyro_est[i]);
    }
}


int Kalman_init_all_Calibration(float* accl, float* gyro, double* accl_offset, double* gyro_offset){
    int i, ret;
    double accl_mean[3], gyro_mean[3], accl_sd[3], gyro_sd[3], gyro_est[3], accl_est[3];
    if ( (ret=Calibration_getSD(accl, gyro, accl_mean, gyro_mean, accl_sd, gyro_sd))!=0 ) return ret ;
    Kalman_init_all(accl, accl_sd, gyro, gyro_sd);
    for (i=0; i<1000; ++i) {
        if ( (ret = getAccGyro(accl, gyro))!=0 ) return ret;
        Kalman_estimateAll(accl, gyro, accl_est, gyro_est);
    }
    if ( (ret=Calibration_getSD(accl, gyro, accl_mean, gyro_mean, accl_sd, gyro_sd))!=0 ) return ret ;

    for (i=0; i<3; ++i) {
        accl_offset[i] = accl_mean[i];
        gyro_offset[i] = gyro_mean[i];
    }
    printf("ACCL OFFSET : %f, %f, %f \n", accl_offset[0], accl_offset[1], accl_offset[2]);
    printf("GYRO OFFSET : %f, %f, %f \n", gyro_offset[0], gyro_offset[1], gyro_offset[2]);

    return 0;
}

