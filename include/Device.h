#ifndef H_DEVICE
#define H_DEVICE
#include "I2CControl.h"

typedef struct {
    float accl[3], gyro[3], magn[3], altitude;
    float accl_ref[3], gyro_zero[3], gyro_corr[3], altitude_corr, altitude_zero;
    float accl_err;
    float acc_magnitude, mag_magnitude, g_magnitude;
    float angle[3], angVel[3];
    float x[3], v[3], a[3];
    int status;
} Drone_Status;


void Data_init(I2CVariblesCali *i2c_cali, Drone_Status *stat);
void Data_Copy(I2CVariables *i2c_val, Drone_Status *stat);
void Data_Renew(Drone_Status *stat, float* deltaT);
void Quaternion_From_Stat(Drone_Status *stat);
void Quaternion_renew_Drone(Drone_Status *stat, float* deltaT);
#endif
