#ifndef H_DEVICE
#define H_DEVICE
#include "I2CControl.h"

typedef struct {
    float accl[3], gyro[3], magn[3], altitude;
    float accl_ref[3], gyro_corr[3], altitude_corr;
    float acc_magnitude, mag_magnitude, g_magnitude;
    float angle[3], angVel[3];
    float x[3], v[3], a[3];
} Drone_Status;


void Data_init(I2CVariblesCali *i2c_cali, Drone_Status *stat);
void Data_Copy(I2CVariables *i2c_val, I2CVariblesCali *i2c_cali, Drone_Status *stat, float* dt);

#endif
