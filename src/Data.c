#include <math.h>
#include <stdio.h>
#include <string.h>
#include "I2CControl.h"
#include "Device.h"
#define	ACC_UNIT	9.8
#define ALTITUDE_FILTER	0.95

static int ret, i;
static float acc_norm, facter;

void Data_init(I2CVariblesCali *i2c_cali, Drone_Status *stat) {
    memset(stat, 0, sizeof(Drone_Status));
    stat->altitude_corr = i2c_cali->altitude_offset;

    angle[0] = atan2(i2c_cali->accl_offset[1], i2c_cali->accl_offset[2]) ; // roll
    angle[1]  = -asin(i2c_cali->accl_offset[0]/i2c_cali->accl_abs);
    angle[2] = acos(i2c_cali->magn_offset[1]/Common_GetNorm(&i2c_cali->magn_offset, 2);
}

int Data_Copy(I2CVariables *i2c_val, I2CVariblesCali *i2c_cali, Drone_Status *stat, float* dt) {
    ret = 0;
    if (pthread_mutex_trylock(&i2c_val->mutex) == 0) {
	if (!i2c_val->ret[0]) {
	    memcpy(&stat->accl, &i2c_val->accl, sizeof(float)*6);
	} else {
	    ret <<= 4;
	}
	if (!i2c_val->ret[1]) {
	    memcpy(&stat->magn, &i2c_val->magn, sizeof(float)*3);
	} else {
	    ret <<= 5;
	}
        if (!i2c_val->ret[2]) {
	    stat->altitude = i2c_val->altitude;
        } else {
	    ret <<= 6;
	}
    }
    pthread_mutex_unlock (&i2c_val->mutex);

    // Check data quality

    acc_norm = Common_GetNorm(stat->accl, 3);
    if ( (facter = fabsf(acc_norm - i2c_cali->accl_abs) ) >  i2c_cali->accl_sd[2] * 3 ) {
	ret <<= 3;											// Means this accl value should not be used into filter
	if ( (facter=acc_norm/i2c_cali->accl_abs)>2.5 || facter < 0.5 ) ret <<= 2;			// Data may be incorrect
    }

    for (i=0; i<3; ++i) stat->gyro_corr[i] = stat->gyro[i] - i2c_cali->gyro_offset[i];

    stat->altitude_corr = stat->altitude_corr * ALTITUDE_FILTER + stat->altitude * (1-ALTITUDE_FILTER);
}
