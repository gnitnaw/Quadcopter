#include <math.h>
#include <stdio.h>
#include <string.h>
#include "Device.h"
#include "Common.h"
#define	ACC_UNIT	9.8
#define ALTITUDE_FILTER	0.995

static int ret, i;
static float facter;

void Data_init(I2CVariblesCali *i2c_cali, Drone_Status *stat) {
    memset(stat, 0, sizeof(Drone_Status));
    stat->altitude_corr = 0.0;
    stat->altitude_zero = i2c_cali->altitude_offset;
    stat->g_magnitude = i2c_cali->accl_abs;
    stat->accl_err = Common_GetNorm(i2c_cali->accl_sd, 3);
    stat->angle[0] = atan2(i2c_cali->accl_offset[1], i2c_cali->accl_offset[2]) ; // roll
    stat->angle[1]  = -asin(i2c_cali->accl_offset[0]/i2c_cali->accl_abs);
    stat->angle[2] = acos(i2c_cali->magn_offset[1]/Common_GetNorm(i2c_cali->magn_offset, 2));
    for (i=0; i<3; ++i) stat->gyro_zero[i] = i2c_cali->gyro_offset[i];
    Quaternion_From_Stat(stat);
}

void Data_Copy(I2CVariables *i2c_val, Drone_Status *stat) {
    ret = 0;
    if (pthread_mutex_trylock(&i2c_val->mutex) == 0) {
	if (!i2c_val->ret[0]) {
	    memcpy(&stat->accl, &i2c_val->accl, sizeof(float)*6);
	} else {
	    if ((i2c_val->ret[0] >> 8)==0) {				// if gyro has problem
		ret |= (1<<8);						// gyro OK. accl has problem
		memcpy(&stat->gyro, &i2c_val->gyro, sizeof(float)*3);	// Only copy gyro
	    } else {
		if ((i2c_val->ret[0]&0xFF)==0) {			// gyro fail, check if accl OK
		    memcpy(&stat->accl, &i2c_val->accl, sizeof(float)*3);   // accl OK, copy accl
		} else ret |= (1<<9) ;					// Both fail
	    }
	}
	if (!i2c_val->ret[1]) {
	    memcpy(&stat->magn, &i2c_val->magn, sizeof(float)*3);
	} else {
	    ret |= (1<<10);
	}
        if (!i2c_val->ret[2]) {
	    stat->altitude = i2c_val->altitude;
        } else {
	    ret |= (1<<11);
	}
    }
    pthread_mutex_unlock (&i2c_val->mutex);

    // Check data quality

    stat->acc_magnitude = Common_GetNorm(stat->accl, 3);
    stat->mag_magnitude = Common_GetNorm(stat->magn, 2);
    if ( (facter = fabsf(stat->acc_magnitude - stat->g_magnitude) ) >  stat->accl_err * 3 ) {
	ret |= (1<<7);											// Means this accl value should not be used into filter
	if ( (facter=stat->acc_magnitude/stat->g_magnitude)>2.5 || facter < 0.5 ) ret |= (1<<6);			// Data may be incorrect
    }

    for (i=0; i<3; ++i) stat->gyro_corr[i] = stat->gyro[i] - stat->gyro_zero[i];

    if (stat->altitude>300) stat->altitude_corr = stat->altitude_corr * ALTITUDE_FILTER + (stat->altitude-stat->altitude_zero) * (1-ALTITUDE_FILTER);

    stat->status = ret;
}

void Data_Renew(Drone_Status *stat, float* deltaT) {
    Quaternion_renew_Drone(stat, deltaT);
}
