#ifndef H_CALIBRATION
#define H_CALIBRATION
#include "I2CControl.h"

void* Calibration_getSD_singlethread(void *cal);
void Calibration_getSD_multithread(I2CVariblesCali* i2c_valCali) ;
int Calibration_HMC5883L_singleThread(float* mag_offset, float* mag_gain);

#endif
