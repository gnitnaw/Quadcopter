#include "Device.h"
#ifndef H_QUATERNION
#define H_QUATERNION

void Quaternion_From_Stat(Drone_Status *stat);
void Euler_direct(Drone_Status *stat);
void Quaternion_calculate_MagField_Earth(Drone_Status *stat);
void Quaternion_renew_Drone(Drone_Status *stat, float* deltaT);
#endif
