#ifndef H_DEVICE
#define H_DEVICE
#include "I2CControl.h"
#include "SPIControl.h"
//#include "EKF.h"
#include "Filter_Digital.h"
//#include "Filter_Kalman.h"
#include "AHRS.h"

typedef struct {
    I2CVariblesCali i2c_cali;
    I2CVariables i2c_var;
    SPIVariables spi_var;
    AHRS ahrs;
    float accl_est[3], gyro_est[3], magn_est[3];
    Filter_Digital filter_acc[3], filter_gyr[3], filter_mag[3];
    float gyro_corr[3], altitude_corr; //altitude_zero, gyro_zero[3];
    float acc_magnitude, mag_magnitude; //g_magnitude;
    float angle[3], angVel[3], yaw_real;
    int status;
} Drone_Status;

int Drone_init(Drone_Status *stat);
void Drone_end(Drone_Status *stat);
void Drone_Calibration(Drone_Status *stat);
void Drone_Calibration_printResult(Drone_Status *stat);
void Drone_Start(Drone_Status *stat);
void Drone_Renew(Drone_Status *stat, float* deltaT);
void Data_init(I2CVariblesCali *i2c_cali, Drone_Status *stat);
void Data_Copy(I2CVariables *i2c_val, Drone_Status *stat);
void Data_Renew(Drone_Status *stat, float* deltaT);
void Drone_NoiseFilter(Drone_Status *stat);
void Quaternion_From_Stat(Drone_Status *stat);
void Quaternion_renew_Drone(Drone_Status *stat, float* deltaT);
#endif
