#ifndef H_I2CCONTROL
#define H_I2CCONTROL

#include <pthread.h>
#include "GY80.h"
#include "PID.h"
#include "PCA9685PW.h"

typedef struct {
    short accl[3], gyro[3], magn[3];
    long UT, UP;
} I2CRawDada;

typedef struct {
    pthread_mutex_t mutex;
    float accl[3], gyro[3], magn[3];
    float RTD, altitude;
    long RP;
    int ret[3];
    PIDControl pid;
    int PWM_power[4];
} I2CVariables;

typedef struct {
    float accl_offset[3], gyro_offset[3], magn_offset[3], altitude_offset;
    float accl_sd[3], gyro_sd[3], magn_sd[3], altitude_sd;
    float accl_abs, magn_abs;
} I2CVariblesCali;

typedef struct {
    I2CVariables *i2c_var;
    float *mean;
    float *sd;
    char c;
} I2CCaliThread;


void I2CVariables_init(I2CVariables *i2c_var);
int I2CVariables_end(I2CVariables *i2c_var);
int Renew_acclgyro(I2CVariables *i2c_var);
int Renew_acclgyro_wait(I2CVariables *i2c_var);
int Renew_magn(I2CVariables *i2c_var);
int Renew_magn_Origin(I2CVariables *i2c_var);
void Trigger_baroTemp(void) ;
int measureTemp_triggerPres(void) ;
int measurePres_triggerTemp(I2CVariables *i2c_var) ;
void Trigger_magn(void) ;
void measureAndTrigger_magn(I2CVariables *i2c_var) ;
int Renew_baro(I2CVariables *i2c_var);
void PWM_init(I2CVariables *i2c_var);
void Renew_PWM(I2CVariables *i2c_var);
int Renew_PWM_read(I2CVariables *i2c_var);
void PWM_reset(I2CVariables *i2c_var);

#endif
