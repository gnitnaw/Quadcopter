#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "I2CControl.h"
//#include "Error.h"
pthread_mutex_t mutex_I2C = PTHREAD_MUTEX_INITIALIZER;
void I2CVariables_init(I2CVariables *i2c_var) {
    memset(i2c_var, sizeof(I2CVariables), 0);
    pthread_mutex_init (&i2c_var->mutex, NULL);
}

int I2CVariables_end(I2CVariables *i2c_var) {
    return pthread_mutex_destroy(&i2c_var->mutex);
}

int Renew_acclgyro(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0);
    while (pthread_mutex_trylock(&i2c_var->mutex) != 0);
    i2c_var->ret[0] = getAccGyro(i2c_var->accl, i2c_var->gyro);
    pthread_mutex_unlock (&i2c_var->mutex);
    pthread_mutex_unlock (&mutex_I2C);
    usleep(5000);

    return i2c_var->ret[0];
}

int Renew_magn(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0) usleep(500);
    HMC5883L_singleMeasurement();
    pthread_mutex_unlock (&mutex_I2C);
    usleep(8000);

    while (pthread_mutex_trylock(&mutex_I2C) == 0) usleep(500);
    while (pthread_mutex_trylock(&i2c_var->mutex) != 0) usleep(100);
    i2c_var->ret[1] = HMC5883L_getRealData_Direct(i2c_var->magn);
    pthread_mutex_unlock (&i2c_var->mutex);
    pthread_mutex_unlock (&mutex_I2C);

    return i2c_var->ret[1];
}

int Renew_magn_Origin(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0) usleep(500);
    HMC5883L_singleMeasurement();
    pthread_mutex_unlock (&mutex_I2C);
    usleep(6000);

    while (pthread_mutex_trylock(&mutex_I2C) == 0) usleep(500);
    while (pthread_mutex_trylock(&i2c_var->mutex) != 0) usleep(100);
    i2c_var->ret[1] = HMC5883L_getOriginalData_Direct(i2c_var->magn);
    pthread_mutex_unlock (&i2c_var->mutex);
    pthread_mutex_unlock (&mutex_I2C);

    return i2c_var->ret[1];
}


int Renew_baro(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0) usleep(1000);
    BMP085_Trigger_UTemp();
    pthread_mutex_unlock (&mutex_I2C);
    usleep(5000);

    while (pthread_mutex_trylock(&mutex_I2C) != 0) usleep(1000);
    BMP085_getRawTemp();
    BMP085_Trigger_UPressure();
    pthread_mutex_unlock (&mutex_I2C);
    usleep(25500);

    while (pthread_mutex_trylock(&mutex_I2C) != 0) usleep(1000);
    BMP085_getRawPressure();
    pthread_mutex_unlock (&mutex_I2C);

    while (pthread_mutex_trylock(&i2c_var->mutex) != 0);
    i2c_var->ret[2] = BMP085_getRealData(&i2c_var->RTD, &i2c_var->RP, &i2c_var->altitude);
    pthread_mutex_unlock (&i2c_var->mutex);

    return i2c_var->ret[2];
}

