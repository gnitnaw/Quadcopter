#include <pthread.h>
pthread_mutex_t mutex_I2C = PTHREAD_MUTEX_INITIALIZER;
static int ret;
int Renew_acclgyro(float *accl, float *gyro, pthread_mutex_t* mutex) {
    if (pthread_mutex_trylock(&mutex_I2C) == 0) {
	while (pthread_mutex_trylock(mutex) != 0);
	if ( (ret=getAccGyro(accl, gyro)) !=0 ) return ret;
	pthread_mutex_unlock (mutex);
        pthread_mutex_unlock (&mutex_I2C);
        usleep(3000);
    }
}

int Renew_magn(float *magn, pthread_mutex_t* mutex) {
    if (pthread_mutex_trylock(&mutex_I2C) == 0) {
    	while (pthread_mutex_trylock(mutex) != 0);
        if (  (ret=HMC5883L_getRealData_Direct(magn)) !=0 ) return ret;
	pthread_mutex_unlock (mutex);
        HMC5883L_singleMeasurement();
        pthread_mutex_unlock (&mutex_I2C);
        usleep(6500);
    }
}

int Renew_baro(float *RTD, float *RP, float *altitude, pthread_mutex_t* mutex) {
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

    while (pthread_mutex_trylock(mutex) != 0);
    if ( (ret=BMP085_getRealData(&RTD, &RP, &altitude)) !=0) return ret;
    pthread_mutex_unlock (mutex);
}

