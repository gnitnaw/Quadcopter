/*
    Quadcopter -- I2CControl.c
    Copyright 2015 Wan-Ting CHEN (wanting@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <bcm2835.h>
#include "I2CControl.h"
#define	NPWM	4

extern int PWM_CHANNEL[NPWM];

int ADXL345_getRawValue(void);
void ADXL345_getRealData(float* acceleration);
int L3G4200D_getRawValue(void);
void L3G4200D_getRealData(float* angVel);
void HMC5883L_singleMeasurement(void);
int HMC5883L_getRawValue(void);
int HMC5883L_getRealData(float* magn);
void HMC5883L_getRealData_Direct(float* magn);
void HMC5883L_getOriginalData_Direct(float* magn);
int HMC5883L_dataReady(void);
void BMP085_Trigger_UTemp(void);
void BMP085_Trigger_UPressure(void);
int BMP085_getRawTemp(void);
int BMP085_getRawPressure(void);
void BMP085_getRealData(float *RTD, long *RP, float *altitude);
void getAccGyro(float *accl, float *gyro);


int PCA9685PWMFreq(void);
void PCA9685PW_PWMReset(void);
/*
int pca9685PWMReadSingle(int pin, int *data);
int pca9685PWMReadSingleOff(int pin, int *off);
int pca9685PWMReadMulti(int* pin, int data[][2], int num);
int pca9685PWMReadMultiOff(int* pin, int *data, int num);
void pca9685PWMWriteSingle(int pin, int* data);
void pca9685PWMWriteSingleOff(int pin, int off);
void pca9685PWMWriteMulti(int *pin, int data[][2], int num);
void pca9685PWMWriteMultiOff(int *pin, int *data, int num);
*/

//#include "Error.h"
pthread_mutex_t mutex_I2C;
static int pwm_power[NPWM];

static int i, ret, i_I2C;

extern float pid_setting[8]; 
void I2CVariables_init(I2CVariables *i2c_var) {
    memset(i2c_var, 0, sizeof(I2CVariables));
    PID_init(&i2c_var->pid, pid_setting);
//    memcpy(pwm_power,i2c_var->PWM_power, sizeof(int)*NPWM);
    pthread_mutex_init (&i2c_var->mutex, NULL);
}

int I2CVariables_end(I2CVariables *i2c_var) {
//    while (pthread_mutex_trylock(&mutex_I2C) != 0) delayMicroseconds(100);
    PCA9685PW_PWMReset();
//    pthread_mutex_unlock (&mutex_I2C);
//    pthread_mutex_destroy(&mutex_I2C);
    return pthread_mutex_destroy(&i2c_var->mutex);
}

int Renew_acclgyro(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C ACC");
//	usleep(500);
	delayMicroseconds(100);
    }
    i2c_var->ret[0] = (L3G4200D_getRawValue() << 8);
    i2c_var->ret[0] += ADXL345_getRawValue();
    pthread_mutex_unlock (&mutex_I2C);

    if (i2c_var->ret[0]!=0) {
//	usleep(1000);
	return i2c_var->ret[0];
    }

    while (pthread_mutex_trylock(&i2c_var->mutex) != 0) delayMicroseconds(100);//usleep(100);
    getAccGyro(i2c_var->accl, i2c_var->gyro);
    pthread_mutex_unlock (&i2c_var->mutex);
    delayMicroseconds(4000);

    return 0;
}

int Renew_magn(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C MAG1");
	//usleep(1000);
	delay(1);
    }
    HMC5883L_singleMeasurement();
    pthread_mutex_unlock (&mutex_I2C);
    //usleep(8000);
    delay(8);

    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C MAG2");
	//usleep(1000);
	delay(1);
    }
    i2c_var->ret[1] = HMC5883L_getRawValue();
    pthread_mutex_unlock (&mutex_I2C);

    if (i2c_var->ret[1]!=0) return i2c_var->ret[1];

    while (pthread_mutex_trylock(&i2c_var->mutex) != 0) delayMicroseconds(100); //usleep(100);
    HMC5883L_getRealData_Direct(i2c_var->magn);
    pthread_mutex_unlock (&i2c_var->mutex);

    return 0;
}

int Renew_magn_Origin(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C MAG");
//	usleep(1000);
	delay(1);
    }
    HMC5883L_singleMeasurement();
    pthread_mutex_unlock (&mutex_I2C);
//    usleep(8000);
    delay(8);

    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C MAG");
//	usleep(1000);
	delay(1);
    }
    i2c_var->ret[1] = HMC5883L_getRawValue();
    pthread_mutex_unlock (&mutex_I2C);

    if (i2c_var->ret[1]!=0) return i2c_var->ret[1];

    while (pthread_mutex_trylock(&i2c_var->mutex) != 0) delayMicroseconds(100); //usleep(100);
    HMC5883L_getOriginalData_Direct(i2c_var->magn);
    pthread_mutex_unlock (&i2c_var->mutex);

    return 0;
}


int Renew_baro(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C BAR0");
//	usleep(1000);
	delay(1);
    }
    BMP085_Trigger_UTemp();
    pthread_mutex_unlock (&mutex_I2C);
    delay(5);

    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C BAR1");
	delay(1);
    }
    i2c_var->ret[2] = BMP085_getRawTemp();
    BMP085_Trigger_UPressure();
    pthread_mutex_unlock (&mutex_I2C);
    delayMicroseconds(25500);

    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C BAR2");
//	usleep(1000);
	delay(1);
    }
    i2c_var->ret[2] = BMP085_getRawPressure();
    pthread_mutex_unlock (&mutex_I2C);
    if (i2c_var->ret[2]!=0) return i2c_var->ret[2];

    while (pthread_mutex_trylock(&i2c_var->mutex) != 0) delayMicroseconds(100);//usleep(100);
    BMP085_getRealData(&i2c_var->RTD, &i2c_var->RP, &i2c_var->altitude);
    pthread_mutex_unlock (&i2c_var->mutex);

    return 0;
}

void Renew_PWM(I2CVariables *i2c_var) {

    while (pthread_mutex_trylock(&i2c_var->mutex) != 0) {
        delayMicroseconds(100);
	//usleep(100);
    }
    memcpy(pwm_power,i2c_var->PWM_power, sizeof(int)*4);
    for (i_I2C=0; i_I2C<4; ++i_I2C) {
	pwm_power[i_I2C] = i2c_var->PWM_power[i_I2C];
    }

    pthread_mutex_unlock (&i2c_var->mutex);

    while (pthread_mutex_trylock(&mutex_I2C) != 0) delayMicroseconds(100);

    pca9685PWMWriteMultiOff(PWM_CHANNEL, pwm_power, NPWM);
    pthread_mutex_unlock (&mutex_I2C);

}

void PWM_init(I2CVariables *i2c_var) {
    for (i_I2C=0; i_I2C<4; ++i_I2C) i2c_var->PWM_power[i_I2C]=POWER_MAX;
    Renew_PWM(i2c_var);
    usleep(800000);
    for (i_I2C=0; i_I2C<4; ++i_I2C) i2c_var->PWM_power[i_I2C]=POWER_MIN;
    Renew_PWM(i2c_var);
}

int Renew_PWM_read(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0) delayMicroseconds(100);

    if ( (ret=pca9685PWMReadMultiOff(PWM_CHANNEL, pwm_power, NPWM))!=0) {
	return ret;
    }
    pthread_mutex_unlock (&mutex_I2C);

    while (pthread_mutex_trylock(&i2c_var->mutex) != 0) delayMicroseconds(100);

    for (i=0; i<4; ++i) {
        i2c_var->PWM_power[i] = pwm_power[i];
    }
//    memcpy(i2c_var->PWM_power, pwm_power, sizeof(int)*4);

    pthread_mutex_unlock (&i2c_var->mutex);

    return 0;
}

