/*
    Quadcopter -- I2CControl.c
    Copyright 2015 Wan-Ting CHEN (wanting@gmail.com)
    Description : Control all I2C device

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
#include "GY80.h"
#include "PCA9685PW.h"
#define	NPWM	4

extern int PWM_CHANNEL[NPWM];

static I2CRawDada raw_data;

//#include "Error.h"
pthread_mutex_t mutex_I2C;
static int pwm_power[NPWM];

static int ret;

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
    pthread_mutex_destroy(&i2c_var->mutex);
//    bcm2835_i2c_end();
    return 0;
}

int Renew_acclgyro(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0) ;
    i2c_var->ret[0] = (L3G4200D_getRawValue(raw_data.gyro) << 8);
    i2c_var->ret[0] += ADXL345_getRawValue(raw_data.accl);
    pthread_mutex_unlock (&mutex_I2C);

//    if (i2c_var->ret[0]!=0) {
//	return i2c_var->ret[0];
//    }

    while (pthread_mutex_trylock(&i2c_var->mutex) != 0);
    ADXL345_convertRawToReal(raw_data.accl, i2c_var->accl);
    L3G4200D_convertRawToReal(raw_data.gyro, i2c_var->gyro);
    pthread_mutex_unlock (&i2c_var->mutex);
//    delayMicroseconds(3000);

    return 0;
}

int Renew_magn(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C MAG1");
	delay(1);
    }
    HMC5883L_singleMeasurement();
    pthread_mutex_unlock (&mutex_I2C);
    delay(6);

    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C MAG2");
//	delay(1);
    }
    i2c_var->ret[1] = HMC5883L_getRawValue(raw_data.magn);
    pthread_mutex_unlock (&mutex_I2C);

    if (i2c_var->ret[1]!=0) return i2c_var->ret[1];

    while (pthread_mutex_trylock(&i2c_var->mutex) != 0) delayMicroseconds(100);
    HMC5883L_convertRawToReal(raw_data.magn, i2c_var->magn);
    pthread_mutex_unlock (&i2c_var->mutex);

    return 0;
}

void Trigger_magn(void) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0) ;
    HMC5883L_singleMeasurement();
    pthread_mutex_unlock (&mutex_I2C);
    delay(6);
}

void measureAndTrigger_magn(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0) ;
    int ret = HMC5883L_getRawValue(raw_data.magn);
    HMC5883L_singleMeasurement();
    pthread_mutex_unlock (&mutex_I2C);

    while (pthread_mutex_trylock(&i2c_var->mutex) != 0);
    HMC5883L_convertRawToReal(raw_data.magn, i2c_var->magn);
    i2c_var->ret[1] = ret;
    pthread_mutex_unlock (&i2c_var->mutex);

}

int Renew_magn_Origin(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C MAG");
	delay(1);
    }
    HMC5883L_singleMeasurement();
    pthread_mutex_unlock (&mutex_I2C);
    delay(8);

    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C MAG");
	delay(1);
    }
    i2c_var->ret[1] = HMC5883L_getRawValue(raw_data.magn);
    pthread_mutex_unlock (&mutex_I2C);

    if (i2c_var->ret[1]!=0) return i2c_var->ret[1];

    while (pthread_mutex_trylock(&i2c_var->mutex) != 0) delayMicroseconds(100);
    HMC5883L_convertRawToReal_Zero(raw_data.magn, i2c_var->magn);
    pthread_mutex_unlock (&i2c_var->mutex);

    return 0;
}


void Trigger_baroTemp(void) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0);
    BMP085_Trigger_UTemp();
    pthread_mutex_unlock (&mutex_I2C);
}

int measureTemp_triggerPres(void) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0);
    int ret = BMP085_getRawTemp(&raw_data.UT);
    BMP085_Trigger_UPressure();
    pthread_mutex_unlock (&mutex_I2C);
    return ret;
}

int measurePres_triggerTemp(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0);
    int ret = BMP085_getRawPressure(&raw_data.UP);
    BMP085_Trigger_UTemp();
    pthread_mutex_unlock (&mutex_I2C);

    while (pthread_mutex_trylock(&i2c_var->mutex) != 0) delayMicroseconds(100);
    BMP085_getRealData(&raw_data.UT, &raw_data.UP, &i2c_var->RTD, &i2c_var->RP, &i2c_var->altitude);
    i2c_var->ret[2] = ret;
    pthread_mutex_unlock (&i2c_var->mutex);
    return ret;
}

int Renew_baro(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C BAR0");
	delay(1);
    }
    BMP085_Trigger_UTemp();
    pthread_mutex_unlock (&mutex_I2C);
    delay(5);

    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C BAR1");
	delay(1);
    }
    i2c_var->ret[2] = BMP085_getRawTemp(&raw_data.UT);
    BMP085_Trigger_UPressure();
    pthread_mutex_unlock (&mutex_I2C);
    delayMicroseconds(25500);

    while (pthread_mutex_trylock(&mutex_I2C) != 0) {
//	puts("LOCK I2C BAR2");
	delay(1);
    }
    i2c_var->ret[2] = BMP085_getRawPressure(&raw_data.UP);
    pthread_mutex_unlock (&mutex_I2C);
    if (i2c_var->ret[2]!=0) return i2c_var->ret[2];

    while (pthread_mutex_trylock(&i2c_var->mutex) != 0) delayMicroseconds(100);
    BMP085_getRealData(&raw_data.UT, &raw_data.UP, &i2c_var->RTD, &i2c_var->RP, &i2c_var->altitude);
    pthread_mutex_unlock (&i2c_var->mutex);

    return 0;
}

void Renew_PWM(I2CVariables *i2c_var) {

    while (pthread_mutex_trylock(&i2c_var->mutex) != 0) {
        delayMicroseconds(100);
    }
    memcpy(pwm_power,i2c_var->PWM_power, sizeof(int)*4);

    pthread_mutex_unlock (&i2c_var->mutex);
    while (pthread_mutex_trylock(&mutex_I2C) != 0) delayMicroseconds(100);

    pca9685PWMWriteMultiOff(PWM_CHANNEL, pwm_power, NPWM);
    pthread_mutex_unlock (&mutex_I2C);

}

void PWM_init(I2CVariables *i2c_var) {
    int i;
    for (i=0; i<4; ++i) i2c_var->PWM_power[i]=POWER_MIN;
    Renew_PWM(i2c_var);
    delayMicroseconds(500000);
    for (i=0; i<4; ++i) i2c_var->PWM_power[i]=POWER_MAX;
    Renew_PWM(i2c_var);
    delayMicroseconds(400000);
    for (i=0; i<4; ++i) i2c_var->PWM_power[i]=POWER_MIN;
    Renew_PWM(i2c_var);
    delayMicroseconds(3000000);
}

void PWM_reset(I2CVariables *i2c_var) {
    while (pthread_mutex_trylock(&i2c_var->mutex) != 0);
    PCA9685PW_PWMReset();
    pthread_mutex_unlock (&i2c_var->mutex);
}
int Renew_PWM_read(I2CVariables *i2c_var) {
    int i;
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

