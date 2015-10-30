/*
    Quadcopter -- PCA9685PW.c
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
#include "PCA9685PW_Address.h"
#include "PCA9685PW.h"
#include "Common.h"

extern int PCA9685PW_FREQ;
static char regaddr[41];
static char databuf[40];
static int ret_PWM;

int baseReg(int pin) {
    return PCA9685PW_LED0_ON_L + pin * 4;
}

int baseRegOff(int pin) {
    return PCA9685PW_LED0_OFF_L + pin * 4;
}

void PCA9685PW_switch(void) {
    bcm2835_i2c_setSlaveAddress(PCA9685PW_ADDR);
}
void PCA9685PW_init(void) {
    PCA9685PW_switch();

    regaddr[0] = PCA9685PW_MODE1;
    regaddr[1] = 0x20;
    bcm2835_i2c_write(regaddr, 2);

    regaddr[0] = PCA9685PW_MODE2;
    regaddr[1] = PCA9685PW__OUTDRV;
    bcm2835_i2c_write(regaddr, 2);

    if ((ret_PWM = PCA9685PWMFreq())!=0) {
        printf("Parameters of PCA9685PW are not correctly loaded -- code %d", ret_PWM);
        return;
    }

    PCA9685PW_PWMReset();
}

int PCA9685PWMFreq(void) {
    PCA9685PW_switch();
    PCA9685PW_FREQ = (PCA9685PW_FREQ > 1526 ? 1526 : (PCA9685PW_FREQ < 24 ? 24 : PCA9685PW_FREQ));
    int prescale = (int)(25000000.0f / (4096 * PCA9685PW_FREQ) - 0.5f);
    regaddr[0] = PCA9685PW_MODE1;
    regaddr[1] = 0;
    bcm2835_i2c_write(regaddr, 1);
    if ((ret_PWM = bcm2835_i2c_read(databuf, 1)) != BCM2835_I2C_REASON_OK) {
        return ret_PWM;
    }
    regaddr[1] = (databuf[0] & 0x7F) | PCA9685PW__SLEEP;		// Go to sleep
    bcm2835_i2c_write(regaddr, 2);

    regaddr[0] = PCA9685PW_PRE_SCALE;			// Set frequency
    regaddr[1] = prescale & 0xFF;
    bcm2835_i2c_write(regaddr, 2);

    regaddr[0] = PCA9685PW_MODE1;			// Restore the setting
    regaddr[1] = databuf[0];
    bcm2835_i2c_write(regaddr, 2);

    _usleep(5000);

    regaddr[1] = databuf[0] | PCA9685PW__RESTART;	// Restart
    bcm2835_i2c_write(regaddr, 2);

    return 0;
}

void PCA9685PW_PWMReset(void) {				// == All turn off
    PCA9685PW_switch();

    regaddr[0] = PCA9685PW_ALL_LED_ON_L;                       // Restore the setting
    regaddr[1] = 0x00;
    bcm2835_i2c_write(regaddr, 2);

    regaddr[0] = PCA9685PW_ALL_LED_ON_H;                       // Restore the setting
    regaddr[1] = 0x00;
    bcm2835_i2c_write(regaddr, 2);

    regaddr[0] = PCA9685PW_ALL_LED_OFF_L;                       // Restore the setting
    regaddr[1] = 0x00;
    bcm2835_i2c_write(regaddr, 2);

    regaddr[0] = PCA9685PW_ALL_LED_OFF_H;                       // Restore the setting
    regaddr[1] = 0x00;
    bcm2835_i2c_write(regaddr, 2);
}


int pca9685PWMReadSingle(int pin, int *data) {
    PCA9685PW_switch();
    regaddr[0] = baseReg(pin);

    bcm2835_i2c_write(regaddr, 1);
    if ( (ret_PWM=bcm2835_i2c_read(databuf, 4)) != BCM2835_I2C_REASON_OK) return ret_PWM;

    data[0] = (databuf[0] + ((int)databuf[1]<<8)) & 0xFFF;
    data[1] = (databuf[2] + ((int)databuf[3]<<8)) & 0xFFF;

    return 0;
}

int pca9685PWMReadSingleOff(int pin, int *off) {
    PCA9685PW_switch();
    regaddr[0] = baseReg(pin)+2;
    bcm2835_i2c_write(regaddr, 1);
    if ( (ret_PWM=bcm2835_i2c_read(databuf, 2)) != BCM2835_I2C_REASON_OK) return ret_PWM;

    *off = (databuf[0] + ((int)databuf[1]<<8)) & 0xFFF;
    return 0;
}


int pca9685PWMReadMulti(int* pin, int data[][2], int num) {
    int i;
//    PCA9685PW_switch();
    for (i=0; i<num; ++i) {
        pca9685PWMReadSingle(pin[i], data[i]);
    }

    return 0;
}

int pca9685PWMReadMultiOff(int *pin, int *data, int num) {  // if pin = 0, num = 3, data will be : off_0, off_1, off_2
    int i;
    for (i=0; i<num; ++i) {
        pca9685PWMReadSingleOff(pin[i], &data[i]);
    }

    return 0;

}

void pca9685PWMWriteSingle(int pin, int* data) {
    PCA9685PW_switch();
    regaddr[0] = baseReg(pin);
    regaddr[1] = data[0]&0xFF;
    regaddr[2] = data[0] >> 8;
    regaddr[3] = data[1]&0xFF;
    regaddr[4] = data[1] >> 8;
    bcm2835_i2c_write(regaddr, 5);
}

void pca9685PWMWriteSingleOff(int pin, int off) {
    PCA9685PW_switch();
    regaddr[0] = baseReg(pin)+2;
    regaddr[1] = off&0xFF;
    regaddr[2] = off >> 8;
    bcm2835_i2c_write(regaddr, 3);
}

void pca9685PWMWriteMulti(int *pin, int data[][2], int num) { // if pin = 0, num = 3, data will be : on_0, off_0, on_1, off_1, on_2, off_2
    int i;

    for (i=0; i<num; ++i) {
        pca9685PWMWriteSingle(pin[i], data[i]);
    }
}

void pca9685PWMWriteMultiOff(int *pin, int *data, int num) { // if pin = 0, num = 3, data will be : off_0, off_1, off_2
    int i;
    PCA9685PW_switch();
    for (i=0; i<num; ++i) {
        pca9685PWMWriteSingleOff(pin[i], data[i]);
    }

}

