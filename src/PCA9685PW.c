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
#include "I2CControl.h"
#include "PCA9685PW.h"

extern int PCA9685PW_FREQ;
static char regaddr[2];
static char databuf[4];
static int i, ret_PWM;
int PCA9685PWMFreq(void);
void PCA9685PW_PWMReset(void);

int baseReg(int pin) {
    return PCA9685PW_LED0_ON_L + pin * 4;
}

int baseRegOff(int pin) {
    return PCA9685PW_LED0_OFF_L + pin * 4;
}

void PCA9685PW_init(int k) {
    if (k != 0 && k != 1) {
        puts("MUST BE 0 or 1 !");
        return;
    }
    bcm2835_i2c_setSlaveAddress(PCA9685PW_ADDR);

    if (k==1) {
        regaddr[0] = PCA9685PW_MODE1;
        regaddr[1] = 0x00;
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
}

int PCA9685PWMFreq(void) {
    PCA9685PW_init(0);
    PCA9685PW_FREQ = (PCA9685PW_FREQ > 1526 ? 1526 : (PCA9685PW_FREQ < 24 ? 24 : PCA9685PW_FREQ));
    int prescale = (int)(25000000.0f / (4096 * PCA9685PW_FREQ) - 0.5f);
//    printf("%02X\n", prescale);
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

    usleep(5000);

    regaddr[1] = databuf[0] | PCA9685PW__RESTART;	// Restart
    bcm2835_i2c_write(regaddr, 2);

    return 0;
}

void PCA9685PW_PWMReset(void) {				// == All turn off
    PCA9685PW_init(0);

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
    PCA9685PW_init(0);
    regaddr[0] = baseReg(pin);

    for (i=0; i<4; ++i) {
    	bcm2835_i2c_write(regaddr, 1);
    	if ( (ret_PWM=bcm2835_i2c_read(&databuf[i], 1)) != BCM2835_I2C_REASON_OK) return ret_PWM;
	++regaddr[0];
    }

    data[0] = (databuf[0] + ((int)databuf[1]<<8)) & 0xFFF;
    data[1] = (databuf[2] + ((int)databuf[3]<<8)) & 0xFFF;

    return 0;
}

int pca9685PWMReadSingleOff(int pin, int *off) {
    PCA9685PW_init(0);
    regaddr[0] = baseReg(pin)+2;
    for (i=0; i<2; ++i) {
        bcm2835_i2c_write(regaddr, 1);
        if ( (ret_PWM=bcm2835_i2c_read(&databuf[i], 1)) != BCM2835_I2C_REASON_OK) return ret_PWM;
        ++regaddr[0];
    }

    *off = (databuf[0] + ((int)databuf[1]<<8)) & 0xFFF;
    return 0;
}


int pca9685PWMReadMulti(int* pin, int data[][2], int num) {
    int j;
    PCA9685PW_init(0);

    for (i=0; i<num; ++i) {
	regaddr[0] = baseReg(pin[i]);
	for (j=0; j<4; ++j) {
	    bcm2835_i2c_write(regaddr, 1);
	    if ( (ret_PWM=bcm2835_i2c_read(&databuf[j], 1)) != BCM2835_I2C_REASON_OK) return ret_PWM;
            ++regaddr[0];
    	}

	data[i][0] = (databuf[0] + ((int)databuf[1]<<8)) & 0xFFF;
    	data[i][1] = (databuf[2] + ((int)databuf[3]<<8)) & 0xFFF;
    }

    return 0;
}

int pca9685PWMReadMultiOff(int *pin, int *data, int num) {  // if pin = 0, num = 3, data will be : off_0, off_1, off_2
    int j;
    PCA9685PW_init(0);

    for (i=0; i<num; ++i) {
	regaddr[0] = baseReg(pin[i])+2;
        for (j=0; j<2; ++j) {
            bcm2835_i2c_write(regaddr, 1);
            if ( (ret_PWM=bcm2835_i2c_read(&databuf[j], 1)) != BCM2835_I2C_REASON_OK) return ret_PWM;
            ++regaddr[0];
        }
        data[i] = (databuf[0] + ((int)databuf[1]<<8)) & 0xFFF;
    }

    return 0;
}

void pca9685PWMWriteSingle(int pin, int* data) {
    PCA9685PW_init(0);
    regaddr[0] = baseReg(pin);
    regaddr[1] = data[0]&0xFF;
    bcm2835_i2c_write(regaddr, 2);

    regaddr[0] += 1;
    regaddr[1] = data[0] >> 8;
    bcm2835_i2c_write(regaddr, 2);

    regaddr[0] += 1;
    regaddr[1] = data[1]&0xFF;
    bcm2835_i2c_write(regaddr, 2);

    regaddr[0] += 1;
    regaddr[1] = data[1] >> 8;
    bcm2835_i2c_write(regaddr, 2);
}

void pca9685PWMWriteSingleOff(int pin, int off) {
    PCA9685PW_init(0);

    regaddr[0] = baseReg(pin)+2;
    regaddr[1] = off&0xFF;
    bcm2835_i2c_write(regaddr, 2);

    regaddr[0] += 1;
    regaddr[1] = off >> 8;
    bcm2835_i2c_write(regaddr, 2);

}

void pca9685PWMWriteMulti(int *pin, int data[][2], int num) { // if pin = 0, num = 3, data will be : on_0, off_0, on_1, off_1, on_2, off_2
    int j;

    PCA9685PW_init(0);

    for (i=0; i<num; ++i) {
	regaddr[0] = baseReg(pin[i]);
        for (j=0; j<2; ++j) {
	    regaddr[1] = data[i][j] &0xFF;
            bcm2835_i2c_write(regaddr, 2);
	    regaddr[0]++;
            regaddr[1] = data[i][j] >>8;
            bcm2835_i2c_write(regaddr, 2);
	    regaddr[0]++;
        }
    }
}

void pca9685PWMWriteMultiOff(int *pin, int *data, int num) { // if pin = 0, num = 3, data will be : off_0, off_1, off_2
    PCA9685PW_init(0);

    for (i=0; i<num; ++i) {
	regaddr[0] = baseReg(pin[i])+2;
        regaddr[1] = data[i] &0xFF;
        bcm2835_i2c_write(regaddr, 2);
        regaddr[0]++;
        regaddr[1] = data[i] >>8;
        bcm2835_i2c_write(regaddr, 2);
    }

}

