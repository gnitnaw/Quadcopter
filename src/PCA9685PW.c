#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <bcm2835.h>
#include "I2CDevice.h"
#include "PCA9685PW.h"

extern int PCA9685PW_FREQ;
static char regaddr[64];
static char databuf[2];

void exchange(char *buf, int len) {
    int i;
    char tmp;
    for (i=0; i<len; ++i) {
        tmp = buf[i];
        buf[i] = buf[i+1];
        buf[i+1] = tmp;
        ++i;
    }
}

char baseReg(char pin) {
    return PCA9685PW_LED0_ON_L + pin*4;
}


void PCA9685PW_PWMReset(void);
void PCA9685PW_init(int i) {
    if (i != 0 && i != 1) {
        puts("MUST BE 0 or 1 !");
        return;
    }
    bcm2835_i2c_setSlaveAddress(PCA9685PW_ADDR);

    if (i==1) {
	int ret;
        regaddr[0] = PCA9685PW_MODE1;
        regaddr[1] = 0x00;
        bcm2835_i2c_write(regaddr, 2);

        regaddr[0] = PCA9685PW_MODE2;
        regaddr[1] = PCA9685PW__OUTDRV;
	bcm2835_i2c_write(regaddr, 2);

	if ((ret = PCA9685PWMFreq())!=0) {
            printf("Parameters of PCA9685PW are not correctly loaded -- code %d", ret);
            return;
	}

	PCA9685PW_PWMReset();
    }
}

int PCA9685PWMFreq(void) {
    int ret;
    PCA9685PW_init(0);
    PCA9685PW_FREQ = (PCA9685PW_FREQ > 1526 ? 1526 : (PCA9685PW_FREQ < 24 ? 24 : PCA9685PW_FREQ));
    int prescale = (int)(25000000.0f / (4096 * PCA9685PW_FREQ) - 0.5f);
//    printf("%02X\n", prescale);
    regaddr[0] = PCA9685PW_MODE1;
    regaddr[1] = 0;
    bcm2835_i2c_write(regaddr, 1);
    if ((ret = bcm2835_i2c_read(databuf, 1)) != BCM2835_I2C_REASON_OK) {
        return ret;
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


int pca9685PWMReadSingle(char pin, unsigned short *on, unsigned short *off) {
    PCA9685PW_init(0);
    regaddr[0] = baseReg(pin);
    bcm2835_i2c_write(regaddr, 1);
    if (bcm2835_i2c_read((char*)on, 2) != BCM2835_I2C_REASON_OK) return -1;
//    printf("On: %02X\n", *on&0xFFF);
    regaddr[0] += 2;
    bcm2835_i2c_write(regaddr, 1);
    if (bcm2835_i2c_read((char*)off, 2) != BCM2835_I2C_REASON_OK) return -1;
//    printf("Off: %02X\n", *off&0xFFF);
    return 0;
}

int pca9685PWMReadSingleOff(char pin, int *off) {
    PCA9685PW_init(0);
    regaddr[0] = PCA9685PW_LED0_ON_L + pin * 4;
    bcm2835_i2c_write(regaddr, 1);
    if (bcm2835_i2c_read((char*)off, 4) != BCM2835_I2C_REASON_OK) return -1;
    *off >>= 16;
    return 0;
}


int pca9685PWMReadMulti(char pin, unsigned short *data, int num) {  // if pin = 0, num = 3, data will be : on_0, off_0, on_1, off_1, on_2, off_2
    PCA9685PW_init(0);
    regaddr[0] = PCA9685PW_LED0_ON_L + pin * 4;
    bcm2835_i2c_write(regaddr, 1);
    if (bcm2835_i2c_read((char*)data, num*4) != BCM2835_I2C_REASON_OK) return -1;

    return 0;
}

int pca9685PWMReadMultiOff(char pin, int *data, int num) {  // if pin = 0, num = 3, data will be : off_0, off_1, off_2
    int i;
    PCA9685PW_init(0);
    regaddr[0] = PCA9685PW_LED0_ON_L + pin * 4;
    bcm2835_i2c_write(regaddr, 1);
    if (bcm2835_i2c_read((char*)data, num*4) != BCM2835_I2C_REASON_OK) return -1;
    for (i=0; i<num; ++i) data[i] >>= 16;
    return 0;
}

void pca9685PWMWriteSingle(char pin, unsigned short *on, unsigned short *off) {
    PCA9685PW_init(0);
    regaddr[0] = baseReg(pin);
    regaddr[1] = *on&0xFF;
    bcm2835_i2c_write(regaddr, 2);

    regaddr[0] += 1;
    regaddr[1] = *on >> 8;
    bcm2835_i2c_write(regaddr, 2);

    regaddr[0] += 1;
    regaddr[1] = *off&0xFF;
    bcm2835_i2c_write(regaddr, 2);

    regaddr[0] += 1;
    regaddr[1] = *off >> 8;
    bcm2835_i2c_write(regaddr, 2);
}

void pca9685PWMWriteSingleOff(char pin, int *off) {
    PCA9685PW_init(0);
    *off <<= 16;
    regaddr[0] = PCA9685PW_LED0_ON_L + pin * 4;
    memcpy(&regaddr[1], (char*)off, 4);
    bcm2835_i2c_write(regaddr, 5);
}

void pca9685PWMWriteMulti(char pin, unsigned short *data, int num) { // if pin = 0, num = 3, data will be : on_0, off_0, on_1, off_1, on_2, off_2
    PCA9685PW_init(0);
    regaddr[0] = PCA9685PW_LED0_ON_L + pin * 4;
    memcpy(&regaddr[1], (char*)data, num*4);

    bcm2835_i2c_write(regaddr, num*4+1);
}

void pca9685PWMWriteMultiOff(char pin, int *data, int num) { // if pin = 0, num = 3, data will be : on_0, off_0, on_1, off_1, on_2, off_2
    PCA9685PW_init(0);
    int i;
    for (i=0; i<num; ++i) data[i] <<=16;
    regaddr[0] = PCA9685PW_LED0_ON_L + pin * 4;
    memcpy(&regaddr[1], (char*)data, num*4);

    bcm2835_i2c_write(regaddr, num*4+1);
}


