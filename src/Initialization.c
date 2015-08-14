#include <stdio.h>
#include <string.h>
#include <bcm2835.h>
#include "I2CDevice.h"
#define	NI2CITEM	5
#define LINESIZE        80

enum {
    ADXL345,
    L3G4200D,
    HMC5883L,
    BMP085,
    PCA9685PW
};

int init_all(void) {
    if (!bcm2835_init()) return -1;
    bcm2835_i2c_begin();
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);
    return 0;
}

int checkI2CDevice(void) {
    char s[LINESIZE];
    char check[] = "0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f";
    char exist[NI2CITEM];
    int i;
    for (i=0; i<NI2CITEM; ++i) {
        exist[i] = 0;
    }
    if (system("i2cdetect -y 1 > i2cdetect.dat") !=0) return -1;
    FILE *fp = fopen("i2cdetect.dat", "r");
    fgets(s, LINESIZE, fp);
    if (strstr(s, check) == NULL) return -2;
        while (!feof(fp)) {
        fgets(s, LINESIZE, fp);
        for (i=0; i<NI2CITEM; ++i) {
            if (!exist[i]) {
                switch (i) {
                    case ADXL345:
                        snprintf(check, sizeof(check), "%02x", ADXL345_ADDR);
                        break;
                    case L3G4200D:
                        snprintf(check, sizeof(check), "%02x", L3G4200D_ADDR);
                        break;
                    case HMC5883L:
                        snprintf(check, sizeof(check), "%02x", HMC5883L_ADDR);
                        break;
                    case BMP085:
                        snprintf(check, sizeof(check), "%02x", BMP085_ADDR);
                        break;
		    case PCA9685PW:
			snprintf(check, sizeof(check), "%02x", PCA9685PW_ADDR);
                        break;
                }
                if (strstr(&s[4], check) != NULL) exist[i]=1;
            }
        }
    }

    fclose(fp);

    for (i=0; i<NI2CITEM; ++i) {
        if (exist[i]==0) {
	    printf("%dth Item!\n", i);
	    return -1;
	}
    }


    return 0;
}
