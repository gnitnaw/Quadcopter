#include <stdio.h>
#include <string.h>
#include <bcm2835.h>
#include "I2CControl.h"
#include "Error.h"

#define	NI2CITEM	5
#define LINESIZE        80

//extern void ADXL345_init(int i);
//extern void L3G4200D_init(int i);
//extern void HMC5883L_init(int i);
//extern void BMP085_init(int i);
//extern void PCA9685PW_init(int i);

enum {
    ADXL345,
    L3G4200D,
    HMC5883L,
    BMP085,
    PCA9685PW
};

int init_all(I2CVariables *i2c_var) {
    if (!bcm2835_init()) return ERROR_BCM2835_INIT;
    bcm2835_i2c_begin();
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);         // 400 kHz

    ADXL345_init(1);
    L3G4200D_init(1);
    HMC5883L_init(1);
    BMP085_init(1);
    PCA9685PW_init(1);

    I2CVariables_init(i2c_var);
    return 0;
}

int end_all(I2CVariables *i2c_var) {
    bcm2835_i2c_end();
    return I2CVariables_end(i2c_var);
}

/*
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
	    return -10-i;
	}
    }


    return 0;
}

*/
