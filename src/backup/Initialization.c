/*
    Quadcopter -- Initialization.c
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
#include <bcm2835.h>
#include "I2CControl.h"
#include "Error.h"

#define	NI2CITEM	5
#define LINESIZE        80

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

    ADXL345_init();
    L3G4200D_init();
    HMC5883L_init();
    BMP085_init();
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
