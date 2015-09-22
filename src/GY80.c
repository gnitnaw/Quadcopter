/*
    Quadcopter -- GY80.c
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


#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <bcm2835.h>
#include "GY80.h"
//#include "Error.h"

#define	OSRS		3
#define P0              101325
//#define	ADXL345_UNIT	0.0392266	// Unit of ADXL345 is 4mg
#define ADXL345_UNIT    0.004       // Unit of ADXL345 is 4mg
#define L3G4200D_UNIT	0.00875		// Unit of L3G4200D when range = 250 dps
#define DEG_TO_RAD	(M_PI/180)
#define HMC5883L_RESOLUTION	0.92
//#define I2C_DEBUG

extern int ADXL345_RATE;
extern int L3G4200D_RATE;
extern int L3G4200D_RANGE;
extern int HMC5883L_RATE;

extern float mag_offset[3];
extern float mag_gain[3];

struct BMP085_Parameters {
    short AC1;
    short AC2;
    short AC3;
    unsigned short AC4;
    unsigned short AC5;
    unsigned short AC6;
    short B1;
    short B2;
    short MB;
    short MC;
    short MD;
};

static struct BMP085_Parameters Para_BMP085;
static char regaddr[2], databuf[3];
volatile static short accl[3], gyro[3], mag[3];
volatile static long UP, UT;
static short tmpMag;
static int ret_acc, ret_gyr, ret_mag, ret_bar;
static int iMag, iAcc, iGyr, ie;

void exchange(char *buf, int len) {
    char tmp;
    for (ie=0; ie<len; ++ie) {
        tmp = buf[ie];
        buf[ie] = buf[ie+1];
        buf[ie+1] = tmp;
        ++ie;
    }
}

void ADXL345_init(int k) {
    if (k != 0 && k != 1) {
	puts("MUST BE 0 or 1 !");
	return;
    }
    bcm2835_i2c_setSlaveAddress(ADXL345_ADDR);

    if (k==1) {
        regaddr[0] = ADXL345_POWER_CTL;			// Standby
        regaddr[1] = 0x00;
        bcm2835_i2c_write(regaddr,2);

        regaddr[0] = ADXL345_DATA_FORMAT;		// Full resolution
        regaddr[1] = 0x0A;
        bcm2835_i2c_write(regaddr,2);

        regaddr[0] = ADXL345_BW_RATE;			// Sampling rate
	switch (ADXL345_RATE) {
	    case 100 :
		regaddr[1] = 0x0A;
		break;
	    case 200 :
		regaddr[1] = 0x0B;
		break;
	    case 400 :
		regaddr[1] = 0x0C;
		break;
	    case 800 :
                regaddr[1] = 0x0D;
                break;
            case 1600 :
                regaddr[1] = 0x0E;
                break;
            case 3200 :
                regaddr[1] = 0x0F;
                break;
	    default :
		regaddr[1] = 0x0C;
	}
        bcm2835_i2c_write(regaddr,2);

        regaddr[0] = ADXL345_FIFO_CTL;			// by-Pass mode
        regaddr[1] = 0x00;
        bcm2835_i2c_write(regaddr,2);

        regaddr[0] = ADXL345_POWER_CTL;			// Switch ON
        regaddr[1] = 0x08;
        bcm2835_i2c_write(regaddr,2);
    }

}

int ADXL345_getRawValue(void) {
    ADXL345_init(0);
    regaddr[0] = ADXL345_DATAX0;
    bcm2835_i2c_write(regaddr, 1);
    if ( (ret_acc = bcm2835_i2c_read((char*)accl, 6)) != BCM2835_I2C_REASON_OK) return ret_acc;
    return 0;
}

void ADXL345_getRealData(float* acceleration) {
//    if ( (ret_acc=ADXL345_getRawValue()) !=0) return ret_acc;
    for (iAcc=0; iAcc<3; ++iAcc) {
	acceleration[iAcc] = accl[iAcc] * ADXL345_UNIT;
    }
//    return 0;
}

// L3G4200D Setting
// See here : http://itseddy.me/embedded/2014/11/30/L3G4200D-i2c.html

void L3G4200D_init(int k) {
    if (k != 0 && k != 1) {
        puts("MUST BE 0 or 1 !");
        return;
    }

    bcm2835_i2c_setSlaveAddress(L3G4200D_ADDR);

    if (k==1) {
        regaddr[0] = L3G4200D_CTRL_REG1;
	switch(L3G4200D_RATE) {
	    case 100:
		regaddr[1] = 0x2F;
		break;
	    case 200:
		regaddr[1] = 0x4F;
		break;
	    case 400:
		regaddr[1] = 0x8F;
		break;
            case 800:
                regaddr[1] = 0xCF;
                break;
	    default:
		regaddr[1] = 0x8F;
	}
        bcm2835_i2c_write(regaddr,2);

	regaddr[0] = L3G4200D_CTRL_REG2;		// Filter related
	regaddr[1] = 0x00;
	bcm2835_i2c_write(regaddr,2);

	regaddr[0] = L3G4200D_CTRL_REG3;		// Interrupt related
        regaddr[1] = 0x00;
        bcm2835_i2c_write(regaddr,2);

        regaddr[0] = L3G4200D_CTRL_REG4;		// bit 7 : Block Data Update : 0 : continue, 1 : update when reading
	switch(L3G4200D_RANGE) {			// bit 4,5 : Full Scale selection (250/500/2000)
	    case 250:
		regaddr[1] = 0x80;
		break;
	    case 500:
		regaddr[1] = 0x90;
		break;
	    case 2000:
		regaddr[1] = 0xA0;
		break;
	    default :
		regaddr[1] = 0x80;
	}
        bcm2835_i2c_write(regaddr,2);

	regaddr[0] = L3G4200D_CTRL_REG5;                // FIFO related
        regaddr[1] = 0x00;
        bcm2835_i2c_write(regaddr,2);
    }

}

int L3G4200D_getRawValue(void) {
    char *buf = (char*) gyro;
    L3G4200D_init(0);
    regaddr[0] = L3G4200D_OUT_X_L;
    for (iGyr=0; iGyr<6; ++iGyr) {
        bcm2835_i2c_write(regaddr, 1);
        if ( (ret_gyr=bcm2835_i2c_read(&buf[iGyr], 1)) != BCM2835_I2C_REASON_OK) return ret_gyr;

	++regaddr[0];
    }
    return 0;
}

void L3G4200D_getRealData(float* angVel) {
//    if ( (ret_gyr=L3G4200D_getRawValue()) !=0) return ret_gyr;

    for (iGyr=0; iGyr<3; ++iGyr) {
	angVel[iGyr] = gyro[iGyr] * L3G4200D_UNIT * DEG_TO_RAD;
	if (L3G4200D_RANGE == 500) angVel[iGyr] *= 2;
	else if (L3G4200D_RANGE == 2000) angVel[iGyr] *= 8;
    }

}

void HMC5883L_singleMeasurement(void) {
    bcm2835_i2c_setSlaveAddress(HMC5883L_ADDR);
    regaddr[0] = HMC5883L_MODE_REG;
    regaddr[1] = 0x01;                              // Single measurement mode
    bcm2835_i2c_write(regaddr,2);
}

void HMC5883L_init(int k) {
    if (k != 0 && k != 1) {
        puts("MUST BE 0 or 1 !");
        return;
    }

    bcm2835_i2c_setSlaveAddress(HMC5883L_ADDR);

    if (k==1) {

        regaddr[0] = HMC5883L_MODE_REG;
        regaddr[1] = 0x00;                              // continue mode
//        regaddr[1] = 0x01;                              // Single measurement mode
        bcm2835_i2c_write(regaddr,2);

        regaddr[0] = HMC5883L_CONF_REG_A;		// No. of Sampling and data rate
        regaddr[1] = 0x60;
//	regaddr[1] = 0x00;				// No average
	switch (HMC5883L_RATE) {
	    case 0 :
		regaddr[1] += 0x00;
		break;
	    case 1 :
		regaddr[1] += 0x04;
		break;
	    case 3 :
		regaddr[1] += 0x08;
		break;
	    case 7 :
		regaddr[1] += 0x0C;
		break;
	    case 15 :
		regaddr[1] += 0x10;
		break;
	    case 30 :
		regaddr[1] += 0x14;
		break;
	    case 75 :
		regaddr[1] += 0x18;
		break;
	    default :
		regaddr[1] += 0x10;
	}
        bcm2835_i2c_write(regaddr,2);

        regaddr[0] = HMC5883L_CONF_REG_B;		// Range
        regaddr[1] = 0x20;				// +- 1.3 Ga
        bcm2835_i2c_write(regaddr,2);

    }

}

int HMC5883L_getRawValue(void) {
    HMC5883L_init(0);
    regaddr[0] = HMC5883L_DATA_X_MSB;
    bcm2835_i2c_write(regaddr, 1);
    if ( (ret_mag=bcm2835_i2c_read((char*) mag, 6)) != BCM2835_I2C_REASON_OK) return ret_mag;
    exchange((char*) mag, 6);
    tmpMag = mag[2];
    mag[2] = mag[1];
    mag[1] = tmpMag;
/*
#ifdef I2C_DEBUG
    printf("MEG: %d\t%d\t%d\n", mag[0], mag[1], mag[2]);
#endif
*/
    return 0;
}

int HMC5883L_getRealData(float* magn) {
    HMC5883L_singleMeasurement();
    //usleep(6000);
    if ( (ret_mag=HMC5883L_getRawValue()) !=0) return ret_mag;

    for (iMag=0; iMag<3; ++iMag) magn[iMag] = mag_gain[iMag] *(mag[iMag] - mag_offset[iMag]) * HMC5883L_RESOLUTION;
    return 0;

}

void HMC5883L_getRealData_Direct(float* magn) {
//    bcm2835_i2c_setSlaveAddress(HMC5883L_ADDR);
    //usleep(6000);
    //if ( (ret_mag=HMC5883L_getRawValue()) !=0) return ret_mag;

    for (iMag=0; iMag<3; ++iMag) magn[iMag] = mag_gain[iMag] *(mag[iMag] - mag_offset[iMag]) * HMC5883L_RESOLUTION;
//    return 0;
}

void HMC5883L_getOriginalData_Direct(float* magn) {
//    bcm2835_i2c_setSlaveAddress(HMC5883L_ADDR);
    //usleep(6000);
//    if ( (ret_mag=HMC5883L_getRawValue()) !=0) return ret_mag;

    for (iMag=0; iMag<3; ++iMag) magn[iMag] = (float) mag[iMag] * HMC5883L_RESOLUTION;
}

int HMC5883L_dataReady(void) {
    HMC5883L_init(0);
    regaddr[0] = HMC5883L_STAT_REG;
    bcm2835_i2c_write(regaddr, 1);
    if ( (ret_mag=bcm2835_i2c_read(regaddr, 1)) != BCM2835_I2C_REASON_OK) return ret_mag;
    if (regaddr[0]&1) return 1;

    return 0;
}


void BMP085_init(int k) {
    if (k != 0 && k != 1) {
        puts("MUST BE 0 or 1 !");
        return;
    }

    bcm2835_i2c_setSlaveAddress(BMP085_ADDR);
    char *buf = (char*)&Para_BMP085;

    if (k==1) {
        regaddr[0] = BMP085_AC1;
	regaddr[1] = 0;
        bcm2835_i2c_write(regaddr, 1);
	if (bcm2835_i2c_read(buf, 22) != BCM2835_I2C_REASON_OK) {
	    puts("Parameters of BMP085 are not correctly loaded");
            return;
	}

        exchange(buf, 22);

//        printf("ACN : %d\t%d\t%d\t", Para_BMP085.AC1, Para_BMP085.AC2, Para_BMP085.AC3);
//        printf("%d\t%d\t%d\n", Para_BMP085.AC4, Para_BMP085.AC5, Para_BMP085.AC6);
//        printf("B and M : %d\t%d\t%d\t%d\t%d\n", Para_BMP085.B1, Para_BMP085.B2, Para_BMP085.MB, Para_BMP085.MC, Para_BMP085.MD);

    }
}

void BMP085_Trigger_UTemp(void) {
    BMP085_init(0);
    regaddr[0] = 0xF4;
    regaddr[1] = 0x2E;
    bcm2835_i2c_write(regaddr,2);
#ifdef BMP085_WAIT
    usleep(5000);
#endif
}

void BMP085_Trigger_UPressure(void) {
    BMP085_init(0);
    regaddr[0] = 0xF4;
    regaddr[1] = 0x34+(OSRS<<6);
    bcm2835_i2c_write(regaddr,2);
#ifdef BMP085_WAIT
    usleep(25500);
#endif
}

int BMP085_getRawTemp(void) {
    BMP085_init(0);
    regaddr[0] = 0xF6;
    bcm2835_i2c_write(regaddr, 1);
    if ( (ret_bar=bcm2835_i2c_read(databuf, 2)) != BCM2835_I2C_REASON_OK) return ret_bar;
    UT = (long)(databuf[0]<<8) + databuf[1];
    return 0;
}

int BMP085_getRawPressure(void) {
    BMP085_init(0);
    regaddr[0] = 0xF6;
    bcm2835_i2c_write(regaddr, 1);
    if ( (ret_bar=bcm2835_i2c_read(databuf, 3)) != BCM2835_I2C_REASON_OK) return ret_bar;
    UP = ((long)(databuf[0]<<16) + (long)(databuf[1]<<8) + databuf[2]) >> (8-OSRS);
    return 0;
}

void BMP085_getRealData(float *RTD, long *RP, float *altitude) {
//    BMP085_Trigger_UTemp();
//    if (BMP085_getRawTemp() !=0 ) return ERROR_BMP085_IO;
//    BMP085_Trigger_UPressure();
//    if (BMP085_getRawPressure() != 0 ) return ERROR_BMP085_IO;
//    printf("%ld, %ld\n", UT, UP);

    long X1 = ((UT - Para_BMP085.AC6) * Para_BMP085.AC5) >>15;
    long X2 = (Para_BMP085.MC <<11) / (X1+Para_BMP085.MD);
    long B5 = X1+X2;
    *RTD = ((B5+8)>>4)/10.;

    long B6 = B5 - 4000;
    X1 = (Para_BMP085.B2*((B6*B6)>>12))>>11;
    X2 = (Para_BMP085.AC2 * B6) >>11;
    long X3 = X1 + X2;
    long B3 = ( (( (long)Para_BMP085.AC1 * 4 + X3 ) << OSRS) + 2 ) / 4;
    X1 = (Para_BMP085.AC3 * B6) >>13;
    X2 = (Para_BMP085.B1 * (B6*B6)>>12)>>16;
    X3 = ((X1+X2)+2)>>2;
    long B4 = (Para_BMP085.AC4 * ((unsigned long)X3 + 32768)) >>15;
    unsigned long B7 = ((unsigned long)UP - B3) * (50000 >> OSRS);

    if (B7 < 0x80000000) *RP = (B7 * 2) / B4 ;
    else *RP = (B7 / B4) * 2 ;

    X1 = (*RP>>8)*(*RP>>8);
    X1 = (X1 * 3038) >>16;
    X2 = (-7357 * (*RP)) >> 16;

    *RP = *RP + ((X1 + X2 + 3791)>>4);

    *altitude = 44330 * (1 - pow((*RP)*1.0/P0, 1/5.255) );

//    return 0;
}

void getAccGyro(float *accl, float *gyro) {
    ADXL345_getRealData(accl);
    L3G4200D_getRealData(gyro);
}
