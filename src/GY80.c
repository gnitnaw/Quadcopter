/*
    Quadcopter -- GY80.c
    Description :
    The library to read data from GY80

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
#include "I2CControl.h"
#include "GY80_Address.h"
#include "GY80.h"
#include "Common.h"

#define	OSRS			3		// oversampling_setting : 3 means ultra high resolution
#define P0              	101325		// atmosphere pressure
#define ADXL345_UNIT    	0.004       	// Unit of ADXL345 is 4mg
#define L3G4200D_UNIT		0.00875		// Unit of L3G4200D when range = 250 dps
#define DEG_TO_RAD		(M_PI/180)	// Convert degree to rad
#define HMC5883L_RESOLUTION	0.92
//#define I2C_DEBUG				// debug mode

extern int ADXL345_RATE;			// Sample rate of ADXL345
extern int ADXL345_RANGE;			// Range of ADXL345	(2,4,8 indicate +-2g, +-4g, +-8g)
extern int L3G4200D_RATE;			// Sample rate of L3G4200D
extern int L3G4200D_RANGE;			// Range of L3G4200D
extern int HMC5883L_RATE;			// Sample rate of HMC5883L (continuous measurement)

// Users need to calibrate HMC5883L in order to minimize the effect from other source of magnetic field (laptop, etc.)

extern float mag_offset[3];			// Calibration parameter for HMC5883L : offset of each axis
extern float mag_gain[3];			// Calibration parameter for HMC5883L : Gain of each axis

// This is the struct of calibration parameters of BMP085_Parameters
// Each piece of sensor has different parameters.
// Need to read them at the beginning of measurement (or simply save them, then user can load them to estimate temperature and pressure)
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

// calibration parameters of BMP085_Parameters
// Since only BMP085 will use it, and I prefer that nobody can see them, so I hide it here.
static struct BMP085_Parameters Para_BMP085;

static char regaddr[2], databuf[3];
static short stat_accl[3], stat_gyro[3], stat_mag[3];
//static long stat_UP, stat_UT;
static short tmpMag;
static int ret_acc, ret_gyr, ret_mag, ret_bar;

// void exchange(char *buf, int len) :
// Exchange the char between buf[i] and buf[i+1] when i%2==0 and i < len
// Need to do it when the data (each element is usually 2 bytes) is MSB
// Parameter :
// char* buf : pointer of the data
// len : length of data
void exchange(char *buf, int len) {
    char tmp;
    int i;
    for (i=0; i<len; ++i) {
        tmp = buf[i];
        buf[i] = buf[i+1];
        buf[i+1] = tmp;
        ++i;
    }
}

// void ADXL345_switch() : Change the slave address of i2c to ADXL345
void ADXL345_switch(void) {
    bcm2835_i2c_setSlaveAddress(ADXL345_ADDR);
}

// void ADXL345_init() : Initialize ADXL345
void ADXL345_init(void) {
    ADXL345_switch();

    regaddr[0] = ADXL345_POWER_CTL;			// Standby
    regaddr[1] = 0x00;
    bcm2835_i2c_write(regaddr,2);

    regaddr[0] = ADXL345_DATA_FORMAT;			// Range
    switch (ADXL345_RANGE) {
	case 2 :
	    regaddr[1] = 0x08;
	    break;
	case 4 :
	    regaddr[1] = 0x09;
	    break;
	case 8 :
	    regaddr[1] = 0x0A;
	    break;
	default :
	    regaddr[1] = 0x0A;
    }
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

int ADXL345_getRawValue(short* acc) {
    ADXL345_switch();
    regaddr[0] = ADXL345_DATAX0;
    bcm2835_i2c_write(regaddr, 1);
    if ( (ret_acc = bcm2835_i2c_read((char*)acc, 6)) != BCM2835_I2C_REASON_OK) return ret_acc;
    return 0;
}

int ADXL345_getRealData(float* acceleration) {
    if ( (ret_acc=ADXL345_getRawValue(stat_accl)) !=0) return ret_acc;
    int i;
    for (i=0; i<3; ++i) {
	acceleration[i] = stat_accl[i] * ADXL345_UNIT;
    }
    return 0;
}

void ADXL345_convertRawToReal(short *acc, float* acceleration) {
    int i;
    for (i=0; i<3; ++i) {
        acceleration[i] = acc[i] * ADXL345_UNIT;
    }
}

// L3G4200D Setting
// See here : http://itseddy.me/embedded/2014/11/30/L3G4200D-i2c.html

void L3G4200D_switch(void) {
    bcm2835_i2c_setSlaveAddress(L3G4200D_ADDR);
}

void L3G4200D_init(void) {
    L3G4200D_switch();

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
    regaddr[1] = 0x04;
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
/*
int L3G4200D_getRawValue(short* gyr) {
    char *buf = (char*) gyr;
    L3G4200D_switch();
    regaddr[0] = L3G4200D_OUT_X_L;
    int i;
    for (i=0; i<6; ++i) {
        bcm2835_i2c_write(regaddr, 1);
        if ( (ret_gyr=bcm2835_i2c_read(&buf[i], 1)) != BCM2835_I2C_REASON_OK) return ret_gyr;

	++regaddr[0];
    }
    return 0;
}
*/
int L3G4200D_getRawValue(short* gyr) {
    L3G4200D_switch();
    regaddr[0] = L3G4200D_OUT_X_L_7B;
    bcm2835_i2c_write(regaddr, 1);
    if ( (ret_gyr = bcm2835_i2c_read((char*)gyr, 6)) != BCM2835_I2C_REASON_OK) return ret_gyr;
    return 0;
}

int L3G4200D_getRealData(float* angVel) {
    if ( (ret_gyr=L3G4200D_getRawValue(stat_gyro)) !=0) return ret_gyr;
    int i;
    for (i=0; i<3; ++i) {
	angVel[i] = stat_gyro[i] * L3G4200D_UNIT * DEG_TO_RAD;
	if (L3G4200D_RANGE == 500) angVel[i] *= 2;
	else if (L3G4200D_RANGE == 2000) angVel[i] *= 8;
    }
    return 0;
}

void L3G4200D_convertRawToReal(short *gyr, float* angVel) {
    int i;
    for (i=0; i<3; ++i) {
        angVel[i] = gyr[i] * L3G4200D_UNIT * DEG_TO_RAD;
        if (L3G4200D_RANGE == 500) angVel[i] *= 2;
        else if (L3G4200D_RANGE == 2000) angVel[i] *= 8;
    }
}

void HMC5883L_switch(void) {
    bcm2835_i2c_setSlaveAddress(HMC5883L_ADDR);
}


void HMC5883L_init(void) {
    HMC5883L_switch();

    regaddr[0] = HMC5883L_MODE_REG;
    regaddr[1] = 0x00;                              // continue mode , default
    bcm2835_i2c_write(regaddr,2);

    regaddr[0] = HMC5883L_CONF_REG_A;		// No. of Sampling and data rate
    regaddr[1] = 0x60;				// 8
    //regaddr[1] = 0x00;			// No average
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

void HMC5883L_singleMeasurement(void) {
    HMC5883L_switch();
    regaddr[0] = HMC5883L_MODE_REG;
    regaddr[1] = 0x01;                              // Single measurement mode
    bcm2835_i2c_write(regaddr,2);
}

int HMC5883L_getRawValue(short* magn) {
    HMC5883L_switch();
    regaddr[0] = HMC5883L_DATA_X_MSB;
    bcm2835_i2c_write(regaddr, 1);
    if ( (ret_mag=bcm2835_i2c_read((char*) magn, 6)) != BCM2835_I2C_REASON_OK) return ret_mag;
    exchange((char*) magn, 6);
    tmpMag = magn[2];
    magn[2] = magn[1];
    magn[1] = tmpMag;
/*
#ifdef I2C_DEBUG
    printf("MEG: %d\t%d\t%d\n", mag[0], mag[1], mag[2]);
#endif
*/
    return 0;
}

int HMC5883L_getRealData(float* magn) {
    int i;
    if ( (ret_mag=HMC5883L_getRawValue(stat_mag)) !=0) return ret_mag;

    for (i=0; i<3; ++i) magn[i] = mag_gain[i] *(stat_mag[i] - mag_offset[i]) * HMC5883L_RESOLUTION;
    return 0;

}

void HMC5883L_getRealData_Direct(float* magn) {
    int i;
    for (i=0; i<3; ++i) magn[i] = mag_gain[i] *(stat_mag[i] - mag_offset[i]) * HMC5883L_RESOLUTION;
}

void HMC5883L_convertRawToReal_Zero(short *mag, float* magn) {
    int i;
    for (i=0; i<3; ++i) magn[i] = (float) mag[i] * HMC5883L_RESOLUTION;
}

void HMC5883L_convertRawToReal(short *magn, float* magn_real) {
    int i;
    for (i=0; i<3; ++i) magn_real[i] = mag_gain[i] *(magn[i] - mag_offset[i]) * HMC5883L_RESOLUTION;
}


void BMP085_switch(void) {
    bcm2835_i2c_setSlaveAddress(BMP085_ADDR);
}

void BMP085_init(void) {
    BMP085_switch();
    char *buf = (char*)&Para_BMP085;

    regaddr[0] = BMP085_AC1;
    regaddr[1] = 0;
    bcm2835_i2c_write(regaddr, 1);
    if (bcm2835_i2c_read(buf, 22) != BCM2835_I2C_REASON_OK) {
	puts("Parameters of BMP085 are not correctly loaded");
        return;
    }

    exchange(buf, 22);
//#ifdef I2C_DEBUG
//        printf("ACN : %d\t%d\t%d\t", Para_BMP085.AC1, Para_BMP085.AC2, Para_BMP085.AC3);
//        printf("%d\t%d\t%d\n", Para_BMP085.AC4, Para_BMP085.AC5, Para_BMP085.AC6);
//        printf("B and M : %d\t%d\t%d\t%d\t%d\n", Para_BMP085.B1, Para_BMP085.B2, Para_BMP085.MB, Para_BMP085.MC, Para_BMP085.MD);
//#endif
}

void BMP085_Trigger_UTemp(void) {
    BMP085_switch();
    regaddr[0] = 0xF4;
    regaddr[1] = 0x2E;
    bcm2835_i2c_write(regaddr,2);
#ifdef BMP085_WAIT
    _usleep(5000);
#endif
}

void BMP085_Trigger_UPressure(void) {
    BMP085_switch();
    regaddr[0] = 0xF4;
    regaddr[1] = 0x34+(OSRS<<6);
    bcm2835_i2c_write(regaddr,2);
#ifdef BMP085_WAIT
    _usleep(25500);
#endif
}

int BMP085_getRawTemp(long* UT) {
    BMP085_switch();
    regaddr[0] = 0xF6;
    bcm2835_i2c_write(regaddr, 1);
    if ( (ret_bar=bcm2835_i2c_read(databuf, 2)) != BCM2835_I2C_REASON_OK) return ret_bar;
    *UT = (long)(databuf[0]<<8) + databuf[1];
    return 0;
}

int BMP085_getRawPressure(long* UP) {
    BMP085_switch();
    regaddr[0] = 0xF6;
    bcm2835_i2c_write(regaddr, 1);
    if ( (ret_bar=bcm2835_i2c_read(databuf, 3)) != BCM2835_I2C_REASON_OK) return ret_bar;
    *UP = ((long)(databuf[0]<<16) + (long)(databuf[1]<<8) + databuf[2]) >> (8-OSRS);
    return 0;
}

void BMP085_getRealData(long *UT, long *UP, float *RTD, long *RP, float *altitude) {
//    BMP085_Trigger_UTemp();
//    if (BMP085_getRawTemp() !=0 ) return ERROR_BMP085_IO;
//    BMP085_Trigger_UPressure();
//    if (BMP085_getRawPressure() != 0 ) return ERROR_BMP085_IO;
//    printf("%ld, %ld\n", UT, UP);

    long X1 = ((*UT - Para_BMP085.AC6) * Para_BMP085.AC5) >>15;
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
    unsigned long B7 = ((unsigned long)(*UP) - B3) * (50000 >> OSRS);

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
