#include <stdio.h>
#include <math.h>
#include <bcm2835.h>
#include "GY80.h"
#define	OSRS		3
#define P0              101325

extern int ADXL345_RATE;
extern int L3G4200D_RATE;
extern int L3G4200D_RANGE;
extern int HMC5883L_RATE;

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

void ADXL345_init(int i) {
    if (i != 0 && i != 1) {
	puts("MUST BE 0 or 1 !");
	return;
    }
    bcm2835_i2c_setSlaveAddress(ADXL345);

    if (i==1) {
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

int ADXL345_getRawValue(short* accl) {
    char *buf = (char*) accl;
    init_ADXL345(0);
    regaddr[0] = ADXL345_DATAX0;
    bcm2835_i2c_write(regaddr, 1);
    if (bcm2835_i2c_read(buf, 6) != BCM2835_I2C_REASON_OK) return -1;
    return 0;
}

// L3G4200D Setting
// See here : http://itseddy.me/embedded/2014/11/30/L3G4200D-i2c.html

void L3G4200D_init(int i) {
    if (i != 0 && i != 1) {
        puts("MUST BE 0 or 1 !");
        return;
    }

    bcm2835_i2c_setSlaveAddress(L3G4200D);

    if (i==1) {
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
	    default:
		regaddr[1] = 0x8F;
	}

	if (L3G4200D_RATE == 200) regaddr[1] = 0x4F;

	regaddr[0] = L3G4200D_CTRL_REG2;		// Filter related
	regaddr[1] = 0x00;
	bcm2835_i2c_write(regaddr,2);

	regaddr[0] = L3G4200D_CTRL_REG3;		// Interrupt related
        regaddr[1] = 0x00;
        bcm2835_i2c_write(regaddr,2);

        regaddr[0] = L3G4200D_CTRL_REG4;		// bit 7 : Block Data Update : 0 : continue, 1 : update when reading
	switch(L3G4200D_RANGE) {			// bit 4,5 : Full Scale selection (250/500/2000)
	    case 250:
		regaddr[1] = 0x00;
		break;
	    case 500:
		regaddr[1] = 0x10;
		break;
	    case 2000:
		regaddr[1] = 0x20;
		break;
	    default :
		regaddr[1] = 0x00;
	}
        bcm2835_i2c_write(regaddr,2);

	regaddr[0] = L3G4200D_CTRL_REG5;                // FIFO related
        regaddr[1] = 0x00;
        bcm2835_i2c_write(regaddr,2);

    }

}

int L3G4200D_getRawValue(short* gyco) {
    int i ;
    char *buf = (char*) gyco;
    init_L3G4200D(0);

    for (i=0; i<6; ++i) {
        regaddr[0] = L3G4200D_OUT_X_L + i;
        bcm2835_i2c_write(regaddr, 1);
        if (bcm2835_i2c_read(&buf[i], 1) != BCM2835_I2C_REASON_OK) return -1;
    }

    return 0;

}

void HMC5883L_init(int i) {
    if (i != 0 && i != 1) {
        puts("MUST BE 0 or 1 !");
        return;
    }

    bcm2835_i2c_setSlaveAddress(HMC5883L);

    if (i==1) {
        regaddr[0] = HMC5883L_CONF_REG_A;		// No. of Sampling and data rate
        regaddr[1] = 0x60;
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

int HMC5883L_getRawValue(short* mag) {
    char *buf = (char*) mag;
    init_HMC5883L(0);
    regaddr[0] = HMC5883L_DATA_X_MSB;
    bcm2835_i2c_write(regaddr, 1);
    if (bcm2835_i2c_read(buf, 6) != BCM2835_I2C_REASON_OK) return -1;
    exchange(buf, 6);
    return 0;
}

void BMP085_init(int i) {
    if (i != 0 && i != 1) {
        puts("MUST BE 0 or 1 !");
        return;
    }

    char *buf = (char*)&Para_BMP085;
    if (i==1) {
        regaddr[0] = BMP085_AC1;
        bcm2835_i2c_write(regaddr, 1);
	if (bcm2835_i2c_read(buf, sizeof(Para_BMP085)) != BCM2835_I2C_REASON_OK) {
	    puts("Parameters of BMP085 are not correctly loaded");
            return;
	}

        exchange(buf, 22);

        printf("ACN : %d\t%d\t%d\t", Para_BMP085.AC1, Para_BMP085.AC2, Para_BMP085.AC3);
        printf("%d\t%d\t%d\n", Para_BMP085.AC4, Para_BMP085.AC5, Para_BMP085.AC6);
        printf("B and M : %d\t%d\t%d\t%d\t%d\n", Para_BMP085.B1, Para_BMP085.B2, Para_BMP085.MB, Para_BMP085.MC, Para_BMP085.MD);
    }
}

int BMP085_Trigger_UTemp(void) {
    init_BMP085(0);
    regaddr[0] = 0xF4;
    regaddr[1] = 0x2E;
    bcm2835_i2c_write(regaddr,2);
    usleep(5000);
    return 0;
}

int BMP085_Trigger_UPressure(void) {
    init_BMP085(0);
    regaddr[0] = 0xF4;
    regaddr[1] = 0x34+(OSRS<<6);
    bcm2835_i2c_write(regaddr,2);
    usleep(25500);
    return 0;
}

int BMP085_getRawTemp(long *UT) {
    regaddr[0] = 0xF6;
    bcm2835_i2c_write(regaddr, 1);
    if (bcm2835_i2c_read(databuf, 2) != BCM2835_I2C_REASON_OK) return -1;
    *UT = (long)(databuf[0]<<8) + databuf[1];
    if (*UT == 0) return -1;
    else return 0;
}

int BMP085_getRawPressure(long *UP) {
    regaddr[0] = 0xF6;
    bcm2835_i2c_write(regaddr, 1);
    if (bcm2835_i2c_read(databuf, 3) != BCM2835_I2C_REASON_OK) return -1;
    *UP = ((long)(databuf[0]<<16) + (long)(databuf[1]<<8) + databuf[2]) >> (8-OSRS);
    return 0;
}

int BMP085_getRealData(long *UT, long *UP, long *RT, long *RP, double *altitude) {
    long X1 = ((*UT - Para_BMP085.AC6) * Para_BMP085.AC5) >>15;
    long X2 = (Para_BMP085.MC <<11) / (X1+Para_BMP085.MD);
    long B5 = X1+X2;
    *RT = (B5+8)>>4;

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

    return 0;
}

