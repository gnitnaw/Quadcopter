#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <bcm2835.h>
#include "main.h"
#include "Setup.h"
#include "PCA9685PW.h"

#define	LINESIZE	256

char Pause(void) {
    char c;
    puts("Start measuring... Press Enter to continue");
    while ( (c=getchar()) != '\n' ){}
    return c;
}

int main(void) {

    int i,j,ret;
    float accl[3], gyro[3];
    double accl_est[3], gyro_est[3], accl_offset[3], gyro_offset[3], mag_real[3], angle;
    char buf[LINESIZE];
//    FILE *fp = fopen("testdata.dat","w");
    if ( (ret=init_all()) !=0) return ret;
    usleep(1000);

    if ( (ret=Kalman_init_all_Calibration(accl, gyro, accl_offset, gyro_offset)) !=0 ) return ret;

//    Pause();

//    if ( (ret=Calibration_HMC5883L(mag, mag_offset, mag_gain)) != 0 ) return ret;

    for (i=0; i<3; ++i) {
	printf("%d th: offset = %f, gain = %f\n", i, mag_offset[i], mag_gain[i]);
    }


    for (i=0; i<10; ++i) {
	if ( (ret=HMC5883L_getAngle(&angle))!=0 ) return ret;
	printf("%f\n", angle);
	usleep(67000);
    }

//    fclose(fp);
    return 0;
}

