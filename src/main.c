#include <time.h>
#include <stdio.h>
//#include <string.h>
#include <unistd.h>
#include <bcm2835.h>
#include <pthread.h>
#include "Setup.h"
#include "I2CControl.h"
#include "Initialization.h"
#include "Calibration.h"
#include "Device.h"

#define	LINESIZE	256

/* A mutex protecting job_queue. */

char Pause(void) {
    char c;
    puts("Start measuring... Press Enter to continue");
    while ( (c=getchar()) != '\n' ){}
    return c;
}

void* Renew_acclgyro_cycle(void *data) {
    I2CVariables* var = (I2CVariables*) data;
    while (1) {
	Renew_acclgyro(var);
    }
}

void* Renew_magn_cycle(void *data) {
    I2CVariables* var = (I2CVariables*) data;
    while (1) {
        Renew_magn(var);
    }
}

void* Renew_baro_cycle(void *data) {
    I2CVariables* var = (I2CVariables*) data;
    while(1) {
	Renew_baro(var);
    }
}

int main(void) {
    struct timespec tp1, tp2;
    unsigned long startTime, procesTime;
    float deltaT = 2.5e-3;
    I2CVariables i2c_var;
    I2CVariblesCali i2c_cali;

    pthread_t t_magn, t_baro;

    int i=0,ret;
    if ( (ret=init_all(&i2c_var)) !=0) return ret;

    Calibration_getSD_multithread(&i2c_cali);
    printf("ACCL MEAN: %f, %f, %f; ", i2c_cali.accl_offset[0], i2c_cali.accl_offset[1], i2c_cali.accl_offset[2]);
    printf("ACCL SD: %f, %f, %f\n", i2c_cali.accl_sd[0], i2c_cali.accl_sd[1], i2c_cali.accl_sd[2]);
    printf("GYRO MEAN: %f, %f, %f; ", i2c_cali.gyro_offset[0], i2c_cali.gyro_offset[1], i2c_cali.gyro_offset[2]);
    printf("GYRO SD: %f, %f, %f\n", i2c_cali.gyro_sd[0], i2c_cali.gyro_sd[1], i2c_cali.gyro_sd[2]);
    printf("MAGN MEAN: %f, %f, %f; ", i2c_cali.magn_offset[0], i2c_cali.magn_offset[1], i2c_cali.magn_offset[2]);
    printf("MAGN SD: %f, %f, %f\n", i2c_cali.magn_sd[0], i2c_cali.magn_sd[1], i2c_cali.magn_sd[2]);
    printf("ALTITUDE MEAN: %f; ALTITUDE SD: %f\n", i2c_cali.altitude_offset, i2c_cali.altitude_sd);

    Drone_Status stat;
    Data_init(&i2c_cali, &stat);

    pthread_create(&t_magn, NULL, &Renew_magn_cycle, (void*) &i2c_var);
    pthread_create(&t_baro, NULL, &Renew_baro_cycle, (void*) &i2c_var);
    usleep(50000);


    clock_gettime(CLOCK_REALTIME, &tp1);
    startTime = tp1.tv_sec*1000000000 + tp1.tv_nsec;

    for (i=0; i<10000; ++i) {
	if ( (ret=Renew_acclgyro(&i2c_var))!=0 ) return ret;
	clock_gettime(CLOCK_REALTIME, &tp2);
    	procesTime = tp2.tv_sec*1000000000 + tp2.tv_nsec - startTime;
	deltaT = (float)procesTime/1000000000.0;
	Data_Copy(&i2c_var, &stat);
	Data_Renew(&stat, &deltaT);

//	Quaternion_renew(accl, gyro_corr, magn, &deltaT, Eular);
	//for (j=0; j<3; ++j) {}
	if (i%100==0){
	    printf("A = : %f, %f, %f, %f, 0x%X\t", stat.x[0], stat.x[1], stat.x[2], stat.altitude_corr, stat.status);
	    printf("Roll = %f, Pitch = %f, Yaw = %f, dt = %E\n", stat.angle[0], stat.angle[1], stat.angle[2], deltaT);
	}
	usleep(2000);
	tp1 = tp2;
	startTime = tp1.tv_sec*1000000000 + tp1.tv_nsec;
    }

    return end_all(&i2c_var);
}

