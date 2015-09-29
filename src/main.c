#include <time.h>
#include <stdio.h>
#include <math.h>
//#include <string.h>
#include <unistd.h>
#include <bcm2835.h>
#include <pthread.h>
#include "Setup.h"
#include "I2CControl.h"
#include "Initialization.h"
#include "Calibration.h"
#include "SPIControl.h"
#include "Device.h"
#include "Quaternion.h"

#define	LINESIZE	256
#define RAD_TO_DEG      (180/M_PI)
#define NUM_THREADS 2

int iThread = 1;
/* A mutex protecting job_queue. */
static unsigned int thread_count;

char Pause(void) {
    char c;
    puts("Start measuring... Press Enter to continue");
    while ( (c=getchar()) != '\n' ){}
    return c;
}

void* Renew_acclgyro_cycle(void *data) {
    I2CVariables* var = (I2CVariables*) data;
    while (iThread) {
	Renew_acclgyro(var);
    }
    return 0;
}

void* Renew_magn_cycle(void *data) {
//    I2CVariables* var = (I2CVariables*) data;
    while (iThread) {
        Renew_magn((I2CVariables*) data);
    }
    __sync_fetch_and_sub(&thread_count,1);
    return 0;
}

void* Renew_baro_cycle(void *data) {
    I2CVariables* var = (I2CVariables*) data;
    while(iThread) {
	Renew_baro(var);
    }
    __sync_fetch_and_sub(&thread_count,1);
    return 0;
}

int main(void) {
    struct timespec tp1, tp2;
    unsigned long startTime, procesTime;
    int command=0, angle=0;
    float deltaT = 2.5e-3;
    double asum[3];
    thread_count = NUM_THREADS;
    Drone_Status stat;
//    I2CVariables i2c_var;
//    I2CVariblesCali i2c_cali;

    pthread_t t_magn, t_baro;

    int i=0,j,ret;
//    if ( (ret=init_all(&i2c_var)) !=0) return ret;
    if ( (ret=Drone_init(&stat)) != 0 ) return ret;
    for (i=0; i<3; ++i) asum[i] = 0.0;
    puts("Start calibration!");
    Drone_Calibration(&stat);

    Drone_Calibration_printResult(&stat);

//    RF24_init();
//    Data_init(&i2c_cali, &stat);
    Drone_Start(&stat);

    puts("Run thread!");
    pthread_create(&t_magn, NULL, &Renew_magn_cycle, (void*) &stat.i2c_var);
    pthread_create(&t_baro, NULL, &Renew_baro_cycle, (void*) &stat.i2c_var);
    usleep(50000);


    clock_gettime(CLOCK_REALTIME, &tp1);
    startTime = tp1.tv_sec*1000000000 + tp1.tv_nsec;

    for (i=0; i<10000; ++i) {
	if ( (ret=Renew_acclgyro(&stat.i2c_var))!=0 ) {
	    i--;
	    continue;
	}

	clock_gettime(CLOCK_REALTIME, &tp2);
    	procesTime = tp2.tv_sec*1000000000 + tp2.tv_nsec - startTime;
	deltaT = (float)procesTime/1000000000.0;
	Drone_Renew(&stat, &deltaT);
	for (j=0; j<3; ++j) {
	    asum[j] += stat.a[j]/10000;
	}

	if (i%100==0){
	    printf("%d: A = : %f, %f, %f, %f, \t", i, stat.a[0], stat.a[1], stat.a[2], stat.altitude_corr);
	    printf("Roll = %f, Pitch = %f, Yaw = %f, Yaw_real = %f, dt = %E \n", RAD_TO_DEG*stat.angle[0], RAD_TO_DEG*stat.angle[1], RAD_TO_DEG*stat.angle[2], RAD_TO_DEG*stat.yaw_real, deltaT);
//	    printf("Angle = %d, command = %d\n", angle, command);
	}

//	angle = 0;

//	for (j=0; j<3; ++j) {
//	    angle += ((int)roundf(RAD_TO_DEG*stat.angle[j])&0xFF)<<(j*8) ;
//	}
//	angle += ((int)roundf(stat.altitude_corr*100)&0xFF)<<24;

//	RF24_exchangeInfo(&command, &angle);
//	for (j=0; j<4; ++j) {
//	    stat.i2c_var.PWM_power[0]=0;
//	}
//	Renew_PWM(&stat.i2c_var);
	tp1 = tp2;
	startTime = tp1.tv_sec*1000000000 + tp1.tv_nsec;
    }
    puts("Done!");
    iThread = 0;

//    for (j=0; j<3; ++j) {
//        asum[j]/= 100000;
//    }
    printf("average a : %f, %f, %f\n", asum[0], asum[1], asum[2]);

    do {
        __sync_synchronize();
//	printf("count = %d\n", thread_count);
        usleep(1000000);
    } while (thread_count);

    Drone_end(&stat);

    return 0;
}

