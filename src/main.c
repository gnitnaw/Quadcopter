#include <time.h>
#include <stdio.h>
//#include <string.h>
//#include <unistd.h>
#include <bcm2835.h>
#include <pthread.h>
#include "Setup.h"
#include "Error.h"
#include "EKF.h"
#define	LINESIZE	256
#define	PIN		RPI_BPLUS_GPIO_J8_07

/* A mutex protecting job_queue. */
pthread_mutex_t mutex;

float accl[3], gyro[3], magn[3];
float RTD, altitude;
long RP;

char Pause(void) {
    char c;
    puts("Start measuring... Press Enter to continue");
    while ( (c=getchar()) != '\n' ){}
    return c;
}

void* Renew_acclgyro(void *argv) {
    while (1) {
    	if (pthread_mutex_trylock(&mutex) == 0) {
    	    if ( getAccGyro(accl, gyro)!=0 ) {
	    	printf("Error on Accl or Gyro\n");
    	    }
	    pthread_mutex_unlock (&mutex);
	    usleep(3000);
    	}
    }
}

void* Renew_magn(void *argv) {
    while (1) {
	if (pthread_mutex_trylock(&mutex) == 0) {
            if ( HMC5883L_getRealData_Direct(magn)!=0 ) {
            	printf("Error on Magn\n");
            }
	    HMC5883L_singleMeasurement();
            pthread_mutex_unlock (&mutex);
	    usleep(6500);
    	}
    }
}

void* Renew_baro(void *argv) {
    while(1) {
	while (pthread_mutex_trylock(&mutex) != 0) usleep(1000);
	BMP085_Trigger_UTemp();
	pthread_mutex_unlock (&mutex);
	usleep(5000);

        while (pthread_mutex_trylock(&mutex) != 0) usleep(1000);
        BMP085_getRawTemp();
	BMP085_Trigger_UPressure();
	pthread_mutex_unlock (&mutex);
	usleep(25500);

	while (pthread_mutex_trylock(&mutex) != 0) usleep(1000);
        BMP085_getRawPressure();
        pthread_mutex_unlock (&mutex);

	if (BMP085_getRealData(&RTD, &RP, &altitude) !=0) {
	    printf("Error on Baro\n");
	}
    }
}
int main(void) {
    struct timespec tp1, tp2;
    unsigned long startTime, procesTime;
//    float accl[3], gyro[3], magn[3];
    float accl_offset[3], gyro_offset[3], magn_offset[3], accl_est[3], gyro_est[3], magn_est[3], gyro_corr[3];
    float deltaT = 2.5e-3;
    float Eular[3];
//    float acceleration[3], velocity[3], deplace[3];
    pthread_t t_acclgyro, t_magn, t_baro;

    pthread_mutex_init (&mutex, NULL); 
    int i=0,j,ret;
    if ( (ret=init_all()) !=0) return ret;
//    HMC5883L_singleMeasurement();
//    EKF_Filter ekf;
//    if ( (ret=EKF_New(&ekf)) !=0 ) ret;
//    if ( (ret=Kalman_init_all_Calibration(accl, gyro, magn, accl_offset, gyro_offset, magn_offset)) !=0 ) return ret;
//    Kalman_estimateAll(accl, gyro, magn, accl_est, gyro_est, magn_est);
//    Quaternion_init(accl_offset, magn_offset);

    pthread_create(&t_acclgyro, NULL, &Renew_acclgyro, NULL);
    usleep(1000);
    pthread_create(&t_magn, NULL, &Renew_magn, NULL);
    usleep(10000);
    pthread_create(&t_baro, NULL, &Renew_baro, NULL);
    usleep(50000);

    while (i<100) {
	printf("Magn = : %f, %f, %f\t", magn[0], magn[1], magn[2]);
	printf("Accl = : %f, %f, %f\t", accl[0], accl[1], accl[2]);
	printf("Gyro = : %f, %f, %f\t", gyro[0], gyro[1], gyro[2]);
	printf("altitude = %f\n", altitude);
	usleep(60000);
	++i;
    }
/*
    for (i=0; i<3; ++i) {
	acceleration[i] = 0.0;
	velocity[i] = 0.0;
	deplace[i] = 0.0;
    }
*/
/*
    clock_gettime(CLOCK_REALTIME, &tp1);
    startTime = tp1.tv_sec*1000000000 + tp1.tv_nsec;
    HMC5883L_singleMeasurement();
    usleep(6600);
    if ( (ret=HMC5883L_getRealData(magn))!=0 ) return ret;

    for (i=0; i<1000000; ++i) {
	if (i%4==0) HMC5883L_singleMeasurement();
	if ( (ret=getAccGyro(accl, gyro))!=0 ) return ret;
	if (i%4==3) {
	    if ( (ret=HMC5883L_getRealData(magn))!=0 ) return ret;
	}
//	Kalman_estimateAll(accl, gyro, magn, accl_est, gyro_est, magn_est);
//        EFK_Update(&ekf, gyro, accl, magn, &deltaT);
//        EKF_GetAngle(&ekf, Eular);
//	printf("ACCL : %f, %f, %f\t", accl[0], accl[1], accl[2]);
//	printf("GYRO : %f, %f, %f\t", gyro[0], gyro[1], gyro[2]);
//	printf("MAGN : %f, %f, %f\n\n", magn[0], magn[1], magn[2]);
 
	for (j=0; j<3; ++j) gyro_corr[j] = gyro[j]-gyro_offset[j];
	clock_gettime(CLOCK_REALTIME, &tp2);
    	procesTime = tp2.tv_sec*1000000000 + tp2.tv_nsec - startTime;
	deltaT = (float)procesTime/1000000000.0;
	Quaternion_renew(accl, gyro_corr, magn, &deltaT, Eular);
	//for (j=0; j<3; ++j) {}
	if (i%500==0){
//	    printf("Magn = : %f, %f, %f\t", magn[0], magn[1], magn[2]);
	    printf("Roll = %f, Pitch = %f, Yaw = %f, dt = %E\n", Eular[0], Eular[1], Eular[2], deltaT);
	}
	tp1 = tp2;
	startTime = tp1.tv_sec*1000000000 + tp1.tv_nsec;
    }
*/
    return end_all();
}

