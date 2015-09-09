#include <time.h>
#include <stdio.h>
//#include <string.h>
//#include <unistd.h>
#include <bcm2835.h>
#include <pthread.h>
#include "Setup.h"
#include "I2CControl.h"
//#include "Error.h"
//#include "EKF.h"
#define	LINESIZE	256

//extern int Renew_acclgyro(I2CVariables *i2c_var);
//extern int Renew_magn(I2CVariables *i2c_var);
//extern int Renew_baro(I2CVariables *i2c_var);
extern void* Calibration_getSD_singlethread(void *cal) ;
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
    float accl_est[3], gyro_est[3], magn_est[3], gyro_corr[3], mag_offset[3], mag_gain[3];
    float deltaT = 2.5e-3;
    float Eular[3];
    I2CVariables i2c_var;
    I2CVariblesCali i2c_cali;
    I2CVariables_init(&i2c_var);
//    pthread_mutex_init (&i2c_var.mutex, NULL);
    float acceleration[3], velocity[3], deplace[3];
    pthread_t t_acclgyro, t_magn, t_baro;

    int i=0,j,ret;
    if ( (ret=init_all()) !=0) return ret;
//    EKF_Filter ekf;
//    if ( (ret=EKF_New(&ekf)) !=0 ) ret;
//    if ( (ret=Kalman_init_all_Calibration(accl, gyro, magn, accl_offset, gyro_offset, magn_offset)) !=0 ) return ret;
//    Kalman_estimateAll(accl, gyro, magn, accl_est, gyro_est, magn_est);
//    Quaternion_init(accl_offset, magn_offset);
//    puts("Start !");
//    sleep(5);
//    Calibration_HMC5883L_singleThread(mag_offset, mag_gain);
//    printf("MAGN Offset: %f, %f, %f; ", mag_offset[0], mag_offset[1], mag_offset[2]);
//    printf("MAGN Gain: %f, %f, %f\n", mag_gain[0], mag_gain[1], mag_gain[2]);

    Calibration_getSD_multithread(&i2c_cali);
    printf("ACCL MEAN: %f, %f, %f; ", i2c_cali.accl_offset[0], i2c_cali.accl_offset[1], i2c_cali.accl_offset[2]);
    printf("ACCL SD: %f, %f, %f\n", i2c_cali.accl_sd[0], i2c_cali.accl_sd[1], i2c_cali.accl_sd[2]);
    printf("GYRO MEAN: %f, %f, %f; ", i2c_cali.gyro_offset[0], i2c_cali.gyro_offset[1], i2c_cali.gyro_offset[2]);
    printf("GYRO SD: %f, %f, %f\n", i2c_cali.gyro_sd[0], i2c_cali.gyro_sd[1], i2c_cali.gyro_sd[2]);
    printf("MAGN MEAN: %f, %f, %f; ", i2c_cali.magn_offset[0], i2c_cali.magn_offset[1], i2c_cali.magn_offset[2]);
    printf("MAGN SD: %f, %f, %f\n", i2c_cali.magn_sd[0], i2c_cali.magn_sd[1], i2c_cali.magn_sd[2]);
    printf("ALTITUDE MEAN: %f; ALTITUDE SD: %f\n", i2c_cali.altitude_offset, i2c_cali.altitude_sd);

    pthread_create(&t_acclgyro, NULL, &Renew_acclgyro_cycle, (void*) &i2c_var);
//    usleep(1000);
    pthread_create(&t_magn, NULL, &Renew_magn_cycle, (void*) &i2c_var);
//    usleep(10000);
    pthread_create(&t_baro, NULL, &Renew_baro_cycle, (void*) &i2c_var);
    usleep(50000);

    while (i<100) {
	printf("Magn = : %f, %f, %f\t", i2c_var.magn[0], i2c_var.magn[1], i2c_var.magn[2]);
	printf("Accl = : %f, %f, %f\t", i2c_var.accl[0], i2c_var.accl[1], i2c_var.accl[2]);
	printf("Gyro = : %f, %f, %f\t", i2c_var.gyro[0], i2c_var.gyro[1], i2c_var.gyro[2]);
	printf("altitude = %f\n", i2c_var.altitude);
	usleep(6000);
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
    I2CVariables_end(&i2c_var);
    return end_all();
}

