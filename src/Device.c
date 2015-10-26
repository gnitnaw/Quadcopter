/*
    Quadcopter -- Device.c
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
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <bcm2835.h>
#include "Common.h"
#include "Device.h"
//#include "Quaternion.h"
#include "Calibration.h"
#include "PID.h"
#include "PCA9685PW.h"
#include "GY80.h"

#define NUM_THREADS	1
#define ACC_UNIT        9.79
#define ALTITUDE_FILTER 0.995
#define RAD_TO_DEG      (180/M_PI)
//#define NOPWM
#define LOGFILENAME	"Kp_7_105_0.2_all.dat"
//#define LOGFILENAME   "Silence4.dat"

static int iThread = 1;
/* A mutex protecting job_queue. */
//static unsigned int thread_count;
//static unsigned int thread_adc = 0, thread_mag = 0;
static int ret;
static float factor;
static unsigned long iDetect = 0;
extern unsigned short THROTTLE;
extern float pi[2];
//extern float expectAngle[3], expectAngvel[3];
extern int DEBUG_MODE;

static pthread_t t_magn, t_baro, t_accgyr, t_adc, t_pid;

static struct timespec tp1, tp2;
static unsigned long startTime, procesTime;
static float deltaT = 2.5e-3, dT_PWM = 0.0;
extern int power;
extern float angle_expect[3];
//static float angle_expect[] = {0, 0, 0};
static FILE *fp;
static float T = 0;

void* Renew_ADC(void *data) {
    pthread_detach(pthread_self());
    SPIVariables* var = (SPIVariables*) data;
    MCP3008_Renew(var);
    return 0;
}

void* Renew_MAG(void *data) {
    pthread_detach(pthread_self());
    measureAndTrigger_magn((I2CVariables*) data);
    return 0;
}

void* Renew_BAR1(void *data) {
//    stepBaro = 1;
//    I2CVariables* var = (I2CVariables*) data;
    pthread_detach(pthread_self());
    measureTemp_triggerPres();
//    stepBaro = 2;
    return 0;
}

void* Renew_BAR2(void *data) {
    pthread_detach(pthread_self());
    I2CVariables* var = (I2CVariables*) data;
    measurePres_triggerTemp(var);
    return 0;
}

void Renew_PID(void *data) {
    pthread_detach(pthread_self());
    Drone_Status *stat = (Drone_Status*) data;
    PID_update(&stat->i2c_var.pid, angle_expect, stat->angle, stat->gyro_corr, stat->i2c_var.PWM_power, &dT_PWM, &power);
    fprintf(fp, "%f\t%f\t", T, deltaT);
    fprintf(fp, "%f\t%f\t%f\t", stat->angle[0], stat->angle[1], stat->angle[2]);
    fprintf(fp, "%f\t%f\t%f\t", stat->accl_est[0], stat->accl_est[1],stat->accl_est[2]);
    fprintf(fp, "%f\t%f\t%f\t", stat->i2c_var.accl[0], stat->i2c_var.accl[1],stat->i2c_var.accl[2]);
    fprintf(fp, "%f\t%f\t%f\t", stat->gyro_corr[0]*RAD_TO_DEG,stat->gyro_corr[1]*RAD_TO_DEG,stat->gyro_corr[2]*RAD_TO_DEG);
    fprintf(fp, "%f\t%f\t%f\t", stat->ahrs.angVel[0], stat->ahrs.angVel[1], stat->ahrs.angVel[2]);
    fprintf(fp, "%f\t%f\t%f\t", stat->i2c_var.magn[0], stat->i2c_var.magn[1],stat->i2c_var.magn[2]);
    fprintf(fp, "%f\t%f\t%f\t", stat->magn_est[0], stat->magn_est[1],stat->magn_est[2]);
    fprintf(fp, "%d\t%d\t%d\t%d\n", stat->i2c_var.PWM_power[0], stat->i2c_var.PWM_power[1], stat->i2c_var.PWM_power[2], stat->i2c_var.PWM_power[3]);
    dT_PWM = 0.0;
#ifndef NOPWM
    Renew_PWM(&stat->i2c_var);
#endif
//    return 0;
}

void* Renew_accgyr_cycle(void *data) {
    Drone_Status *stat = (Drone_Status*) data;
    clock_gettime(CLOCK_REALTIME, &tp1);
    startTime = tp1.tv_sec*1000000000 + tp1.tv_nsec;

    while (iThread) {
	Renew_acclgyro(&stat->i2c_var);
	angle_expect[2] = stat->angle[2];
	clock_gettime(CLOCK_REALTIME, &tp2);
        procesTime = tp2.tv_sec*1000000000 + tp2.tv_nsec - startTime;
        deltaT = (float)procesTime/1000000000.0;
	if (deltaT < 0.003) continue;
	dT_PWM += deltaT;
	while (pthread_mutex_trylock(&stat->i2c_var.mutex) != 0) bcm2835_delayMicroseconds(100);
	//Filter_AcclGyro(stat->i2c_var.accl, stat->i2c_var.gyro);
	Drone_Renew(stat, &deltaT);
	AHRS_getAngle(&stat->ahrs, stat->angle);
        if (iDetect%100==0 && DEBUG_MODE){
            //printf("A = : %f, %f, %f, %f\t", stat->angVel[0], stat->angVel[1], stat->angVel[2], stat->altitude_corr);
            printf("Roll = %f, Pitch = %f, Yaw = %f, dt = %E, alti = %f,  ", stat->angle[0], stat->angle[1], stat->angle[2], deltaT, stat->i2c_var.altitude);
	    while (pthread_mutex_trylock(&stat->spi_var.mutex) != 0) delayMicroseconds(100);
	    printf("Voltage : %f\n", stat->spi_var.voltage);
	    pthread_mutex_unlock (&stat->spi_var.mutex);
	    printf("PWM = : %d, %d, %d, %d\n", stat->i2c_var.PWM_power[0], stat->i2c_var.PWM_power[1], stat->i2c_var.PWM_power[2], stat->i2c_var.PWM_power[3]);
        }
	pthread_mutex_unlock (&stat->i2c_var.mutex);
/*
	if ( iDetect%2==1 ) {
	    T += dT_PWM;
	    PID_update(&stat->i2c_var.pid, angle_expect, stat->angle, stat->gyro_corr, stat->i2c_var.PWM_power, &dT_PWM, &power);
	    fprintf(fp, "%f\t%f\t", T, deltaT);
	    fprintf(fp, "%f\t%f\t%f\t", stat->angle[0], stat->angle[1], stat->angle[2]);
	    fprintf(fp, "%f\t%f\t%f\t", stat->accl_est[0], stat->accl_est[1],stat->accl_est[2]);
	    fprintf(fp, "%f\t%f\t%f\t", stat->i2c_var.accl[0], stat->i2c_var.accl[1],stat->i2c_var.accl[2]);
	    fprintf(fp, "%f\t%f\t%f\t", stat->gyro_corr[0]*RAD_TO_DEG,stat->gyro_corr[1]*RAD_TO_DEG,stat->gyro_corr[2]*RAD_TO_DEG);
	    fprintf(fp, "%f\t%f\t%f\t", stat->ahrs.angVel[0], stat->ahrs.angVel[1], stat->ahrs.angVel[2]);
	    fprintf(fp, "%f\t%f\t%f\t", stat->i2c_var.magn[0], stat->i2c_var.magn[1],stat->i2c_var.magn[2]);
	    fprintf(fp, "%f\t%f\t%f\t", stat->magn_est[0], stat->magn_est[1],stat->magn_est[2]);
	    fprintf(fp, "%d\t%d\t%d\t%d\n", stat->i2c_var.PWM_power[0], stat->i2c_var.PWM_power[1], stat->i2c_var.PWM_power[2], stat->i2c_var.PWM_power[3]);
	    dT_PWM = 0.0;
#ifndef NOPWM
            Renew_PWM(&stat->i2c_var);
#endif

	}
*/
//	if (!iThread) puts("Interrupt");

	if ( (iDetect%2) == 1 ) {
            T += dT_PWM;
	    Renew_PID(stat);
//	    pthread_create(&t_pid, NULL, &Renew_PID, (void*) stat);
	} else if ( (iDetect%2) == 0 ) pthread_create(&t_magn, NULL, &Renew_MAG, (void*) &stat->i2c_var);
	if ( (iDetect%3000) == 1 ) pthread_create(&t_adc, NULL, &Renew_ADC, (void*) &stat->spi_var);
	if ( (iDetect%9) == 1 ) pthread_create(&t_baro, NULL, &Renew_BAR1, (void*)&stat->i2c_var);
	else if ( (iDetect%9) == 8 ) pthread_create(&t_baro, NULL, &Renew_BAR2, (void*)&stat->i2c_var);


//	if (iDetect == 4000) iThread = 0;

        iDetect++;
        tp1 = tp2;
        startTime = tp1.tv_sec*1000000000 + tp1.tv_nsec;
	bcm2835_delay(3);
	//delayMicroseconds(3000);
    }
#ifndef NOPWM
    PWM_reset(&stat->i2c_var);
#endif
//    __sync_fetch_and_sub(&thread_count,1);
    return 0;
}

int Drone_init(Drone_Status *stat) {
    memset(stat, 0, sizeof(Drone_Status));
    if (!bcm2835_init()) return -1;
    bcm2835_i2c_begin();
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);         // 400 kHz

    ADXL345_init();
    L3G4200D_init();
    HMC5883L_init();
    BMP085_init();
    PCA9685PW_init();

    I2CVariables_init(&stat->i2c_var);

    RF24_init();

    return 0;
}

void Drone_end(Drone_Status *stat) {
    iThread = 0;
//    do {
//        __sync_synchronize();
//        _usleep(100000);
//    } while (thread_count);
    pthread_join(t_accgyr, (void*)stat);
    fclose(fp);
    I2CVariables_end(&stat->i2c_var);
    SPIVariables_end(&stat->spi_var);
    bcm2835_i2c_end();
    if (DEBUG_MODE) puts("END!");
//    return 0;
}

void Drone_Calibration(Drone_Status *stat) {
    Calibration_getSD_multithread(&stat->i2c_cali);
}

void Drone_Calibration_printResult(Drone_Status *stat) {
    printf("ACCL MEAN: %f, %f, %f; ", stat->i2c_cali.accl_offset[0], stat->i2c_cali.accl_offset[1], stat->i2c_cali.accl_offset[2]);
    printf("ACCL SD: %f, %f, %f\n", stat->i2c_cali.accl_sd[0], stat->i2c_cali.accl_sd[1], stat->i2c_cali.accl_sd[2]);
    printf("GYRO MEAN: %f, %f, %f; ", stat->i2c_cali.gyro_offset[0], stat->i2c_cali.gyro_offset[1], stat->i2c_cali.gyro_offset[2]);
    printf("GYRO SD: %f, %f, %f\n", stat->i2c_cali.gyro_sd[0], stat->i2c_cali.gyro_sd[1], stat->i2c_cali.gyro_sd[2]);
    printf("MAGN MEAN: %f, %f, %f; ", stat->i2c_cali.magn_offset[0], stat->i2c_cali.magn_offset[1], stat->i2c_cali.magn_offset[2]);
    printf("MAGN SD: %f, %f, %f\n", stat->i2c_cali.magn_sd[0], stat->i2c_cali.magn_sd[1], stat->i2c_cali.magn_sd[2]);
    printf("ALTITUDE MEAN: %f; ALTITUDE SD: %f\n", stat->i2c_cali.altitude_offset, stat->i2c_cali.altitude_sd);
}

void Drone_Start(Drone_Status *stat) {
    int i;
    stat->altitude_corr = 0.0;
    stat->angle[0] = atan2(stat->i2c_cali.accl_offset[1], stat->i2c_cali.accl_offset[2]) * RAD_TO_DEG; 		// roll
    stat->angle[1]  = atan2(stat->i2c_cali.accl_offset[0], Common_GetNorm(stat->i2c_cali.accl_offset, 3)) * RAD_TO_DEG;
    stat->angle[2] = acos(stat->i2c_cali.magn_offset[1]/Common_GetNorm(stat->i2c_cali.magn_offset, 2)) * RAD_TO_DEG;	// yaw
    AHRS_init(&stat->ahrs, stat->angle, pi);
//    Quaternion_From_Stat(stat);

    float dt_temp = 0.01;
    for (i=0; i<3; ++i) {
	stat->i2c_var.accl[i] = stat->i2c_cali.accl_offset[i];
	stat->i2c_var.gyro[i] = stat->i2c_cali.gyro_offset[i];
	stat->i2c_var.magn[i] = stat->i2c_cali.magn_offset[i];
    }
    stat->acc_magnitude = Common_GetNorm(stat->i2c_var.accl, 3);
    stat->mag_magnitude = Common_GetNorm(stat->i2c_var.magn, 3);
    stat->yaw_real = acos(stat->i2c_var.magn[0]/Common_GetNorm(stat->i2c_var.magn, 2));
    stat->status = 0;


    for (i=0; i<5000; ++i) {
	AHRS_renew(&stat->ahrs, &dt_temp, stat->i2c_cali.accl_offset, stat->i2c_cali.gyro_offset, stat->i2c_cali.magn_offset);
    }

    Filter_init() ;
    Trigger_magn();
//    angle_expect[0] = 0.0;
//    angle_expect[1] = 0.0;
    angle_expect[2] = stat->angle[2];
    if (DEBUG_MODE) printf("Start Eular Angle : %f, %f, %f\n", stat->angle[0], stat->angle[1], stat->angle[2]);

#ifndef NOPWM
    PWM_init(&stat->i2c_var);
    _usleep(2000000);
#endif

//    thread_count = NUM_THREADS;
    if (DEBUG_MODE) puts("Drone -- Run thread!");

    fp = fopen(LOGFILENAME,"w");
    if (!fp) {
	puts("Error when opening file!");
	exit(1);
    }

    pthread_create(&t_accgyr, NULL, &Renew_accgyr_cycle, (void*) stat);
    _usleep(3000);
    //Renew_accgyr_cycle(stat);
}

void Drone_Renew(Drone_Status *stat, float* deltaT) {
    int i;
    ret = 0;
    // Check data quality
//    while (pthread_mutex_trylock(&stat->i2c_var.mutex) != 0) bcm2835_delayMicroseconds(100);

    if (stat->i2c_var.ret[0]) {
        if ((stat->i2c_var.ret[0] >> 8)!=0) {                            // if gyro has problem
            ret |= (1<<8);                                          // gyro OK. accl has problem
        } else {
            if ((stat->i2c_var.ret[0]&0xFF)!=0) {                        // gyro fail, check if accl OK
                ret |= (1<<9) ;                                  // Both fail
            }
	}
    }
    if (stat->i2c_var.ret[1]) {
        ret |= (1<<10);
    }
    if (stat->i2c_var.ret[2]) {
        ret |= (1<<11);
    }

    stat->acc_magnitude = Common_GetNorm(stat->i2c_var.accl, 3);
    stat->mag_magnitude = Common_GetNorm(stat->i2c_var.magn, 3);
    stat->yaw_real = acos(stat->i2c_var.magn[0]/Common_GetNorm(stat->i2c_var.magn, 2));
    if ( (factor=stat->acc_magnitude/stat->i2c_cali.accl_abs)>2.5 || factor < 0.5 ) ret |= (1<<6);                        // Data may be incorrect

    if ( (factor=stat->mag_magnitude/stat->i2c_cali.magn_abs)>1.5 || factor < 0.5 ) ret |= (1<<4);                        // Data may be incorrect

    for (i=0; i<3; ++i) stat->gyro_corr[i] = stat->i2c_var.gyro[i] - stat->i2c_cali.gyro_offset[i];
//    for (i=0; i<3; ++i) stat->gyro_corr[i] = stat->i2c_var.gyro[i];
    if (stat->i2c_var.altitude>300) {
	stat->altitude_corr = stat->altitude_corr * ALTITUDE_FILTER + (stat->i2c_var.altitude-stat->i2c_cali.altitude_offset) * (1-ALTITUDE_FILTER);
    }

    stat->status = ret;
    Drone_NoiseFilter(stat) ;

    AHRS_renew(&stat->ahrs, deltaT, stat->accl_est, stat->gyro_corr, stat->magn_est);

}


void Drone_NoiseFilter(Drone_Status *stat) {
    int i;
    for (i=0; i<3; ++i) {
	Filter_renew(&stat->filter_acc[i], &stat->i2c_var.accl[i], &stat->accl_est[i], &iDetect) ;
	Filter_renew(&stat->filter_gyr[i], &stat->i2c_var.gyro[i], &stat->gyro_est[i], &iDetect) ;
	Filter_renew(&stat->filter_mag[i], &stat->i2c_var.magn[i], &stat->magn_est[i], &iDetect) ;
    }
}

/*
void Drone_NoiseFilter(Drone_Status *stat) {
    int i;
    for (i=0; i<3; ++i) {
        Kalman_renew(&stat->filter_acc[i], &stat->i2c_var.accl[i], &stat->accl_est[i]) ;
        Kalman_renew(&stat->filter_gyr[i], &stat->i2c_var.gyro[i], &stat->gyro_est[i]) ;
        Kalman_renew(&stat->filter_mag[i], &stat->i2c_var.magn[i], &stat->magn_est[i]) ;
    }
}
*/

