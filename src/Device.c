#include <time.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <bcm2835.h>
#include "Common.h"
#include "Device.h"
#include "Quaternion.h"
#include "Calibration.h"
#include "PID.h"

#define NUM_THREADS	4
#define ACC_UNIT        9.79
#define ALTITUDE_FILTER 0.995
#define RAD_TO_DEG      (180/M_PI)

static int iThread = 1;
/* A mutex protecting job_queue. */
static unsigned int thread_count;

static int ret, i;
static float factor;
static unsigned long iDetect = 0;
extern unsigned short THROTTLE;
extern float expectAngle[3], expectAngvel[3];

static pthread_t t_magn, t_baro, t_accgyr, t_adc;

static struct timespec tp1, tp2;
static unsigned long startTime, procesTime;
static float deltaT = 2.5e-3, dT_PWM = 0.0;
static int power = 1850;
static float angle_expect[] = {0, 0, 0};

void* Renew_accgyr_cycle(void *data) {
    Drone_Status *stat = (Drone_Status*) data;
    clock_gettime(CLOCK_REALTIME, &tp1);
    startTime = tp1.tv_sec*1000000000 + tp1.tv_nsec;

    while (iThread) {
	Renew_acclgyro(&stat->i2c_var);
	clock_gettime(CLOCK_REALTIME, &tp2);
        procesTime = tp2.tv_sec*1000000000 + tp2.tv_nsec - startTime;
        deltaT = (float)procesTime/1000000000.0;
	dT_PWM += deltaT;
	while (pthread_mutex_trylock(&stat->i2c_var.mutex) != 0) bcm2835_delayMicroseconds(100);
	Drone_Renew(stat, &deltaT);
        if (iDetect%200==0){
            printf("A = : %f, %f, %f, %f\t", stat->angVel[0], stat->angVel[1], stat->angVel[2], stat->altitude_corr);
            printf("Roll = %f, Pitch = %f, Yaw = %f, Yaw_real = %f, dt = %E ", RAD_TO_DEG*stat->angle[0], RAD_TO_DEG*stat->angle[1], RAD_TO_DEG*stat->angle[2],RAD_TO_DEG*stat->yaw_real, deltaT);
	    while (pthread_mutex_trylock(&stat->spi_var.mutex) != 0) delayMicroseconds(100);
	    printf("Voltage : %f\n", stat->spi_var.voltage);
	    pthread_mutex_unlock (&stat->spi_var.mutex);
	    printf("PWM = : %d, %d, %d, %d\n", stat->i2c_var.PWM_power[0], stat->i2c_var.PWM_power[1], stat->i2c_var.PWM_power[2], stat->i2c_var.PWM_power[3]);
        }
	if ( iDetect>1000 && (iDetect&1)) {
	    if (iDetect==1000) angle_expect[2] = stat->angle[2];
	    PID_update(&stat->i2c_var.pid, angle_expect, stat->angle, stat->angVel, stat->i2c_var.PWM_power, &dT_PWM, &power);
	    dT_PWM = 0.0;
	}
	pthread_mutex_unlock (&stat->i2c_var.mutex);
//	Renew_PWM(&stat->i2c_var);
	iDetect++;
	tp1 = tp2;
        startTime = tp1.tv_sec*1000000000 + tp1.tv_nsec;
    }
    __sync_fetch_and_sub(&thread_count,1);
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

void* Renew_RF24_cycle(void *data) {
    SPIVariables* var = (SPIVariables*) data;
    while(iThread) {
	RF24_Renew(var);
    }
    __sync_fetch_and_sub(&thread_count,1);
    return 0;
}

void* Renew_ADC_cycle(void *data) {
    SPIVariables* var = (SPIVariables*) data;
    while(iThread) {
        MCP3008_Renew(var);
    }
    __sync_fetch_and_sub(&thread_count,1);
    return 0;
}


int Drone_init(Drone_Status *stat) {
    memset(stat, 0, sizeof(Drone_Status));
    if (!bcm2835_init()) return -1;
    bcm2835_i2c_begin();
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);         // 400 kHz

    ADXL345_init(1);
    L3G4200D_init(1);
    HMC5883L_init(1);
    BMP085_init(1);
    PCA9685PW_init(1);

    I2CVariables_init(&stat->i2c_var);

//    PWM_init(&stat->i2c_var);

    RF24_init();
    return 0;
}

void Drone_end(Drone_Status *stat) {
    iThread = 0;
    do {
        __sync_synchronize();
        usleep(1000000);
    } while (thread_count);

    I2CVariables_end(&stat->i2c_var);
    SPIVariables_end(&stat->spi_var);
    bcm2835_i2c_end();
    puts("END!");
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
    stat->altitude_corr = 0.0;
    stat->accl_err = Common_GetNorm(stat->i2c_cali.accl_sd, 3);
    stat->angle[0] = atan2(stat->i2c_cali.accl_offset[1], stat->i2c_cali.accl_offset[2]); 		// roll
    stat->angle[1]  = -asin(stat->i2c_cali.accl_offset[0]/stat->i2c_cali.accl_abs);			// pitch
    stat->angle[2] = acos(stat->i2c_cali.magn_offset[0]/Common_GetNorm(stat->i2c_cali.magn_offset, 2));	// yaw
    //stat->angle[2] = atan2(stat->i2c_cali.accl_offset[2], sqrtf(stat->i2c_cali.accl_offset[0]*stat->i2c_cali.accl_offset[0] + stat->i2c_cali.accl_offset[2]*stat->i2c_cali.accl_offset[2]));
    angle_expect[2] = stat->angle[2];
    Quaternion_From_Stat(stat);

    thread_count = NUM_THREADS;
    puts("Drone -- Run thread!");
    pthread_create(&t_adc, NULL, &Renew_ADC_cycle, (void*) &stat->spi_var);
    pthread_create(&t_magn, NULL, &Renew_magn_cycle, (void*) &stat->i2c_var);
    pthread_create(&t_baro, NULL, &Renew_baro_cycle, (void*) &stat->i2c_var);
    usleep(50000);
    pthread_create(&t_accgyr, NULL, &Renew_accgyr_cycle, (void*) stat);
}

void Drone_Renew(Drone_Status *stat, float* deltaT) {
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
    if ( (factor = fabsf(stat->acc_magnitude - stat->i2c_cali.accl_abs ) ) >  stat->accl_err * 3 ) {
        ret |= (1<<7);                                                                                  // Means this accl value should not be used into filter
        if ( (factor=stat->acc_magnitude/stat->i2c_cali.accl_abs)>2.5 || factor < 0.5 ) ret |= (1<<6);                        // Data may be incorrect
    }

    for (i=0; i<3; ++i) stat->gyro_corr[i] = stat->i2c_var.gyro[i] - stat->i2c_cali.gyro_offset[i];
    if (stat->i2c_var.altitude>300) {
	stat->altitude_corr = stat->altitude_corr * ALTITUDE_FILTER + (stat->i2c_var.altitude-stat->i2c_cali.altitude_offset) * (1-ALTITUDE_FILTER);
    }

    stat->status = ret;

    Quaternion_renew_Drone(stat, deltaT);

//    pthread_mutex_unlock (&stat->i2c_var.mutex);
}

