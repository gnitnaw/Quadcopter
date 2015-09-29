#include <stdio.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <bcm2835.h>
#include "Common.h"
#include "Device.h"
#include "Quaternion.h"
#include "Calibration.h"

#define ACC_UNIT        9.79
#define ALTITUDE_FILTER 0.995

static int ret, i;
static float factor;

extern unsigned short THROTTLE;
extern float expectAngle[3], expectAngvel[3];

extern float Kp, Ki, Kd;

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

    RF24_init();
    return 0;
}

void Drone_end(Drone_Status *stat) {
    I2CVariables_end(&stat->i2c_var);
    bcm2835_i2c_end();
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
    Quaternion_From_Stat(stat);
}

void Drone_Renew(Drone_Status *stat, float* deltaT) {
    ret = 0;
    // Check data quality
    while (pthread_mutex_trylock(&stat->i2c_var.mutex) != 0) bcm2835_delayMicroseconds(100);

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

    pthread_mutex_unlock (&stat->i2c_var.mutex);
}


void Drone_Control(Drone_Status *stat) {
}
float pidUpdate(PIDControl* pid, const float measured,float expect,float gyro)
{
  float output;
  static float lastoutput=0;

  pid->desired=expect;                                                         //获取期望角度

  pid->error = pid->desired - measured;                   //偏差：期望-测量值
  
  pid->integ += pid->error * IMU_UPDATE_DT;          //偏差积分

  if (pid->integ > pid->iLimit)                                  //作积分限制
  {
    pid->integ = pid->iLimit;
  }
  else if (pid->integ < -pid->iLimit)
  {
    pid->integ = -pid->iLimit;
  }                                 

// pid->deriv = (pid->error - pid->prevError) / IMU_UPDATE_DT;                //微分         应该可用陀螺仪角速度代替
  pid->deriv = -gyro;
  if(fabs(pid->error)>Piddeadband)                                                                        //pid死区
  {
                  pid->outP = pid->kp * pid->error;                                                                 //方便独立观察
                  pid->outI = pid->ki * pid->integ;
                  pid->outD = pid->kd * pid->deriv;
               
                  output = (pid->kp * pid->error) +
                           (pid->ki * pid->integ) +
                           (pid->kd * pid->deriv);
  }
  else
  {
                    output=lastoutput;
  }

  pid->prevError = pid->error;                                                                         //更新前一次偏差
  lastoutput=output;

  return output;
}
