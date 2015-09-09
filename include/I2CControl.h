#include <pthread.h>
// Slave address
#define ADXL345_ADDR            0x53            // 3 Axis Accelerometer         Analog Devices ADXL345 
#define L3G4200D_ADDR           0x69            // 3 Axis Gyro                  ST Microelectronics L3G4200D
#define HMC5883L_ADDR           0x1E            // 3 Axis Magnetometer          Honeywell HMC5883L
#define BMP085_ADDR             0x77            // Barometer + Thermometer      Bosch BMP085
#define PCA9685PW_ADDR          0x40		// PWM				PCA9685PW

typedef struct {
    pthread_mutex_t mutex;
    float accl[3], gyro[3], magn[3];
    float RTD, altitude;
    long RP;
    int ret[3];
    int PWM_pin[4];
    int PWM_power[4];
} I2CVariables;

typedef struct {
    float accl_offset[3], gyro_offset[3], magn_offset[3], altitude_offset;
    float accl_sd[3], gyro_sd[3], magn_sd[3], altitude_sd;
    float accl_abs, magn_abs;
} I2CVariblesCali;

typedef struct {
    I2CVariables *i2c_var;
    float *mean;
    float *sd;
    char c;
} I2CCaliThread;

