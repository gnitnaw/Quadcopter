/*typedef enum DeviceNum {
    GY80_ADXL345,
    GY80_L3G4200D,
    GY80_HMC5883L,
    GY80_BMP085,
}*/

typedef struct {
    float accl[3], gyro[3], magn[3], altitude;
    float accl_ref[3], gyro_corr[3], altitude_corr;
    float angle[3], float angVel[3];
    float x[3], v[3], a[3];
} Drone_Status;
