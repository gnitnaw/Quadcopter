#define DEG_TO_RAD_RATIO    0.0174532f
#define RAD_TO_DEG_RATIO    57.2957795f

typedef struct
{
    // Measured noise
    double data_noise;
    // Measured data
    float data_measured;
    // prior error covariance
    double P_EC;
    // Time Update
    double data_updated;
    // Measurement Update
    double K_factor;
    double data_estimated;
    double P_EC_estimated;
} Kalman;
