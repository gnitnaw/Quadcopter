#ifndef H_FILTER_KALMAN
#define H_FILTER_KALMAN

#define DEG_TO_RAD_RATIO    0.0174532f
#define RAD_TO_DEG_RATIO    57.2957795f

typedef struct
{
    // Measured noise
    float data_noise;
    // Measured data
    float data_measured;
    // prior error covariance
    float P_EC;
    // Time Update
    float data_updated;
    // Measurement Update
    float K_factor;
    float data_estimated;
    float P_EC_estimated;
} Filter_Kalman;

void Kalman_init(Filter_Kalman* k, float var, float noise);
void Kalman_renew(Filter_Kalman* k, float* var_m, float* var_e );
#endif
