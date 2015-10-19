#ifndef H_AHRS
#define H_AHRS

typedef struct {
    float q[4], qp[4];
    float half_dt;
    float norm;
    float anorm[3], mnorm[3];
    float v[3], h[3], b[3], w[3];
    float e[3];
    float eInt[3];
    float Kp, Ki;
    float angVel[3];
} AHRS;

void AHRS_init(AHRS* ahrs, float* angle, float* pi);
void AHRS_getAngle(AHRS* ahrs, float* angle);
void AHRS_calculate_MagField_Earth(AHRS* ahrs, float* magn);
void AHRS_renew(AHRS* ahrs, float* deltaT, float* accl, float* gyro, float* magn);

#endif
