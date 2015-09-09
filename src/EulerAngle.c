#include <stdio.h>
#include <math.h>
#include "I2CControl.h"
#include "Device.h"
#define DEG_TO_RAD      (M_PI/180)
#define RAD_TO_DEG	(180/M_PI)
static float q[4];
static float half_dt;
static float norm;
static float anorm[3], mnorm[3];
static float v[3], h[3], b[3], w[3];
static float e[3];
static float eInt[3];
static float gyro_adj[3];
static float EulerAngle[3];

static int i,j;

extern float Kp, Ki, Kd;

float findNorm(float* vec) {
    return sqrtf(pow(vec[0],2)+pow(vec[1],2)+pow(vec[2],2));
}

float getAngle(float *val) {
    return acos(val[1]/sqrtf(pow(val[0],2) + pow(val[1],2)));
}

void Rotate_Vector(float A[][3], float* x, float *b) {
    for (i=0; i<3; ++i) {
	b[i] = 0.0;
	for (j=0; j<3; ++j) b[i] += A[i][j] * x[j];
    }
}

void Quaternion_Euler(void) {
    EulerAngle[0] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1); // roll
    EulerAngle[1] = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2]); // pitch
    EulerAngle[2] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3]*q[3] + 1); // yaw
}

void Quaternion_From_EulerAngle(void) {
    float dCos[3], dSin[3];
    for (i=0; i<3; ++i) {
	dCos[i] = cos(EulerAngle[i] * 0.5);
	dSin[i] = sin(EulerAngle[i] * 0.5);
    }
    q[0] = dCos[0] * dCos[1] * dCos[2] + dSin[0] * dSin[1] * dSin[2];
    q[1] = dSin[0] * dCos[1] * dCos[2] - dCos[0] * dSin[1] * dSin[2];
    q[2] = dCos[0] * dSin[1] * dCos[2] + dSin[0] * dCos[1] * dSin[2];
    q[3] = dCos[0] * dCos[1] * dSin[2] - dSin[0] * dSin[1] * dCos[2];
}

void Quaternion_init(float* accl, float* magn) {
    float norm = sqrtf(accl[0]*accl[0]+accl[1]*accl[1]+accl[2]*accl[2]);
    EulerAngle[0] = atan2(accl[1], accl[2]) ; // roll
    EulerAngle[1]  = -asin(accl[0]/norm);
    EulerAngle[2] = getAngle(magn);

    Quaternion_From_EulerAngle();
    for (i=0; i<3; ++i) eInt[i] = 0.0f;
//    for (i=0; i<4; ++i) printf("%f\t", q[i]);
    puts("");
    printf("Init : Roll = %f, Pitch = %f, Yaw = %f\n", EulerAngle[0]*RAD_TO_DEG, EulerAngle[1]*RAD_TO_DEG, EulerAngle[2]*RAD_TO_DEG);
}

void Quaternion_renew(Drone_Status *stat, float* deltaT) {
}

void Quaternion_renew(float* accl_est, float* gyro_est, float* magn_est, float* deltaT, float* Euler) {
//    for (i=0; i<4; ++i) printf("%f\t", q[i]);
    half_dt = *deltaT/2;
    norm = sqrtf(accl_est[0]*accl_est[0]+accl_est[1]*accl_est[1]+accl_est[2]*accl_est[2]);
    for (i=0; i<3; ++i) {
	anorm[i] = accl_est[i]/norm;
    }

    norm = sqrtf(magn_est[0]*magn_est[0]+magn_est[1]*magn_est[1]+magn_est[2]*magn_est[2]);;
    for (i=0; i<3; ++i) {
        mnorm[i] = magn_est[i]/norm;
    }

    // compute reference direction of flux
    h[0] = 2*mnorm[0]*(0.5 - q[2]*q[2] - q[3]*q[3]) + 2*mnorm[1]*(q[1]*q[2] - q[0]*q[3]) + 2*mnorm[2]*(q[1]*q[3] + q[0]*q[2]);
    h[1] = 2*mnorm[0]*(q[1]*q[2] + q[0]*q[3]) + 2*mnorm[1]*(0.5 - q[1]*q[1] - q[3]*q[3]) + 2*mnorm[2]*(q[2]*q[3] - q[0]*q[1]);
    h[2] = 2*mnorm[0]*(q[1]*q[3] - q[0]*q[2]) + 2*mnorm[1]*(q[2]*q[3] + q[0]*q[1]) + 2*mnorm[2]*(0.5 - q[1]*q[1] - q[2]*q[2]);
    b[0] = sqrtf((h[0]*h[0]) + (h[1]*h[1]));
    b[2] = h[2];


    // estimated direction of gravity and flux (v and w)
    v[0] = 2*(q[1]*q[3] - q[0]*q[2]);
    v[1] = 2*(q[0]*q[1] + q[2]*q[3]);
    v[2] = pow(q[0],2) - pow(q[1],2) - pow(q[2],2) + pow(q[3],2);
//    printf("V: %f\t%f\t%f\n", v[0], v[1], v[2]);
//    printf("A: %f\t%f\t%f\n", anorm[0], anorm[1], anorm[2]);

    w[0] = 2*b[0]*(0.5 - q[2]*q[2] - q[3]*q[3]) + 2*b[2]*(q[1]*q[3] - q[0]*q[2]);
    w[1] = 2*b[0]*(q[1]*q[2] - q[0]*q[3]) + 2*b[2]*(q[0]*q[1] + q[2]*q[3]);
    w[2] = 2*b[0]*(q[0]*q[2] + q[1]*q[3]) + 2*b[2]*(0.5 - q[1]*q[1] - q[2]*q[2]);

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    e[0] = (anorm[1] * v[2] - anorm[2] * v[1]) + (mnorm[1]*w[2] - mnorm[2]*w[1]);
    e[1] = (anorm[2] * v[0] - anorm[0] * v[2]) + (mnorm[2]*w[0] - mnorm[0]*w[2]);
    e[2] = (anorm[0] * v[1] - anorm[1] * v[0]) + (mnorm[0]*w[1] - mnorm[1]*w[0]);
//    e[0] = (mnorm[1]*w[2] - mnorm[2]*w[1]);
//    e[1] = (mnorm[2]*w[0] - mnorm[0]*w[2]);
//    e[2] = (mnorm[0]*w[1] - mnorm[1]*w[0]);

//    e[0] = (anorm[1] * v[2] - anorm[2] * v[1]) ;
//    e[1] = (anorm[2] * v[0] - anorm[0] * v[2]) ;
//    e[2] = (anorm[0] * v[1] - anorm[1] * v[0]) ;

//    printf("e: %f\t%f\t%f\n", e[0], e[1], e[2]);
    for (i=0; i<3; ++i) {
        eInt[i] += e[i] * Ki;
	gyro_adj[i] = gyro_est[i] + Kp * e[i] + eInt[i];
//	gyro_adj[i] = gyro_est[i] + Kp * e[i] / half_dt;
    }
//    printf("gyro_est: %f\t%f\t%f\n", gyro_est[0], gyro_est[1], gyro_est[2]);
//    printf("gyro_adj: %f\t%f\t%f\n", gyro_adj[0], gyro_adj[1], gyro_adj[2]);

    q[0] = q[0] - half_dt*(q[1]*gyro_adj[0] + q[2]*gyro_adj[1] + q[3]*gyro_adj[2]);
    q[1] = q[1] + half_dt*(q[0]*gyro_adj[0] + q[2]*gyro_adj[2] - q[3]*gyro_adj[1]);
    q[2] = q[2] + half_dt*(q[0]*gyro_adj[1] - q[1]*gyro_adj[2] + q[3]*gyro_adj[0]);
    q[3] = q[3] + half_dt*(q[0]*gyro_adj[2] + q[1]*gyro_adj[1] - q[2]*gyro_adj[0]);
//    for (i=0; i<4; ++i) printf("%f\t", q[i]);

    norm = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);

    for (i=0; i<4; ++i) q[i] /= norm;
    Quaternion_Euler();
    for (i=0; i<3; ++i) Euler[i] = EulerAngle[i]*RAD_TO_DEG;
}
