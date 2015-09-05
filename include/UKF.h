#include <gsl/gsl_matrix.h>
//7-state q0 q1 q2 q3 wx wy wz

#define UKF_STATE_DIM 7
//13-measurement q0 q1 q2 q3 ax ay az wx wy wz mx my mz
#define UKF_MEASUREMENT_DIM 13
#define UKF_HALFPI 1.5707963267948966192313216916398f
#define UKF_PI 3.1415926535897932384626433832795f
#define UKF_TWOPI 6.283185307179586476925286766559f
#define UKF_TODEG(x) ((x) * 57.2957796f)

typedef struct
{
    //scaling factor
    float gamma;
    //weights for means
    float Wm0, Wmi;
    //weights for covariance
    float Wc[UKF_SP_POINTS * UKF_SP_POINTS];
    //state covariance
    float P[UKF_STATE_DIM * UKF_STATE_DIM];
    float PX[UKF_STATE_DIM * UKF_STATE_DIM];
    float PY[UKF_MEASUREMENT_DIM * UKF_MEASUREMENT_DIM];
    float tmpPY[UKF_MEASUREMENT_DIM * UKF_MEASUREMENT_DIM];
    float PXY[UKF_STATE_DIM * UKF_MEASUREMENT_DIM];
    float PXYT[UKF_MEASUREMENT_DIM * UKF_STATE_DIM];
    float Q[UKF_STATE_DIM * UKF_STATE_DIM];
    float R[UKF_MEASUREMENT_DIM * UKF_MEASUREMENT_DIM];
    //Sigma points
    float XSP[UKF_STATE_DIM * UKF_SP_POINTS];
    float tmpXSP[UKF_STATE_DIM * UKF_SP_POINTS];
    float tmpXSPT[UKF_SP_POINTS * UKF_STATE_DIM];
    float tmpWcXSP[UKF_STATE_DIM * UKF_SP_POINTS];
    float tmpWcYSP[UKF_MEASUREMENT_DIM * UKF_SP_POINTS];
    float YSP[UKF_MEASUREMENT_DIM * UKF_SP_POINTS];
    float tmpYSP[UKF_MEASUREMENT_DIM * UKF_SP_POINTS];
    float YSP[UKF_MEASUREMENT_DIM * UKF_SP_POINTS];
    float tmpYSP[UKF_MEASUREMENT_DIM * UKF_SP_POINTS];
    float tmpYSPT[UKF_SP_POINTS * UKF_MEASUREMENT_DIM];
    //Kalman gain
    float K[UKF_STATE_DIM * UKF_MEASUREMENT_DIM];
    float KT[UKF_MEASUREMENT_DIM * UKF_STATE_DIM];
    //state vector
    float X[UKF_STATE_DIM];
    float tmpX[UKF_STATE_DIM];
    //measurement vector
    float Y[UKF_MEASUREMENT_DIM];
    float tmpY[UKF_MEASUREMENT_DIM];
    //
    gsl_matrix* m_Wc;
    gsl_matrix* m_P;
    gsl_matrix* m_PX;
    gsl_matrix* m_PY;
    gsl_matrix* m_tmpPY;
    gsl_matrix* m_PXY;
    gsl_matrix* m_PXYT;
    gsl_matrix* m_Q;
    gsl_matrix* m_R;
    //
    gsl_matrix* m_XSP;
    gsl_matrix* m_tmpXSP;
    gsl_matrix* m_tmpXSPT;
    gsl_matrix* m_tmpWcXSP;
    gsl_matrix* m_tmpWcYSP;
    gsl_matrix* m_YSP;
    gsl_matrix* m_tmpYSP;
    gsl_matrix* m_tmpYSPT;
    //
    gsl_matrix* m_K;
    gsl_matrix* m_KT;
    //
    gsl_matrix* m_X;
    gsl_matrix* m_tmpX;
    gsl_matrix* m_Y;
    gsl_matrix* m_tmpY;

} UKF;

extern int ADXL345_getRealData(float *accl);
extern int L3G4200D_getRealData(float *gyro);
extern int HMC5883L_getRealData(float *magn);
