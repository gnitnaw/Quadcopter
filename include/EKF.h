#ifndef _EKF_H
#define _EKF_H
#include "I2CControl.h"
//#include <gsl/gsl_matrix.h>
#include <fgsl/gsl_blas.h>
//7-state q0 q1 q2 q3 wx wy wz
#define EKF_STATE_DIM 7
//6 measurement ax ay az mx my mz
#define EKF_MEASUREMENT_DIM 6

#define EKF_HALFPI 1.5707963267948966192313216916398f
#define EKF_PI 3.1415926535897932384626433832795f
#define EKF_TWOPI 6.283185307179586476925286766559f
#define EKF_TODEG(x) ((x) * 57.2957796f)

typedef struct EKF_FILTER_T{
	// Estimate covariance : initial : 100000 * I(7), update : P = FPF' + Q
	float P[EKF_STATE_DIM * EKF_STATE_DIM];
	// Covariance matrices : 0th~3rd should be 0 (q has no measurement error), 4th-6th is the standard deviation of gyro
	// But it will only be done when desired value of X is reached
	// Initial value: 0,0,0,0,0.2,0.2,0.2
	float Q[EKF_STATE_DIM * EKF_STATE_DIM];
        // Covariance matrices : 0-2 s.d. of accl, 3th-5th s.t. of magn
        // But it will only be done when desired value of X is reached
        // Initial value: 1e6,1e6,1e6,1e7,1e7,1e7
	float R[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
	// Optimal Kalman gain
	float K[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
	// Residual covariance
	float S[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
	//The H matrix maps the measurement to the states
 	// F : Jacobian
	float F[EKF_STATE_DIM * EKF_STATE_DIM];
	float H[EKF_MEASUREMENT_DIM * EKF_STATE_DIM];
	float I[EKF_STATE_DIM * EKF_STATE_DIM];
	//state vector : q0, q1, q2, q3, wx, wy, wz
	float X[EKF_STATE_DIM];
	//measurement vector
	float Y[EKF_MEASUREMENT_DIM];
	//
	float tmpP[EKF_STATE_DIM * EKF_STATE_DIM];
	float tmpS[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
	float tmpX[EKF_STATE_DIM];
	float tmpXX[EKF_STATE_DIM * EKF_STATE_DIM];
	float tmpXY[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
	float tmpYX[EKF_MEASUREMENT_DIM * EKF_STATE_DIM];

	gsl_matrix_float_view m_P;
	gsl_matrix_float_view m_Q;
	gsl_matrix_float_view m_R;
	gsl_matrix_float_view m_K;
	gsl_matrix_float_view m_S;
	gsl_matrix_float_view m_F;
	gsl_matrix_float_view m_H;
	gsl_matrix_float_view m_I;
	//
	gsl_matrix_float_view m_X;
	gsl_matrix_float_view m_Y;
	//
	gsl_matrix_float_view m_tmpP;
	gsl_matrix_float_view m_tmpX;
	gsl_matrix_float_view m_tmpYX;
	gsl_matrix_float_view m_tmpXY;
	gsl_matrix_float_view m_tmpXX;
	gsl_matrix_float_view m_tmpS;

}EKF_Filter;

//void EKF_New(EKF_Filter* ekf);
void EKF_Init(EKF_Filter *ekf, I2CVariblesCali *i2cCali);
//void EFK_Update(EKF_Filter* ekf, float *q, float *gyro, float *accel, float *mag, float dt);
//void EKF_GetAngle(EKF_Filter* ekf, float* rpy);
void EFK_Update(EKF_Filter* ekf, I2CVariables* i2c_var, float* dt);
void EKF_Quaternion_Euler(EKF_Filter* ekf, float* rpy);
#endif


//extern int ADXL345_getRealData(float *accl);
//extern int L3G4200D_getRealData(float *gyro);
//extern int HMC5883L_getRealData(float *magn);

