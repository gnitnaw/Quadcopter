#ifndef _EKF_H
#define _EKF_H
//#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
//15-state x y z vx vy vz ax ay az roll pitch yaw v_roll v_pitch v_yaw
#define EKF_STATE_DIM 15
//13-measurement q0 q1 q2 q3 ax ay az wx wy wz mx my mz
#define EKF_MEASUREMENT_DIM 13

#define EKF_HALFPI 1.5707963267948966192313216916398f
#define EKF_PI 3.1415926535897932384626433832795f
#define EKF_TWOPI 6.283185307179586476925286766559f
#define EKF_TODEG(x) ((x) * 57.2957796f)

typedef struct EKF_FILTER_T{
    //state vector
    float X[EKF_STATE_DIM] ;							// state matrix
    float A[EKF_STATE_DIM*EKF_STATE_DIM] ;					// state equation matrix
    float Q[EKF_STATE_DIM*EKF_STATE_DIM] ;					// Process Noise Covariance
    float H[EKF_MEASUREMENT_DIM*EKF_STATE_DIM] ;				// Measurement Equation - relates x to z 
    float P[EKF_STATE_DIM*EKF_STATE_DIM] ;					// Error Covariance Matrix 
    float R[EKF_MEASUREMENT_DIM*EKF_MEASUREMENT_DIM] ;				// Measurement Noise Covariance
    float G[EKF_STATE_DIM*EKF_MEASUREMENT_DIM] ;				// Kalman matrix

    float tmpX[EKF_STATE_DIM] ;							// Only for temp. use

	//state covariance
	float P[EKF_STATE_DIM * EKF_STATE_DIM];
	float Q[EKF_STATE_DIM * EKF_STATE_DIM];
	float R[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
	//Kalman gain
	float K[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
	//Measurement covariance
	float S[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
	//The H matrix maps the measurement to the states
	float F[EKF_STATE_DIM * EKF_STATE_DIM];
	float H[EKF_MEASUREMENT_DIM * EKF_STATE_DIM];
	float I[EKF_STATE_DIM * EKF_STATE_DIM];
	//state vector
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
//void EKF_Init(EKF_Filter* ekf, float *q, float *gyro);
//void EFK_Update(EKF_Filter* ekf, float *q, float *gyro, float *accel, float *mag, float dt);
//void EKF_GetAngle(EKF_Filter* ekf, float* rpy);

inline void EKF_GetQ(EKF_Filter* efk, float* Q) {
	Q[0] = efk->X[0];
	Q[1] = efk->X[1];
	Q[2] = efk->X[2];
	Q[3] = efk->X[3];
}

#endif


extern int ADXL345_getRealData(float *accl);
extern int L3G4200D_getRealData(float *gyro);
extern int HMC5883L_getRealData(float *magn);

