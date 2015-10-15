#include <stdio.h>
#include <string.h>
#include <math.h>
#include <fgsl/gsl_linalg.h>
#include "Common.h"
#include "EKF.h"
#include "lu.h"
#define USE_4TH_RUNGE_KUTTA
//////////////////////////////////////////////////////////////////////////
//all parameters below need to be tune
//#define EKF_PQ_INITIAL 0.000001f
//#define EKF_PW_INITIAL 0.000010f
#define P_INITIAL	1.0e6f

#define EKF_QQ_INITIAL 0.000045f
#define EKF_QW_INITIAL 0.00025f

//#define EKF_RQ_INITIAL 0.000001f
//#define EKF_RA_INITIAL 0.07f
//#define EKF_RW_INITIAL 0.525f
//#define EKF_RM_INITIAL 0.105f
//#define EKF_RA_INITIAL 0.07f

//////////////////////////////////////////////////////////////////////////

static int i;
//static float accl[3], gyro[3], magn[3];
static float hx, hy, hz;
static float bx, bz;
static float _2mx, _2my, _2mz;
static float accl_norm, magn_norm;
float Vector_Normalize(float* vec, int n) {
    float norm = 0.0f;
    for (i=0; i<n; ++i) {
	norm += pow(vec[i],2);
    }
    return sqrt(norm);
}

void EKF_Quaternion_From_Stat(EKF_Filter* ekf, float *angle) {
    float dCos[3], dSin[3];
    for (i=0; i<3; ++i) {
        dCos[i] = cos(angle[i] * 0.5);
        dSin[i] = sin(angle[i] * 0.5);
    }
    ekf->X[0] = dCos[0] * dCos[1] * dCos[2] + dSin[0] * dSin[1] * dSin[2];
    ekf->X[1] = dSin[0] * dCos[1] * dCos[2] - dCos[0] * dSin[1] * dSin[2];
    ekf->X[2] = dCos[0] * dSin[1] * dCos[2] + dSin[0] * dCos[1] * dSin[2];
    ekf->X[3] = dCos[0] * dCos[1] * dSin[2] - dSin[0] * dSin[1] * dCos[2];
}

void EKF_Init(EKF_Filter* ekf, I2CVariblesCali *i2cCali) {
//    int i;
    memset(ekf, sizeof(EKF_Filter), 0);
//    float var_mean[3], var_sd[3];
//    int (*f)(float*) = 0;
    ekf->m_P = gsl_matrix_float_view_array (ekf->P, EKF_STATE_DIM, EKF_STATE_DIM);
    ekf->m_Q = gsl_matrix_float_view_array (ekf->Q, EKF_STATE_DIM, EKF_STATE_DIM);
    ekf->m_R = gsl_matrix_float_view_array (ekf->R, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM);
    ekf->m_K = gsl_matrix_float_view_array (ekf->K, EKF_STATE_DIM, EKF_MEASUREMENT_DIM);
    ekf->m_S = gsl_matrix_float_view_array (ekf->S, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM);
    ekf->m_F = gsl_matrix_float_view_array (ekf->F, EKF_STATE_DIM, EKF_STATE_DIM);
    ekf->m_H = gsl_matrix_float_view_array (ekf->H, EKF_MEASUREMENT_DIM, EKF_STATE_DIM);
    ekf->m_I = gsl_matrix_float_view_array (ekf->I, EKF_STATE_DIM, EKF_STATE_DIM);

    ekf->m_X = gsl_matrix_float_view_array(ekf->X, EKF_STATE_DIM, 1);
    ekf->m_Y = gsl_matrix_float_view_array(ekf->Y, EKF_MEASUREMENT_DIM, 1);
    ekf->m_tmpS = gsl_matrix_float_view_array (ekf->tmpS, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM);
    ekf->m_tmpP = gsl_matrix_float_view_array (ekf->tmpP, EKF_STATE_DIM, EKF_STATE_DIM);
    ekf->m_tmpX = gsl_matrix_float_view_array (ekf->tmpX, EKF_STATE_DIM, 1);
    ekf->m_tmpXX = gsl_matrix_float_view_array (ekf->tmpXX, EKF_STATE_DIM, EKF_STATE_DIM);
    ekf->m_tmpXY = gsl_matrix_float_view_array (ekf->tmpXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM);
    ekf->m_tmpYX = gsl_matrix_float_view_array (ekf->tmpYX, EKF_MEASUREMENT_DIM, EKF_STATE_DIM);

    //////////////////////////////////////////////////////////////////////////

    // Set X : q0,q1,q2,q3,wx,wy,wz(using gyro_offset)
    float angle[3];
    angle[0] = atan2(i2cCali->accl_offset[1], i2cCali->accl_offset[2]);               // roll
    angle[1]  = atan2(i2cCali->accl_offset[0], Common_GetNorm(i2cCali->accl_offset, 3));
    angle[2] = acos(i2cCali->magn_offset[1]/Common_GetNorm(i2cCali->magn_offset, 2)); // yaw

    EKF_Quaternion_From_Stat(ekf, angle);

    ekf->X[4] = i2cCali->gyro_offset[0];
    ekf->X[5] = i2cCali->gyro_offset[1];
    ekf->X[6] = i2cCali->gyro_offset[2];

    // Set Q : for q0-q3 : 0, wx-wz : gyro_sd
    ekf->Q[0] = ekf->Q[8] = ekf->Q[16] = ekf->Q[24] = 0;
    ekf->Q[32] = i2cCali->gyro_sd[0];
    ekf->Q[40] = i2cCali->gyro_sd[1];
    ekf->Q[48] = i2cCali->gyro_sd[2];	// process noise of gyro
//    ekf->Q[32] = 0.2;
//    ekf->Q[40] = 0.2;
//    ekf->Q[48] = 0.2; // process noise of gyro

    // Set P : I * 100000
    ekf->P[0] = ekf->P[8] = ekf->P[16] = ekf->P[24] = ekf->P[32] = ekf->P[40] = ekf->P[48] = P_INITIAL;

    // Set R : ax-az : accl_sd, mx-mz : magn_sd

    ekf->R[0] = i2cCali->accl_sd[0];
    ekf->R[7] = i2cCali->accl_sd[1];
    ekf->R[14] = i2cCali->accl_sd[2];
    ekf->R[21] = i2cCali->magn_sd[0];
    ekf->R[28] = i2cCali->magn_sd[1];
    ekf->R[35] = i2cCali->magn_sd[2];

//    ekf->R[0] = ekf->R[7] = ekf->R[14] = ekf->R[21] = ekf->R[28] = ekf->R[35] = P_INITIAL;

    // Set F : I (other element will be renew)
    gsl_matrix_float_set_identity(&ekf->m_F.matrix);

    // m_I : I
    gsl_matrix_float_set_identity(&ekf->m_I.matrix);
}


void EFK_Update(EKF_Filter* ekf, I2CVariables* i2c_var, float* dt) {
    float halfdt = 0.5f * *dt;
    float q0q0 = ekf->X[0]*ekf->X[0], q0q1 = ekf->X[0]*ekf->X[1], q0q2 = ekf->X[0]*ekf->X[2], q0q3 = ekf->X[0]*ekf->X[3];
    float q1q1 = ekf->X[1]*ekf->X[1], q1q2 = ekf->X[1]*ekf->X[2], q1q3 = ekf->X[1]*ekf->X[3];
    float q2q2 = ekf->X[2]*ekf->X[2], q2q3 = ekf->X[2]*ekf->X[3];
    float q3q3 = ekf->X[3]*ekf->X[3];

    // Step 1 : Renew F

    ekf->F[1] = -halfdt * (i2c_var->gyro[0] - ekf->X[4]); 		// -dt/2 * (gyro[0]-wx)
    ekf->F[2] = -halfdt * (i2c_var->gyro[1] - ekf->X[5]);		// -dt/2 * (gyro[1]-wy)
    ekf->F[3] = -halfdt * (i2c_var->gyro[2] - ekf->X[6]);		// -dt/2 * (gyro[2]-wz)
    ekf->F[4] =  halfdt * ekf->X[1];					// dt/2 * q1
    ekf->F[5] =  halfdt * ekf->X[2];					// dt/2 * q2
    ekf->F[6] =  halfdt * ekf->X[3];					// dt/2 * q3

    ekf->F[7]  = -ekf->F[1];						// dt/2 * (gyro[0]-wx)
    ekf->F[9]  =  ekf->F[3];						// -dt/2 * (gyro[2]-wz)
    ekf->F[10] = -ekf->F[2];						// dt/2 * (gyro[1]-wy)
    ekf->F[11] = -halfdt * ekf->X[0];					// -dt/2 * q0
    ekf->F[12] = -ekf->F[6];						// -dt/2 * q3
    ekf->F[13] =  ekf->F[5];						// dt/2 * q2

    ekf->F[14] = -ekf->F[2];						// dt/2 * (gyro[1]-wy)
    ekf->F[15] = -ekf->F[3];						// dt/2 * (gyro[2]-wz)
    ekf->F[17] =  ekf->F[1];						// -dt/2 * (gyro[0]-wx)
    ekf->F[18] =  ekf->F[6];						// dt/2 * q3
    ekf->F[19] =  ekf->F[11];						// -dt/2 * q0
    ekf->F[20] = -ekf->F[4];						// -dt/2 * q1

    ekf->F[21] = -ekf->F[3];						// dt/2 * (gyro[2]-wz)
    ekf->F[22] =  ekf->F[2];						// -dt/2 * (gyro[1]-wy)
    ekf->F[23] = -ekf->F[1];						// dt/2 * (gyro[0]-wx)
    ekf->F[25] = -ekf->F[5];						// -dt/2 * q2
    ekf->F[26] =  ekf->F[4];						// dt/2 * q1
    ekf->F[27] =  ekf->F[11];						// -dt/2 * q0

/*
    ekf->F[1] = -halfdt * (i2c_var->gyro[0] - ekf->X[4]);               // -dt/2 * (gyro[0]-wx)
    ekf->F[2] = -halfdt * (i2c_var->gyro[1] - ekf->X[5]);               // -dt/2 * (gyro[1]-wy)
    ekf->F[3] = -halfdt * (i2c_var->gyro[2] - ekf->X[6]);               // -dt/2 * (gyro[2]-wz)
    ekf->F[4] = -halfdt * ekf->X[1];                                    // -dt/2 * q1
    ekf->F[5] = -halfdt * ekf->X[2];                                    // -dt/2 * q2
    ekf->F[6] = -halfdt * ekf->X[3];                                    // -dt/2 * q3

    ekf->F[7]  = -ekf->F[1];                                            // dt/2 * (gyro[0]-wx)
    ekf->F[9]  =  ekf->F[3];                                            // -dt/2 * (gyro[2]-wz)
    ekf->F[10] = -ekf->F[2];                                            // dt/2 * (gyro[1]-wy)
    ekf->F[11] = halfdt * ekf->X[0];                                    // dt/2 * q0
    ekf->F[12] = -ekf->F[6];                                            // dt/2 * q3
    ekf->F[13] =  ekf->F[5];                                            // -dt/2 * q2

    ekf->F[14] = -ekf->F[2];                                            // dt/2 * (gyro[1]-wy)
    ekf->F[15] = -ekf->F[3];                                            // dt/2 * (gyro[2]-wz)
    ekf->F[17] =  ekf->F[1];                                            // -dt/2 * (gyro[0]-wx)
    ekf->F[18] =  ekf->F[6];                                            // -dt/2 * q3
    ekf->F[19] =  ekf->F[11];                                           // dt/2 * q0
    ekf->F[20] = -ekf->F[4];                                            // dt/2 * q1

    ekf->F[21] = -ekf->F[3];                                            // dt/2 * (gyro[2]-wz)
    ekf->F[22] =  ekf->F[2];                                            // -dt/2 * (gyro[1]-wy)
    ekf->F[23] = -ekf->F[1];                                            // dt/2 * (gyro[0]-wx)
    ekf->F[25] = -ekf->F[5];                                            // dt/2 * q2
    ekf->F[26] =  ekf->F[4];                                            // -dt/2 * q1
    ekf->F[27] =  ekf->F[11];                                           // +dt/2 * q0
*/
    // Step 2 : P = FPF' + Q
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &ekf->m_F.matrix, &ekf->m_P.matrix, 0.0, &ekf->m_tmpP.matrix); // tmpP = F * P
    gsl_blas_sgemm (CblasNoTrans, CblasTrans,1.0, &ekf->m_tmpP.matrix, &ekf->m_F.matrix, 0.0, &ekf->m_P.matrix); // P = tmp* F'
    gsl_matrix_float_add (&ekf->m_P.matrix, &ekf->m_Q.matrix);  // P += Q

    // Step 3 : Set Y
    // 3.1 Set accl items
    accl_norm = Common_GetNorm(i2c_var->accl, 3);

    ekf->Y[0] = i2c_var->accl[0]/accl_norm + 2 * (q1q3 - q0q2);
    ekf->Y[1] = i2c_var->accl[1]/accl_norm + 2 * (q2q3 + q0q1);
    ekf->Y[2] = i2c_var->accl[2]/accl_norm - (-q0q0+q1q1+q2q2-q3q3);

    // 3.2 Set magn items
    magn_norm = Common_GetNorm(i2c_var->magn, 3);
    _2mx = 2.0f * i2c_var->magn[0] / magn_norm;
    _2my = 2.0f * i2c_var->magn[1] / magn_norm;
    _2mz = 2.0f * i2c_var->magn[2] / magn_norm;

    //Reference direction of Earth's magnetic field
    hx = _2mx * (0.5f - q2q2 - q3q3) + _2my * (q1q2 - q0q3) + _2mz * (q1q3 + q0q2);
    hy = _2mx * (q1q2 + q0q3) + _2my * (0.5f - q1q1 - q3q3) + _2mz * (q2q3 - q0q1);
    hz = _2mx * (q1q3 - q0q2) + _2my * (q2q3 + q0q1) + _2mz * (0.5f - q1q1 - q2q2);

    bx = sqrtf(hx * hx + hy * hy);
    bz = hz;

    ekf->Y[3] = _2mx/2 - (bx * (1.0f - 2.0f * (q2q2 + q3q3)) + bz * ( 2.0f * (q1q3 - q0q2)));
    ekf->Y[4] = _2my/2 - (bx * (2.0f * (q1q2 - q0q3)) + bz * (2.0f * (q2q3 + q0q1)));
    ekf->Y[5] = _2mz/2 - (bx * (2.0f * (q1q3 + q0q2)) + bz * (1.0f - 2.0f * (q1q1 + q2q2)));

    // Step 4 : Renew H
    /////////////////////////////////////////////////////////////////////////
    //The H matrix maps the measurement to the states 6 x 7
    //row started from 0 to 6, col started from 0 to 6
    ekf->H[0] =  2 * ekf->X[2];					// 2q2
    ekf->H[1] = -2 * ekf->X[3];					// -2q3
    ekf->H[2] =  2 * ekf->X[0];					// 2q0
    ekf->H[3] = -2 * ekf->X[1];					// -2q1

    ekf->H[7]  =  ekf->H[3];					// -2q1
    ekf->H[8]  = -ekf->H[2];					// -2q0
    ekf->H[9]  =  ekf->H[1];					// -2q3
    ekf->H[10] = -ekf->H[0];					// -2q2

    ekf->H[14]  = -ekf->H[2];					// -2q0
    ekf->H[15]  = -ekf->H[3];					// 2q1
    ekf->H[16]  =  ekf->H[0];					// 2q2
    ekf->H[17]  =  ekf->H[1];					// -2q3

    ekf->H[21]  = 2* (ekf->X[0]*bx - ekf->X[2]*bz);		// 2(q0*bx-q2*bz)
    ekf->H[22]  = 2* (ekf->X[1]*bx + ekf->X[3]*bz);		// 2(q1*bx-q3*bz)
    ekf->H[23]  = 2* (-ekf->X[2]*bx - ekf->X[0]*bz);		// 2(-q2*bx-q0*bz)
    ekf->H[24]  = 2* (-ekf->X[3]*bx + ekf->X[1]*bz);		// 2(-q3bx-q1bz)

    ekf->H[28]  = 2* (-ekf->X[3]*bx + ekf->X[1]*bz);
    ekf->H[29]  = 2* (ekf->X[2]*bx + ekf->X[0]*bz);
    ekf->H[30]  = 2* (ekf->X[1]*bx + ekf->X[3]*bz);
    ekf->H[31]  = 2* (-ekf->X[0]*bx + ekf->X[2]*bz);

    ekf->H[35]  = 2* (ekf->X[2]*bx + ekf->X[0]*bz);
    ekf->H[36]  = 2* (ekf->X[3]*bx - ekf->X[1]*bz);
    ekf->H[37]  = 2* (ekf->X[0]*bx - ekf->X[2]*bz);
    ekf->H[38]  = 2* (ekf->X[1]*bx + ekf->X[3]*bz);
//    for (i=0; i<EKF_MEASUREMENT_DIM*EKF_MEASUREMENT_DIM; ++i) printf("%f, ", ekf->S[i]);

    // Step 5 : Renew S = H*P*H' + R
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &ekf->m_H.matrix, &ekf->m_P.matrix, 0.0, &ekf->m_tmpYX.matrix);  // tmpYX = H*P
    gsl_blas_sgemm (CblasNoTrans, CblasTrans,1.0, &ekf->m_tmpYX.matrix, &ekf->m_H.matrix, 0.0, &ekf->m_S.matrix); // S = tmpXY * H'
    gsl_matrix_float_add (&ekf->m_S.matrix, &ekf->m_R.matrix); // S += R

//    for (i=0; i<EKF_MEASUREMENT_DIM*EKF_MEASUREMENT_DIM; ++i) printf("%f, ", ekf->S[i]);
//    puts("");

    // Step 6 : K = P*H'*(S^-1)
    // 6.1 Get S^-1
    gsl_permutation * perm = gsl_permutation_alloc (EKF_MEASUREMENT_DIM);
    gsl_linalg_LU_decomp_float (&ekf->m_S.matrix, perm, &i);
    gsl_linalg_LU_invert_float (&ekf->m_S.matrix, perm, &ekf->m_tmpS.matrix);				// tmpS = S^-1
    gsl_permutation_free (perm);
    // 6.2 K = P*H'*(S^-1)
    gsl_blas_sgemm (CblasNoTrans, CblasTrans,1.0, &ekf->m_P.matrix, &ekf->m_H.matrix, 0.0, &ekf->m_tmpXY.matrix); // tmpXY = P*H'
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &ekf->m_tmpXY.matrix, &ekf->m_tmpS.matrix, 0.0, &ekf->m_K.matrix); // K = tmpXY * S^-1

    // Step 7 : X = X + KY
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &ekf->m_K.matrix, &ekf->m_Y.matrix, 0.0, &ekf->m_tmpX.matrix); // tmpX = K*Y
    gsl_matrix_float_add (&ekf->m_X.matrix, &ekf->m_tmpX.matrix); // X += tmpX

    // Step 8 : Update P = (I-K*H) * P
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &ekf->m_K.matrix, &ekf->m_H.matrix, 0.0, &ekf->m_tmpXX.matrix); // tmpXX = K*H
    gsl_matrix_float_sub (&ekf->m_tmpXX.matrix, &ekf->m_I.matrix); // tmpXX -= I
    gsl_matrix_float_scale (&ekf->m_tmpXX.matrix, -1.0f); // tmpXX = -tmpXX
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &ekf->m_tmpXX.matrix, &ekf->m_P.matrix, 0.0, &ekf->m_tmpP.matrix); // tmpP = tmpXX * P
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &ekf->m_tmpP.matrix, &ekf->m_I.matrix, 0.0, &ekf->m_P.matrix); // P = I * P;

    // Step 9 : Normalize q
    float qnorm = Common_GetNorm(ekf->X, 4);
    for (i=0; i<4; ++i) ekf->X[i] /= qnorm;
}

void EKF_Quaternion_Euler(EKF_Filter* ekf, float* rpy) {
    rpy[0] = atan2(2 * ekf->X[2] * ekf->X[3] + 2 * ekf->X[0] * ekf->X[1], -2 * ekf->X[1] * ekf->X[1] - 2 * ekf->X[2]* ekf->X[2] + 1) + 3.1415926; // roll
    rpy[1] = asin(-2 * ekf->X[1] * ekf->X[3] + 2 * ekf->X[0]* ekf->X[2]) * (-1); // pitch
    rpy[2] = atan2(2 * ekf->X[1] * ekf->X[2] + 2 * ekf->X[0] * ekf->X[3], -2 * ekf->X[2]*ekf->X[2] - 2 * ekf->X[3]*ekf->X[3] + 1) * (-1); // yaw
}


