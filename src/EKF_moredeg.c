#include <stdio.h>
#include <string.h>
#include <math.h>
#include <gsl/gsl_linalg.h>
#include "EKF.h"

#define USE_4TH_RUNGE_KUTTA
//////////////////////////////////////////////////////////////////////////
//all parameters below need to be tune
#define EKF_PQ_INITIAL 0.000001f
#define EKF_PW_INITIAL 0.000010f

#define EKF_QQ_INITIAL 0.000045f
#define EKF_QW_INITIAL 0.00025f

#define EKF_RQ_INITIAL 0.000001f
#define EKF_RA_INITIAL 0.07f
#define EKF_RW_INITIAL 0.525f
#define EKF_RM_INITIAL 0.105f
//////////////////////////////////////////////////////////////////////////

static int i;
static float accl[3], gyro[3], magn[3];

float Vector_Normalize(float* vec, int n) {
    float norm = 0.0f;
    for (i=0; i<n; ++i) {
	norm += pow(vec[i],2);
    }
    return sqrt(norm);
}

int EKF_New(EKF_Filter* ekf)
{
    memset(ekf, sizeof(EKF_Filter), 0);
    float var_mean[3], var_sd[3];
    int (*f)(float*) = 0;
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
    ekf->m_tmpP = gsl_matrix_float_view_array (ekf->tmpP, EKF_STATE_DIM, EKF_STATE_DIM);
    ekf->m_tmpX = gsl_matrix_float_view_array (ekf->tmpX, EKF_STATE_DIM, 1);
    ekf->m_tmpXX = gsl_matrix_float_view_array (ekf->tmpXX, EKF_STATE_DIM, EKF_STATE_DIM);
    ekf->m_tmpXY = gsl_matrix_float_view_array (ekf->tmpXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM);
    ekf->m_tmpYX = gsl_matrix_float_view_array (ekf->tmpYX, EKF_MEASUREMENT_DIM, EKF_STATE_DIM);

    //////////////////////////////////////////////////////////////////////////

    f = L3G4200D_getRealData;
    if ( (ret=Calibration_getSD(gyro, var_mean, var_sd, f))!=0 ) return ret ;
    ekf->Q[32] = var_mean[0]; ekf->Q[40] = var_mean[1]; ekf->Q[48] = var_mean[2];	// process noise of gyro
    ekf->Q[0] = ekf->Q[8] = ekf->Q[16] = ekf->Q[24] = EKF_QQ_INITIAL;			// process noise of Q

    ekf->P[0] = ekf->P[8] = ekf->P[16] = ekf->P[24] = EKF_PQ_INITIAL;
    ekf->P[32] = ekf->P[40] = ekf->P[48] = EKF_PW_INITIAL;
    //
//    ekf->Q[32] = ekf->Q[40] = ekf->Q[48] = EKF_QW_INITIAL;
    //
    ekf->R[0] = ekf->R[14] = ekf->R[28] = ekf->R[42] = EKF_RQ_INITIAL;
    ekf->R[56] = ekf->R[70] = ekf->R[84] = EKF_RA_INITIAL;
    ekf->R[98] = ekf->R[112] = ekf->R[126] = EKF_RW_INITIAL;
    ekf->R[140] = ekf->R[154] = ekf->R[168] = EKF_RM_INITIAL;
    //////////////////////////////////////////////////////////////////////////
    gsl_matrix_float_set_identity(&ekf->m_F.matrix);
    ekf->H[0] = ekf->H[8] = ekf->H[16] = ekf->H[24] = 1.0f; //q row 0~3, col 0~3
    ekf->H[53] = ekf->H[61] = ekf->H[69] = 1.0f; //w row 7~9, col 4~6
    //////////////////////////////////////////////////////////////////////////
    gsl_matrix_float_set_identity(&ekf->m_F.matrix);
    //////////////////////////////////////////////////////////////////////////

    return 0;
}

void EKF_Init(EKF_Filter* ekf, float *q, float *gyro) {
    int i;
    float norm = Vector_Normalize(q, 4);
    for (i=0; i<4; ++i) ekf->X[i] = q[i] / norm;
    for (i=4; i<7; ++i) ekf->X[i] = gyro[i-4];
}

void EFK_Update(EKF_Filter* ekf, float* q, float* gyro, float* accel, float* mag, float* dt) {
    float norm, h[EKF_MEASUREMENT_DIM], halfdx, halfdy, halfdz;
    float halfdtq0, halfdtq1, halfdtq2, halfdtq3;
    float halfdt = 0.5f * *dt;
#ifdef USE_4TH_RUNGE_KUTTA
    float tmpW[4];
#endif
    //////////////////////////////////////////////////////////////////////////
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float _2q0,_2q1,_2q2,_2q3;
    float q0, q1, q2, q3;
    //
    float hx, hy, hz;
    float bx, bz;
    float _2mx, _2my, _2mz;
    //
    //////////////////////////////////////////////////////////////////////////
    halfdx = halfdt * ekf->X[4];
    halfdy = halfdt * ekf->X[5];
    halfdz = halfdt * ekf->X[6];
    //
    q0 = ekf->X[0];
    q1 = ekf->X[1];
    q2 = ekf->X[2];
    q3 = ekf->X[3];

    halfdtq0 = halfdt * q0;
    halfdtq1 = halfdt * q1;
    halfdtq2 = halfdt * q2;
    halfdtq3 = halfdt * q3;

    ekf->F[1] = -halfdx;
    ekf->F[2] = -halfdy;
    ekf->F[3] = -halfdz;
    ekf->F[4] = -halfdtq1;
    ekf->F[5] = -halfdtq2;
    ekf->F[6] = -halfdtq3;

    ekf->F[7] = halfdx;
    ekf->F[9] = -halfdz;
    ekf->F[10] = halfdy;
    ekf->F[11] = halfdtq0;
    ekf->F[12] = halfdtq3;
    ekf->F[13] = -halfdtq2;

    ekf->F[14] = halfdy;
    ekf->F[15] = halfdz;
    ekf->F[17] = -halfdx;
    ekf->F[18] = -halfdtq3;
    ekf->F[19] = halfdtq0;
    ekf->F[20] = halfdtq1;

    ekf->F[21] = halfdz;
    ekf->F[22] = -halfdy;
    ekf->F[23] = halfdx;
    ekf->F[25] = halfdtq2;
    ekf->F[26] = -halfdtq1;
    ekf->F[27] = halfdtq0;

    //model prediction
    //simple way, pay attention!!!
    //according to the actual gyroscope output
    //and coordinate system definition
#ifdef USE_4TH_RUNGE_KUTTA
    tmpW[0] = 0;
    tmpW[1] = ekf->X[4];
    tmpW[2] = ekf->X[5];
    tmpW[3] = ekf->X[6];
    Quaternion_RungeKutta4(ekf->X, tmpW, dt, 1);
#else
    ekf->X[0] = q0 - (halfdx * q1 + halfdy * q2 + halfdz * q3);
    ekf->X[1] = q1 + (halfdx * q0 + halfdy * q3 - halfdz * q2);
    ekf->X[2] = q2 - (halfdx * q3 - halfdy * q0 - halfdz * q1);
    ekf->X[3] = q3 + (halfdx * q2 - halfdy * q1 + halfdz * q0);
    //////////////////////////////////////////////////////////////////////////
    //Re-normalize Quaternion
    norm = Vector_Normalize(ekf->X, 4);
    for (i=0; i<4; ++i) ekf->X[i] /= norm;
#endif
    //X covariance matrix update based on model
    //P = F*P*F' + Q;

    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &ekf->m_F.matrix, &ekf->m_P.matrix, 0.0, &ekf->m_tmpP.matrix);
    gsl_blas_sgemm (CblasNoTrans, CblasTrans,1.0, &ekf->m_tmpP.matrix, &ekf->m_F.matrix, 0.0, &ekf->m_P.matrix);
    gsl_matrix_float_add (&ekf->m_P.matrix, &ekf->m_Q.matrix);

    //arm_mat_add_f32(&ekf->P, &ekf->Q, &ekf->tmpP);

    //////////////////////////////////////////////////////////////////////////
    //model and measurement differences
    //normalize acc and mag measurements
    norm = Vector_Normalize(accel, 3);
    for (i=0; i<3; ++i) accel[i] /= norm;
    //////////////////////////////////////////////////////////////////////////
    norm = Vector_Normalize(mag, 3);
    for (i=0; i<3; ++i) mag[i] /= norm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * ekf->X[0];
    _2q1 = 2.0f * ekf->X[1];
    _2q2 = 2.0f * ekf->X[2];
    _2q3 = 2.0f * ekf->X[3];
    //
    q0q0 = ekf->X[0] * ekf->X[0];
    q0q1 = ekf->X[0] * ekf->X[1];
    q0q2 = ekf->X[0] * ekf->X[2];
    q0q3 = ekf->X[0] * ekf->X[3];
    q1q1 = ekf->X[1] * ekf->X[1];
    q1q2 = ekf->X[1] * ekf->X[2];
    q1q3 = ekf->X[1] * ekf->X[3];
    q2q2 = ekf->X[2] * ekf->X[2];
    q2q3 = ekf->X[2] * ekf->X[3];
    q3q3 = ekf->X[3] * ekf->X[3];

    _2mx = 2.0f * mag[0];
    _2my = 2.0f * mag[1];
    _2mz = 2.0f * mag[2];

    //Reference direction of Earth's magnetic field
    hx = _2mx * (0.5f - q2q2 - q3q3) + _2my * (q1q2 - q0q3) + _2mz *(q1q3 + q0q2);
    hy = _2mx * (q1q2 + q0q3) + _2my * (0.5f - q1q1 - q3q3) + _2mz * (q2q3 - q0q1);
    hz = _2mx * (q1q3 - q0q2) + _2my * (q2q3 + q0q1) + _2mz *(0.5f - q1q1 - q2q2);

    bx = sqrt(hx * hx + hy * hy);
    bz = hz;

    h[0] = ekf->X[0];
    h[1] = ekf->X[1];
    h[2] = ekf->X[2];
    h[3] = ekf->X[3];

    h[4] = 2.0f * (q1q3 - q0q2);
    h[5] = 2.0f * (q2q3 + q0q1);
    h[6] = -1.0f + 2.0f * (q0q0 + q3q3);

    h[7] = ekf->X[4];
    h[8] = ekf->X[5];
    h[9] = ekf->X[6];

    h[10] = bx * (1.0f - 2.0f * (q2q2 + q3q3)) + bz * ( 2.0f * (q1q3 - q0q2));
    h[11] = bx * (2.0f * (q1q2 - q0q3)) + bz * (2.0f * (q2q3 + q0q1));
    h[12] = bx * (2.0f * (q1q3 + q0q2)) + bz * (1.0f - 2.0f * (q1q1 + q2q2));

    /////////////////////////////////////////////////////////////////////////
    //The H matrix maps the measurement to the states 13 x 7
    //row started from 0 to 12, col started from 0 to 6
    //row 4, col 0~3
    ekf->H[28] = -_2q2; ekf->H[29] = _2q3; ekf->H[30] = -_2q0; ekf->H[31] = _2q1;
    //row 5, col 0~3
    ekf->H[35] = _2q1; ekf->H[36] = _2q0; ekf->H[37] = _2q3; ekf->H[38] = _2q2;
    //row 6, col 0~3
    ekf->H[42] = _2q0; ekf->H[43] = -_2q1; ekf->H[44] = -_2q2;	ekf->H[45] = _2q3;
    //row 10, col 0~3
    ekf->H[70] = bx * _2q0 - bz * _2q2;
    ekf->H[71] = bx * _2q1 + bz * _2q3;
    ekf->H[72] = -bx * _2q2 - bz * _2q0;
    ekf->H[73] = bz * _2q1 - bx * _2q3;
    //row 11, col 0~3
    ekf->H[77] = bz * _2q1 - bx * _2q3;
    ekf->H[78] = bx * _2q2 + bz * _2q0;
    ekf->H[79] = bx * _2q1 + bz * _2q3;
    ekf->H[80] = bz * _2q2 - bx * _2q0;
    //row 12, col 0~3
    ekf->H[84] = bx * _2q2 + bz * _2q0;
    ekf->H[85] = bx * _2q3 - bz * _2q1;
    ekf->H[86] = bx * _2q0 - bz * _2q2;
    ekf->H[87] = bx * _2q1 + bz * _2q3;
    //
    //y = z - h;
    ekf->Y[0] = q[0] - h[0];
    ekf->Y[1] = q[1] - h[1];
    ekf->Y[2] = q[2] - h[2];
    ekf->Y[3] = q[3] - h[3];
    //
    ekf->Y[4] = accel[0] - h[4];
    ekf->Y[5] = accel[1] - h[5];
    ekf->Y[6] = accel[2] - h[6];
    ekf->Y[7] = gyro[0] - h[7];
    ekf->Y[8] = gyro[1] - h[8];
    ekf->Y[9] = gyro[2] - h[9];
    //////////////////////////////////////////////////////////////////////////
    ekf->Y[10] = mag[0] - h[10];
    ekf->Y[11] = mag[1] - h[11];
    ekf->Y[12] = mag[2] - h[12];

    //////////////////////////////////////////////////////////////////////////
    //Measurement covariance update
    //S = H*P*H' + R;
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &ekf->m_H.matrix, &ekf->m_P.matrix, 0.0, &ekf->m_tmpYX.matrix);
    gsl_blas_sgemm (CblasNoTrans, CblasTrans,1.0, &ekf->m_tmpYX.matrix, &ekf->m_H.matrix, 0.0, &ekf->m_tmpS.matrix);
    gsl_matrix_float_add (&ekf->m_tmpS.matrix, &ekf->m_R.matrix);
    //arm_mat_add_f32(&ekf->S, &ekf->R, &ekf->tmpS);

    //Calculate Kalman gain
    //K = P*H'/S;
    gsl_permutation * perm = gsl_permutation_alloc (EKF_MEASUREMENT_DIM);
    gsl_linalg_LU_decomp_float (&ekf->m_tmpS.matrix, perm, &i);
    gsl_linalg_LU_invert_float (&ekf->m_tmpS.matrix, perm, &ekf->m_S.matrix);
    gsl_permutation_free (perm);

    gsl_blas_sgemm (CblasNoTrans, CblasTrans,1.0, &ekf->m_P.matrix, &ekf->m_H.matrix, 0.0, &ekf->m_tmpXY.matrix);
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &ekf->m_tmpXY.matrix, &ekf->m_S.matrix, 0.0, &ekf->m_K.matrix);
    //Corrected model prediction
    //S = S + K*y;
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &ekf->m_K.matrix, &ekf->m_Y.matrix, 0.0, &ekf->m_tmpX.matrix);
    gsl_matrix_float_add (&ekf->m_X.matrix, &ekf->m_tmpX.matrix);
    //normalize quaternion
    norm = Vector_Normalize(ekf->X, 4);
    for (i=0; i<4; ++i) ekf->X[i] /= norm;

    //Update state covariance with new knowledge
    //option: P = P - K*H*P or P = (I - K*H)*P*(I - K*H)' + K*R*K'
    //P=(I - K*H)*P*(I - K*H)' + K*R*K'
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &ekf->m_K.matrix, &ekf->m_H.matrix, 0.0, &ekf->m_tmpXX.matrix);
    gsl_matrix_float_sub (&ekf->m_tmpXX.matrix, &ekf->m_I.matrix);
    gsl_matrix_float_scale (&ekf->m_tmpXX.matrix, -1.0f);
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &ekf->m_tmpXX.matrix, &ekf->m_P.matrix, 0.0, &ekf->m_tmpP.matrix);
    gsl_blas_sgemm (CblasNoTrans, CblasTrans,1.0, &ekf->m_tmpP.matrix, &ekf->m_tmpXX.matrix, 0.0, &ekf->m_P.matrix);
    gsl_blas_sgemm (CblasNoTrans, CblasNoTrans,1.0, &ekf->m_K.matrix, &ekf->m_R.matrix, 0.0, &ekf->m_tmpXY.matrix);
    gsl_blas_sgemm (CblasNoTrans, CblasTrans,1.0, &ekf->m_tmpXY.matrix, &ekf->m_K.matrix, 0.0, &ekf->m_tmpXX.matrix);
    gsl_matrix_float_add (&ekf->m_P.matrix, &ekf->m_tmpXX.matrix);
}

void EKF_GetAngle(EKF_Filter* ekf, float* rpy)
{
    Quaternion_ToEuler(&ekf->X, rpy);
    for (i=0; i<3; ++i) rpy[i] = EKF_TODEG(rpy[i]);
}

