#include "UKF.h"
#define USE_4TH_RUNGE_KUTTA
//////////////////////////////////////////////////////////////////////////
//all parameters below need to be tune
#define UKF_PQ_INITIAL 0.000001f
#define UKF_PW_INITIAL 0.000001f

#define UKF_QQ_INITIAL 0.0000045f
#define UKF_QW_INITIAL 0.0000025f

#define UKF_RQ_INITIAL 0.000001f
#define UKF_RA_INITIAL 0.07f
#define UKF_RW_INITIAL 0.0525f
#define UKF_RM_INITIAL 0.105f

#define UKF_alpha (1.0f)
#define UKF_beta (2.0f)
#define UKF_kappa (-1.0f)

void UKF_New(UKF* ukf){
    //scaling factor
    float lambda = UKF_alpha * UKF_alpha *((double)UKF_STATE_DIM + UKF_kappa) - (double)UKF_STATE_DIM;
    float gamma = (double) UKF_STATE_DIM + lambda;

    //weights for means
    ukf->Wm0 = lambda / gamma;
    ukf->Wmi = 0.5f / gamma;

    //weights for covariance
    ukf->m_Wc = gsl_matrix_alloc (UKF_SP_POINTS, UKF_SP_POINTS);
    ukf->Wc = ukf->m_Wc.data;
    arm_mat_init_f32(&ukf->Wc, UKF_SP_POINTS, UKF_SP_POINTS, ukf->Wc_f32);
        arm_mat_identity_f32(&ukf->Wc, ukf->Wmi);
        Wc[0] = ukf->Wm0 + (1.0f - UKF_alpha * UKF_alpha + UKF_beta);

}
