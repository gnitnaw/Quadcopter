#include <stdio.h>
#include <math.h>
#include "Quaternion.h"

static int i, j, k;
float getNorm(float* vec) {
    return sqrt(pow(vec[0],2)+pow(vec[1],2)+pow(vec[2],2)+pow(vec[3],2));
}

void Quaternion_Normalize(float *q)
{
    float norm = 1.0 / getNorm(q);
    for (i=0; i<4; ++i) q[i] *= norm;
}

void Quaternion_FromEulerAngle(float *q, float *rpy) {
    double dCos[3], dSin[3];
    for (i=0; i<3; ++i) {
        dCos[i] = cos(rpy[i] * 0.5);
        dSin[i] = sin(rpy[i] * 0.5);
    }
    q[0] = dCos[0] * dCos[1] * dCos[2] + dSin[0] * dSin[1] * dSin[2];
    q[1] = dSin[0] * dCos[1] * dCos[2] - dCos[0] * dSin[1] * dSin[2];
    q[2] = dCos[0] * dSin[1] * dCos[2] + dSin[0] * dCos[1] * dSin[2];
    q[3] = dCos[0] * dCos[1] * dSin[2] - dSin[0] * dSin[1] * dCos[2];
}

void Quaternion_ToEuler(float *q, float *rpy) {
    rpy[0] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1); // roll
    rpy[1] = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2]); // pitch
    rpy[2] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3]*q[3] + 1); // yaw
}

// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
void Quaternion_FromRotationMatrix(float *R, float *Q) {
    // calculate the trace of the matrix
/*
    float trace = R[0] + R[4] + R[8];
    float s;
    if(trace > 0){
	s = 0.5f / sqrt(trace + 1.0f);
        Q[0] = 0.25f / s;
        Q[1] = (R[7] - R[5]) * s;
        Q[2] = (R[2] - R[6]) * s;
        Q[3] = (R[3] - R[1]) * s;
    }
    else{
	if ( R[0] > R[4] && R[0] > R[8] ) {
            s = 0.5f * sqrt(1.0f + R[0] - R[4] - R[8]);
	    Q[0] = (R[7] - R[5]) * s;
            Q[1] = 0.25f / s;
            Q[2] = (R[1] + R[3]) * s;
            Q[3] = (R[2] + R[6]) * s;
	} else if(R[4] > R[8]) {
	    s = 0.5f * sqrt(1.0f + R[4] - R[0] - R[8]);
	    Q[0] = (R[2] - R[6]) * s;
	    Q[1] = (R[1] + R[3]) * s;
	    Q[2] = 0.25f / s;
	    Q[3] = (R[5] + R[7]) * s;
	} else{
	    s = 0.5f * sqrt(1.0f + R[8] - R[0] - R[4]);
	    Q[0] = (R[3] - R[1]) * s;
	    Q[1] = (R[2] + R[6]) * s;
	    Q[2] = (R[5] + R[7]) * s;
            Q[3] = 0.25f / s;
	}
    }
*/

        // get the instantaneous orientation quaternion
        float fq0sq; // q0^2
        float recip4q0; // 1/4q0
        float fmag; // quaternion magnitude
 	float SMALLQ0 = 0.01f; // limit where rounding errors may appear
        // get q0^2 and q0
        fq0sq = 0.25f * (1.0f + R[0] + R[4] + R[8]);
        Q[0] = (float)sqrt(abs(fq0sq));
        // normal case when q0 is not small meaning rotation angle not near 180 deg
        if (Q[0] > SMALLQ0){
                // calculate q1 to q3
                recip4q0 = 0.25f / Q[0];
                Q[1] = recip4q0 * (R[5] - R[7]);
                Q[2] = recip4q0 * (R[6] - R[2]);
                Q[3] = recip4q0 * (R[1] - R[3]);
        }
        else{
                // special case of near 180 deg corresponds to nearly symmetric matrix
                // which is not numerically well conditioned for division by small q0
                // instead get absolute values of q1 to q3 from leading diagonal
                Q[1] = sqrt(abs(0.5f * (1.0f + R[0]) - fq0sq));
                Q[2] = sqrt(abs(0.5f * (1.0f + R[4]) - fq0sq));
                Q[3] = sqrt(abs(0.5f * (1.0f + R[8]) - fq0sq));
                // first assume q1 is positive and ensure q2 and q3 are consistent with q1
                if ((R[1] + R[3]) < 0.0f){
                        // q1*q2 < 0 so q2 is negative
                        Q[2] = -Q[2];
                        if ((R[5] + R[7]) > 0.0f){
                                // q1*q2 < 0 and q2*q3 > 0 so q3 is also both negative
                                Q[3] = -Q[3];
                        }
                }
                else if ((R[1] + R[3]) > 0.0f){
                        if ((R[5] + R[7]) < 0.0f){
                                // q1*q2 > 0 and q2*q3 < 0 so q3 is negative
                                Q[3] = -Q[3];
                        }
                }
                // negate the vector components if q1 should be negative
                if ((R[5] - R[7]) < 0.0f){
                        Q[1] = -Q[1];
                        Q[2] = -Q[2];
                        Q[3] = -Q[3];
                }
        }
        // finally re-normalize
        fmag = sqrt(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
        Q[0] /= fmag;
        Q[1] /= fmag;
        Q[2] /= fmag;
        Q[3] /= fmag;

}


void Quaternion_RungeKutta4(float *q, float *w, float *dt, int normalize)
{
    float half = 0.5f;
    float two = 2.0f;
    float qw[4], k2[4], k3[4], k4[4];
    float tmpq[4], tmpk[4];

    //qw = q * w * half;
    Quaternion_Multiply(qw, q, w);
    Quaternion_Scalar(qw, qw, half);
    //k2 = (q + qw * dt * half) * w * half;
    Quaternion_Scalar(tmpk, qw, *dt * half);
    Quaternion_Add(tmpk, q, tmpk);
    Quaternion_Multiply(k2, tmpk, w);
    Quaternion_Scalar(k2, k2, half);
    //k3 = (q + k2 * dt * half) * w * half;
    Quaternion_Scalar(tmpk, k2, *dt * half);
    Quaternion_Add(tmpk, q, tmpk);
    Quaternion_Multiply(k3, tmpk, w);
    Quaternion_Scalar(k3, k3, half);
    //k4 = (q + k3 * dt) * w * half;
    Quaternion_Scalar(tmpk, k3, *dt);
    Quaternion_Add(tmpk, q, tmpk);
    Quaternion_Multiply(k4, tmpk, w);
    Quaternion_Scalar(k4, k4, half);
    //q += (qw + k2 * two + k3 * two + k4) * (dt / 6);
    Quaternion_Scalar(tmpk, k2, two);
    Quaternion_Add(tmpq, qw, tmpk);
    Quaternion_Scalar(tmpk, k3, two);
    Quaternion_Add(tmpq, tmpq, tmpk);
    Quaternion_Add(tmpq, tmpq, k4);
    Quaternion_Scalar(tmpq, tmpq, *dt / 6.0f);
    Quaternion_Add(q, q, tmpq);

    if (normalize){
        Quaternion_Normalize(q);
    }
}

