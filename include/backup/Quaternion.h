#include <math.h>
#ifndef _QUATERNION_H_
#define _QUATERNION_H_

inline void Quaternion_Add(float *r, float *a, float *b) {
        r[0] = a[0] + b[0];
        r[1] = a[1] + b[1];
        r[2] = a[2] + b[2];
        r[3] = a[3] + b[3];
}

inline void Quaternion_Sub(float *r, float *a, float *b) {
        r[0] = a[0] - b[0];
        r[1] = a[1] - b[1];
        r[2] = a[2] - b[2];
        r[3] = a[3] - b[3];
}

inline void Quaternion_Multiply(float *r, float *a, float *b) {
        r[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
        r[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
        r[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
        r[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

inline void Quaternion_Conjugate(float *r, float *a) {
        r[0] = a[0];
        r[1] = -a[1];
        r[2] = -a[2];
        r[3] = -a[3];
}

inline void Quaternion_Scalar(float *r, float *q, float scalar) {
        r[0] = q[0] * scalar;
        r[1] = q[1] * scalar;
        r[2] = q[2] * scalar;
        r[3] = q[3] * scalar;
}

#endif

