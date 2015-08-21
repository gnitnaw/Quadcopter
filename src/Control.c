#include <stdio.h>
#include <math.h>

void ComplementaryFilter(double* accl, double* gyro, double *pitch, double *roll, double *yaw) {
    *roll += gyro[0] + atan2f(accl[1], accl[2]) * 180 / M_PI;
    *pitch += gyro[1] + atan2f(accl[2], accl[0]) * 180 / M_PI;
    *yaw += gyro[2] + atan2f(accl[0], accl[1]) * 180 / M_PI;
}

