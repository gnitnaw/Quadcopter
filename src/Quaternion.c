#include <math.h>
#include "Quaternion.h"

static Quaternion quaternion;

void Quaternion_init(double* accl_est, double* angle) {
// Q is estimated by alpha = beta_z
    quaternion.q[0] = cos(angle/2);
    quaternion.q[1] = sin(angle/2);
    quaternion.q[2] = sin(angle/2);
    quaternion.q[3] = sin(angle/2) * cos(angle);
}
