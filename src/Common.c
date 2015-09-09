#include <math.h>

float Common_GetNorm(float* var, size_t n) {
    int i;
    float sum = 0;
    for (i=0; i<n; ++i) sum += var[i]*var[i];
    return sqrtf(sum);
}
