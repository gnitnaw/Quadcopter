#include <math.h>
#include "Common.h"
float Common_GetNorm(float* var, unsigned int n) {
    int i;
    float sum = 0;
    for (i=0; i<n; ++i) sum += var[i]*var[i];
    return sqrtf(sum);
}
