#include "Filter_Digital.h"

static float A1, A2, B0, B1, B2;

void Filter_init(void) {
    A1 = 2 * ZETA / OMEGA_N;
    A2 = 1.0 / OMEGA_N / OMEGA_N;
    B0 = A2/TS/TS - A1/TS + 1;
    B1 = -2*A2/TS/TS + A1/TS;
    B2 = A2/TS/TS;
}

void Filter_renew(Filter_Digital *f, float *rawdata, float *estimated, unsigned long *N) {
    if (*N<2) {
	*estimated = *rawdata;
    } else {
	*estimated = -B1/B2*f->estimated_previous[0] -B0/B2*f->estimated_previous[1] + 1/B2*(*rawdata);
    }

    f->estimated_previous[1] = f->estimated_previous[0];
    f->estimated_previous[0] = *estimated;
}
