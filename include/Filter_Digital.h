#ifndef H_FILTER_DIGITAL
#define H_FILTER_DIGITAL

#include <math.h>
#define OMEGA_N	80*M_PI
#define ZETA	0.5
#define TS	0.004
typedef struct
{
    float estimated_previous[2];
} Filter_Digital;

void Filter_init(void) ;
void Filter_renew(Filter_Digital *f, float *rawdata, float *estimated, unsigned long* N) ;

#endif
