//#include <stdio.h>
//#include <math.h>
#include "Filter_Kalman.h"

void Kalman_init(Filter_Kalman* k, float var, float noise) {
    k->data_measured = var;
    k->data_noise = noise;
    k->data_updated = 0;
    k->P_EC = var * 0.1;
    k->K_factor = k->P_EC / (k->P_EC+k->data_noise);
    k->data_estimated = k->data_updated + k->K_factor * (k->data_measured-k->data_updated);
    k->P_EC_estimated = (1-k->K_factor)*k->P_EC;
}

void Kalman_renew(Filter_Kalman* k, float* var_m, float* var_e ) {
    k->data_measured = *var_m;
    k->data_updated = k->data_estimated;
    k->P_EC = k->P_EC_estimated;
    k->K_factor = k->P_EC / (k->P_EC+k->data_noise);
    k->data_estimated = k->data_updated + k->K_factor * (k->data_measured-k->data_updated);
    k->P_EC_estimated = (1-k->K_factor)*k->P_EC;
    *var_e = k->data_estimated;
}
