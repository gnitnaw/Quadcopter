#include <stdio.h>
#include <string.h>
#include <bcm2835.h>
#include "main.h"
#include "Setup.h"
#include "PCA9685PW.h"
/*
extern int pca9685PWMReadSingle(short pin, short *on, short *off);
extern int pca9685PWMReadSingleOff(short pin, int *off);
extern int pca9685PWMReadMulti(short pin, short *data, int num);
extern int pca9685PWMReadMultiOff(short pin, int *data, int num);
extern void pca9685PWMWriteSingle(short pin, short *on, short *off);
extern void pca9685PWMWriteSingleOff(short pin, int *off);
extern void pca9685PWMWriteMulti(short pin, short *data, int num); // if pin = 0, num = 3, data will be : on_0, off_0, on_1, off_1, on_2, off_2
extern void pca9685PWMWriteMultiOff(short pin, int *data, int num); // if pin = 0, num = 3, data will be : on_0, off_0, on_1, off_1, on_2, off_2

*/
int main(void) {

    int i, ret;
    if ( (ret=init_all()) !=0) return ret;

    return 0;
}
