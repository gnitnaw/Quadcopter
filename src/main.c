#include <stdio.h>
#include <string.h>
#include <bcm2835.h>
#include "main.h"
#include "Setup.h"
#include "PCA9685PW.h"
/*
extern void PCA9685PW_init(int i);
extern void PCA9685PW_PWMReset(void);
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

    char i;
    if (init_all()!=0) return ERROR_INIT;
    PCA9685PW_init(1);

    unsigned short on, off;
    if (pca9685PWMReadSingle(0, &on, &off)!=0) return -1;
    printf("Beginning : %02X, %02X\n", on, off);
    on = 409 & 0x0FFF;
    off = 1228 & 0x0FFF;
    pca9685PWMWriteSingle(0, &on, &off);
    printf("Write : %02X, %02X\n", on, off);

    if (pca9685PWMReadSingle(0, &on, &off)!=0) return -1;
    printf("After : %02X, %02X\n", on, off);
    return 0;
}
