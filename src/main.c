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

    int i, data[5][2], mm[10];
    int pin[] = {0,1,2,3,4,5,6,7,8,9};
    if (init_all()!=0) return ERROR_INIT;
    PCA9685PW_init(1);

    int on, off;
    if (pca9685PWMReadSingleOff(0, mm)!=0) return -1;
    printf("Beginning : %02X, %02X\n", mm[0], mm[1]);
    mm[0] = 0x199 & 0x0FFF;
    mm[1] = 0x4CC & 0x0FFF;

    for (i=0; i<sizeof(pin)/sizeof(pin[0]); ++i) {
	mm[i] = 0x190 + i;
    }

    pca9685PWMWriteMultiOff(pin, mm, 10);

    pca9685PWMReadMulti(pin, data, sizeof(pin)/sizeof(pin[0]));

    for (i=0; i<sizeof(pin)/sizeof(pin[0]); ++i) {
	printf("%dth Data : %03X, %03X\n", i, data[i][0], data[i][1]);
	data[i][0] = 0x4CC + i*2;
    }

    pca9685PWMWriteMulti(pin, data, 5);

    for (i=0; i<sizeof(pin)/sizeof(pin[0]); ++i) {
        printf("%dth Data : %03X, %03X\n", i, data[i][0], data[i][1]);
    }

    return 0;
}
