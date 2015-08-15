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

    int i;
    if (init_all()!=0) return ERROR_INIT;
    PCA9685PW_init(1);
    bcm2835_set_debug(0);

    int on, off;
    if (pca9685PWMReadSingle(0, &on, &off)!=0) return -1;
    printf("Beginning : %02X, %02X\n", on, off);
    on = 0x199 & 0x0FFF;
    off = 0x4CC & 0x0FFF;
    pca9685PWMWriteSingle(0, on++, off++);
    pca9685PWMWriteSingle(1, on++, off++);
    pca9685PWMWriteSingle(4, on++, off++);
    pca9685PWMWriteSingle(5, on++, off++);
    pca9685PWMWriteSingle(8, on++, off++);
    pca9685PWMWriteSingleOff(9, off);

    int data[5];
    data[0] = 0x199 & 0x0FFF;
    data[1] = 0x200 & 0x0FFF;
    data[2] = 0x201 & 0x0FFF;
    data[3] = 0x202 & 0x0FFF;
    data[4] = 0x203 & 0x0FFF;


//    pca9685PWMWriteMultiOff(0, data, 5);

    for (i=0; i<5; ++i) data[i] = 0;

    pca9685PWMReadMultiOff(0, data, 5);

    for (i=0; i<5; ++i) {
	printf("Data : %03X\n", data[i]);
    }

    pca9685PWMWriteSingle(1, on+1, off+1);
    puts("Ohlala!");
    pca9685PWMWriteSingle(4, on, off);

    pca9685PWMReadMultiOff(0, data, 5);

    for (i=0; i<5; ++i) {
        printf("Data : %03X\n", data[i]);
    }

/*
    on = 0 & 0x0FFF;
    off = 0 & 0x0FFF;

    while (1) {
	pca9685PWMWriteSingle(0, on, off);;
	off += 10;
	off &= 0x0FFF;
	usleep(100000);
    }
*/
    if (pca9685PWMReadSingleOff(0, &off)!=0) return -1;
    printf("After : %03X\n", off);
    return 0;
}
