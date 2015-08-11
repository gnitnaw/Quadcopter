#include <stdio.h>
#include <string.h>
#include "main.h"
#include "GY80.h"
#include "Setup.h"

int main(void) {

    short accl[3], gyro[3], mag[3];
    int i, ret;
    if (init_all()!=0) return ERROR_INIT;
    ADXL345_init(1);
    L3G4200D_init(1);
    HMC5883L_init(1);
    BMP085_init(1);

    for (i=0; i<10; ++i) {
	ret = ADXL345_getRawValue(acc);
	ret = L3G4200D_getRawValue(gyco);
    }
    return 0;
}
