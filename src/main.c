#include <stdio.h>
#include <string.h>
#include "main.h"
#include "Setup.h"

int main(void) {

    short mag[3];
    double accl[3], gyro[3];
    int i, ret;
    long RP;
    double RTD, altitude;
    if (init_all()!=0) return ERROR_INIT;
    ADXL345_init(1);
    L3G4200D_init(1);
    HMC5883L_init(1);
    BMP085_init(1);

    for (i=0; i<10; ++i) {
	ret = ADXL345_getRealData(accl);
	ret = L3G4200D_getRealData(gyro);
	printf("accl : %f\t%f\t%f, ", accl[0], accl[1], accl[2]);
	printf("gyro : %f\t%f\t%f\n", gyro[0], gyro[1], gyro[2]);
    }
    HMC5883L_getRawValue(mag);
    printf("mag : %d\t%d\t%d\n", mag[0], mag[1], mag[2]);
    ret = BMP085_getRealData(&RTD, &RP, &altitude);
    printf("RT : %f, RP = %ld, altitude = %f\n", RTD, RP, altitude);
    return 0;
}
