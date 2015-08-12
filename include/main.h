// ERROR CODE
#define	ERROR_INIT	-1

// Functions of ADXL345
extern void ADXL345_init(int i);
extern int ADXL345_getRealData(double* acceleration);

// Functions of L3G4200D
extern void L3G4200D_init(int i);
extern int L3G4200D_getRealData(double* angVel);

// Functions of HMC5883L
extern void HMC5883L_init(int i);
extern int HMC5883L_getRawValue(short* mag);

// Functions of BMP085
extern void BMP085_init(int i);
extern int BMP085_getRealData(double *RTD, long *RP, double *altitude);

