// ERROR CODE
#define	ERROR_INIT	-1

// Functions of ADXL345
//extern void ADXL345_init(int i);
//extern int ADXL345_getRealData(double* acceleration);

// Functions of L3G4200D
//extern void L3G4200D_init(int i);
//extern int L3G4200D_getRealData(double* angVel);

// Functions of HMC5883L
//extern void HMC5883L_init(int i);
//extern int HMC5883L_getRawValue(short* mag);

// Functions of BMP085
//extern void BMP085_init(int i);
//extern int BMP085_getRealData(double *RTD, long *RP, double *altitude);

// Funciton of PCA9685PW
extern void PCA9685PW_init(int i);
extern int PCA9685PWMFreq(void);
extern void PCA9685PW_PWMReset(void);
extern int pca9685PWMReadSingle(int pin, int *on, int *off);
extern int pca9685PWMReadSingleOff(int pin, int *off);
extern int pca9685PWMReadMulti(int pin, int *data, int num);
extern int pca9685PWMReadMultiOff(int pin, int *data, int num);
extern void pca9685PWMWriteSingle(int pin, int on, int off);
extern void pca9685PWMWriteSingleOff(int pin, int off);
extern void pca9685PWMWriteMulti(int pin, int *data, int num); // if pin = 0, num = 3, data will be : on_0, off_0, on_1, off_1, on_2, off_2
extern void pca9685PWMWriteMultiOff(int pin, int *data, int num); // if pin = 0, num = 3, data will be : on_0, off_0, on_1, off_1, on_2, off_2
extern int baseReg(int pin);
