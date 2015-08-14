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
extern int pca9685PWMReadSingle(char pin, unsigned short *on, unsigned short *off);
extern int pca9685PWMReadSingleOff(char pin, int *off);
extern int pca9685PWMReadMulti(char pin, unsigned short *data, int num);
extern int pca9685PWMReadMultiOff(char pin, int *data, int num);
extern void pca9685PWMWriteSingle(char pin, unsigned short *on, unsigned short *off);
extern void pca9685PWMWriteSingleOff(char pin, int *off);
extern void pca9685PWMWriteMulti(char pin, unsigned short *data, int num); // if pin = 0, num = 3, data will be : on_0, off_0, on_1, off_1, on_2, off_2
extern void pca9685PWMWriteMultiOff(char pin, int *data, int num); // if pin = 0, num = 3, data will be : on_0, off_0, on_1, off_1, on_2, off_2
extern char baseReg(char pin);
