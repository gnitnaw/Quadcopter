int ADXL345_RATE	=	200;
int L3G4200D_RATE	=	200;
int L3G4200D_RANGE	=	250;
int HMC5883L_RATE	=	75;
int PCA9685PW_FREQ	=	400;

//float mag_offset[]	= {-270, -115.5, 20.5};
//float mag_gain[]	= {1, 0.9844, 1.1371};
float mag_offset[]	= {-276.919983, -137.080002, -82.799988};
float mag_gain[] 	= {1.000000, 0.992958, 1.128000};

float Kp		= 4.0;
float Ki		= 0.0005;
float Kd		= 0;
