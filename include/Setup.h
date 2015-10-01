int ADXL345_RATE	=	400;
int L3G4200D_RATE	=	400;
int L3G4200D_RANGE	=	250;
int HMC5883L_RATE	=	75;
int PCA9685PW_FREQ	=	400;

int PWM_CHANNEL[]	= {0,1,2,3};
//float mag_offset[]	= {-270, -115.5, 20.5};
//float mag_gain[]	= {1, 0.9844, 1.1371};
float mag_offset[]	= {-276.919983, -137.080002, -82.799988};
float mag_gain[] 	= {1.000000, 0.992958, 1.128000};
float v_input		= 5.0;

float Kp		= 2.0;
float Ki		= 0.0005;
float Kd		= 0;

float pid_setting[]	= {20, 1, 1, 50, 1, 1, 2.0, 0.0005};
