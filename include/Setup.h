#ifndef H_SETUP
#define H_SETUP

int DEBUG_MODE		=	1;
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
float v_input		= 5.3;

float Kp		= 3.5 ;//2.0; //3.5
float Ki		= 0.005; //0.006; // 0.001
float Kd		= 0;

//float pid_setting[]	= {20, 0, 0, 400, 0, 0, 20.0, 0.005};
//float pid_setting[]   = {20, 0, 0, 140, 2, 80, 30.0, 10};
float pid_setting[]   = {20, 0, 0, 480, 9, 250, 15.0, 4};

#endif

// Test until 160, 60
// 210, 70
// 220, 60
//240, 160

// Kp~190 start oscillation
// 220 : symmetric osci
// Good : 340, 7, 90
// 360 7 110 -> D = 200 -> Too much damping
// 360, 7, 130 Good
// 490 is too much
// 470 300 9 is good
