/*
    Quadcopter -- GY80.h
    Description :
    The function of GY80

    Copyright 2015 Wan-Ting CHEN (wanting@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef H_GY80
#define H_GY80

void ADXL345_switch(void);
void ADXL345_init(void);
int ADXL345_getRawValue(short* acc) ;
int ADXL345_getRealData(float* acceleration) ;
void ADXL345_convertRawToReal(short *acc, float* acceleration);

void L3G4200D_switch(void);
void L3G4200D_init(void);
int L3G4200D_getRawValue(short* gyr) ;
int L3G4200D_getRealData(float* angVel) ;
void L3G4200D_convertRawToReal(short *gyr, float* angVel);

void HMC5883L_switch(void);
void HMC5883L_init(void);
void HMC5883L_singleMeasurement(void);
int HMC5883L_getRawValue(short* magn);
int HMC5883L_getRealData(float* magn);
void HMC5883L_getRealData_Direct(float* magn);
void HMC5883L_convertRawToReal_Zero(short *mag, float* magn);
void HMC5883L_convertRawToReal(short *magn, float* magn_real);

void BMP085_switch(void);
void BMP085_init(void);
void BMP085_Trigger_UTemp(void);
void BMP085_Trigger_UPressure(void);
int BMP085_getRawTemp(long* UT);
int BMP085_getRawPressure(long* UP);
void BMP085_getRealData(long *UT, long *UP, float *RTD, long *RP, float *altitude);

void getAccGyro(float *accl, float *gyro);
#endif
