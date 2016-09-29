/*
 *    Quadcopter -- GY80.h
 *    Description : The function of GY80
 *
 *    Copyright 2015 Wan-Ting CHEN (wanting@gmail.com)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef H_GY80
#define H_GY80

// Switch among different devices
void ADXL345_switch(void);
void L3G4200D_switch(void);
void HMC5883L_switch(void);
void BMP085_switch(void);

void ADXL345_init(void);
int ADXL345_getRawValue(short*) ;
int ADXL345_getRealData(float*) ;
void ADXL345_convertRawToReal(short*, float*);

void L3G4200D_init(void);
int L3G4200D_getRawValue(short*) ;
int L3G4200D_getRealData(float*);
void L3G4200D_convertRawToReal(short*, float*);

void HMC5883L_init(void);
void HMC5883L_singleMeasurement(void);
int HMC5883L_getRawValue(short*);
int HMC5883L_getRealData(float*);
void HMC5883L_getRealData_Direct(float*);
void HMC5883L_convertRawToReal_Zero(short*, float*);
void HMC5883L_convertRawToReal(short*, float*);

void BMP085_switch(void);
void BMP085_init(void);
void BMP085_Trigger_UTemp(void);
void BMP085_Trigger_UPressure(void);
int BMP085_getRawTemp(long* UT);
int BMP085_getRawPressure(long*);
void BMP085_getRealData(long*, long*, float*, long*, float*);

void getAccGyro(float*, float*);
#endif
