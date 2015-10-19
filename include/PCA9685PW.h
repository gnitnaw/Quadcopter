/*
    Quadcopter -- GY80_Address.h
    Description :
    The functions of PCA9685PW

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

#ifndef H_PCA9685PW
#define H_PCA9685PW
void PCA9685PW_switch(void);
void PCA9685PW_init(void);

int PCA9685PWMFreq(void);
void PCA9685PW_PWMReset(void);

int pca9685PWMReadSingle(int pin, int *data);
int pca9685PWMReadSingleOff(int pin, int *off);
int pca9685PWMReadMulti(int* pin, int data[][2], int num);
int pca9685PWMReadMultiOff(int* pin, int *data, int num);
void pca9685PWMWriteSingle(int pin, int* data);
void pca9685PWMWriteSingleOff(int pin, int off);
void pca9685PWMWriteMulti(int *pin, int data[][2], int num);
void pca9685PWMWriteMultiOff(int *pin, int *data, int num);

#endif
