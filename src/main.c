/*
    Quadcopter -- main.c
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


#include <time.h>
#include <stdio.h>
#include <math.h>
//#include <string.h>
#include <unistd.h>
#include <bcm2835.h>
#include <pthread.h>
#include "Setup.h"
#include "I2CControl.h"
#include "Initialization.h"
#include "Calibration.h"
#include "SPIControl.h"
#include "Device.h"

#define	LINESIZE	256
#define RAD_TO_DEG      (180/M_PI)

/* A mutex protecting job_queue. */

char Pause(void) {
    char c;
    puts("Start measuring... Press Enter to continue");
    while ( (c=getchar()) != '\n' ){}
    return c;
}

void* Renew_acclgyro_cycle(void *data) {
    I2CVariables* var = (I2CVariables*) data;
    while (1) {
	Renew_acclgyro(var);
    }
}

void* Renew_magn_cycle(void *data) {
    I2CVariables* var = (I2CVariables*) data;
    while (1) {
        Renew_magn(var);
    }
}

void* Renew_baro_cycle(void *data) {
    I2CVariables* var = (I2CVariables*) data;
    while(1) {
	Renew_baro(var);
    }
}

int main(void) {
    struct timespec tp1, tp2;
    unsigned long startTime, procesTime;
    int command=0, angle=0;
    float deltaT = 2.5e-3;
    I2CVariables i2c_var;
    I2CVariblesCali i2c_cali;

    pthread_t t_magn, t_baro;

    int i=0,j,ret,s;
    if ( (ret=init_all(&i2c_var)) !=0) return ret;
    printf("FIRST -- PWM setting is : %d, %d, %d, %d\n", i2c_var.PWM_power[0], i2c_var.PWM_power[1], i2c_var.PWM_power[2], i2c_var.PWM_power[3]);
    do {
	Renew_PWM_read(&i2c_var);
	printf("PWM setting is : %d, %d, %d, %d\n", i2c_var.PWM_power[0], i2c_var.PWM_power[1], i2c_var.PWM_power[2], i2c_var.PWM_power[3]);
        puts("Input a number");
        scanf("%d", &s);
        printf("You have input %d\n", s);
	i2c_var.PWM_power[1]=s;
//	pca9685PWMWriteMultiOff(i2c_var.PWM_pin, i2c_var.PWM_power, 4);
	//pca9685PWMWriteSingleOff(i2c_var.PWM_pin[0], i2c_var.PWM_power[0]);
	Renew_PWM(&i2c_var);
    } while (s>0);

    puts("Done!");
    return end_all(&i2c_var);
}

