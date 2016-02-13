/*
    Quadcopter -- SPIControl.c
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

typedef struct {
    char horDirection[2];
    char verDirection;
    char rotateDirection;
    unsigned int power;
    unsigned char switchValue;
} Command;

*/


#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <sched.h>
#include <bcm2835.h>
#include "SPIControl.h"
#include "RF24_Interface.h"

//pthread_mutex_t mutex_SPI;
static unsigned char spi_stat = 0;

void MCP3008_init(void) ;
void MCP3008_getRawValue(void) ;
void MCP3008_getRealData(float *volt) ;

void SPIVariables_init(SPIVariables *spi_var) {
/*    for (i=0; i<NPWM; ++i) {
        pwm_pin[i] = pwm_power[i] = 0;
    }*/
    memset(spi_var, 0, sizeof(SPIVariables));
    RF24_init();
    MCP3008_init();
}

int SPIVariables_end(SPIVariables *spi_var) {
    pthread_mutex_destroy(&spi_var->mutex);
    bcm2835_spi_end();

    return 0;
}

int RF24_Renew(SPIVariables *spi_var) {
    while ( __sync_lock_test_and_set(&spi_stat, 1) ) sched_yield() ;
//    bcm2835_spi_chipSelect(BCM2835_SPI_CS0); //Slave Select on CS0
//    RF24_exchangeInfo(&spi_var->control, &spi_var->output);
    int iR = RF24_receiveInfo(spi_var->control, 4);
    if (iR>0) Decode_Command(spi_var);
    __sync_lock_release(&spi_stat);
    return iR;
}

void MCP3008_Renew(SPIVariables *spi_var) {
    while ( __sync_lock_test_and_set(&spi_stat, 1) ) sched_yield() ;
    bcm2835_spi_chipSelect(BCM2835_SPI_CS1); //Slave Select on CS1
    MCP3008_getRawValue();
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0); //Slave Select on CS0
    __sync_lock_release(&spi_stat);
    while (pthread_mutex_trylock(&spi_var->mutex) != 0);
    MCP3008_getRealData(&spi_var->voltage);
    pthread_mutex_unlock (&spi_var->mutex);
}

void Decode_Command(SPIVariables *spi_var) {
    spi_var->com.horDirection[0] = (signed char)((spi_var->control[0]>>4) & 0xF) -1;
    spi_var->com.horDirection[1] = (signed char)(spi_var->control[0] &0xF) -1;
    spi_var->com.angle_expect[0] = 5 * spi_var->com.horDirection[0];
    spi_var->com.angle_expect[1] = 5 * spi_var->com.horDirection[1];
    spi_var->com.power = 1640 + 1640*spi_var->control[2]/254;
    spi_var->com.switchValue = spi_var->control[3]>>6 ;
}
