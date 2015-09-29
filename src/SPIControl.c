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
*/


#include <unistd.h>
#include <string.h>
#include <bcm2835.h>
#include "SPIControl.h"
#include "RF24_Interface.h"

pthread_mutex_t mutex_SPI;

void MCP3008_init(void) ;
void MCP3008_getRawValue(void) ;
void MCP3008_getRealData(float *volt) ;

void SPIVariables_init(SPIVariables *spi_var) {
/*    for (i=0; i<NPWM; ++i) {
        pwm_pin[i] = pwm_power[i] = 0;
    }*/
    memset(spi_var, 0, sizeof(SPIVariables));
    pthread_mutex_init (&spi_var->mutex, NULL);
    RF24_init();
    MCP3008_init();
}

int SPIVariables_end(SPIVariables *spi_var) {
    while (pthread_mutex_trylock(&mutex_SPI) != 0) delayMicroseconds(100);
    pthread_mutex_unlock (&mutex_SPI);
    pthread_mutex_destroy(&mutex_SPI);
    return pthread_mutex_destroy(&spi_var->mutex);
}

void RF24_Renew(SPIVariables *spi_var) {
    while (pthread_mutex_trylock(&mutex_SPI) != 0) delayMicroseconds(100);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0); //Slave Select on CS0
    RF24_exchangeInfo(&spi_var->control, &spi_var->output);
    pthread_mutex_unlock (&mutex_SPI);
    delay(1);
}

void MCP3008_Renew(SPIVariables *spi_var) {
    while (pthread_mutex_trylock(&mutex_SPI) != 0) delayMicroseconds(1000);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS1); //Slave Select on CS0
    MCP3008_getRawValue();
    pthread_mutex_unlock (&mutex_SPI);
    while (pthread_mutex_trylock(&spi_var->mutex) != 0) delayMicroseconds(1000);
    MCP3008_getRealData(&spi_var->voltage);
    pthread_mutex_unlock (&spi_var->mutex);
    delay(1000);
}
