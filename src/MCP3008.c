/*
    Quadcopter -- MCP3008.c
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


#include <stdio.h>
#include <bcm2835.h>
#include "SPIControl.h"
#define	FULLVALUE	1024
static char send_buf[3] = {0x01, 0x80, 0x00}, receive_buf[3] = { 0x00, 0x00, 0x00 };
static uint8_t msb, lsb, msbRead, adcRead0;
extern float v_input;

void MCP3008_init(void) {
    bcm2835_spi_chipSelect(BCM2835_SPI_CS1); //Slave Select on CS0
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, LOW);
}

void MCP3008_getRawValue(void) {
    bcm2835_spi_transfernb(send_buf, receive_buf,3);
}

void MCP3008_getRealData(float *volt) {
    msb = (uint8_t)receive_buf[1];
    lsb = (uint8_t)receive_buf[2];
    msbRead = msb & 0b00000011;
    adcRead0 = (msbRead << 8) | lsb;
    *volt = (float)adcRead0 * v_input / FULLVALUE;
}


