/*
    Quadcopter -- RF24_Interface.cpp
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
#include "SPIControl.h"
#include <RF24/RF24.h>

using namespace std;

static RF24 radio(RPI_BPLUS_GPIO_J8_15,RPI_BPLUS_GPIO_J8_24, BCM2835_SPI_SPEED_8MHZ);
// Radio pipe addresses for the 2 nodes to communicate.
static const uint8_t pipes[][6] = {"1Node","2Node"};
void RF24_init(void) {
    radio.begin();
    radio.setDataRate(RF24_250KBPS);
    radio.setRetries(15,15);

    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);

    radio.printDetails();
    radio.startListening();
}

void RF24_exchangeInfo(int *in, int *out) {
    if ( radio.available() ) {
    // Fetch the payload, and see if this was the last one.
    	while(radio.available()){
            radio.read(in, sizeof(int));
        }
        radio.stopListening();
        radio.write(out, sizeof(int));
    // Now, resume listening so we catch the next packets.
	radio.startListening();
//	usleep(1000);
    // Spew it
    }
}
