/*
    Quadcopter -- RF24_Interface.h
    Auther: Wan-Ting CHEN (wanting@gmail.com)

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
#ifndef H_RF24_INTERFACE
#define H_RF24_INTERFACE
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

void RF24_init(void);
void RF24_exchangeInfo(int *in, int *out);
//void MCP3008_init(void) ;
//void MCP3008_getRawValue(void) ;
//void MCP3008_getRealData(float *volt) ;

#ifdef __cplusplus
}
#endif
#endif
