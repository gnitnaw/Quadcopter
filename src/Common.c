/*
    Quadcopter -- Common.c
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

#include <math.h>
#include <unistd.h>
#include <time.h>
#include "Common.h"

float Common_GetNorm(float* var, unsigned int n) {
    int i;
    float sum = 0;
    for (i=0; i<n; ++i) sum += var[i]*var[i];
    return sqrtf(sum);
}

void _usleep(int milisec)
{
    struct timespec req = {0};
    req.tv_sec = 0;
    req.tv_nsec = milisec * 1000L;
    nanosleep(&req, (struct timespec *)NULL);
}

