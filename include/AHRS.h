/*
    Quadcopter -- AHRS.h
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

#ifndef H_AHRS
#define H_AHRS

typedef struct {
    float q[4], qp[4];
    float half_dt;
    float norm;
    float anorm[3], mnorm[3];
    float v[3], h[3], b[3], w[3];
    float e[3];
    float eInt[3];
    float Kp, Ki;
    float angVel[3];
//    float x[3], vel[3], a[3], accl_ref[3];
} AHRS;

void AHRS_init(AHRS* ahrs, float* angle, float* pi);
void AHRS_setPI(AHRS* ahrs, float Kp, float Ki);
void AHRS_getAngle(AHRS* ahrs, float* angle);
void AHRS_calculate_MagField_Earth(AHRS* ahrs, float* magn);
void AHRS_renew(AHRS* ahrs, float* deltaT, float* accl, float* gyro, float* magn);

#endif
