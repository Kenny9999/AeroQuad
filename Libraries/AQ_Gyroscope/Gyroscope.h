/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef _AEROQUAD_GYROSCOPE_H_
#define _AEROQUAD_GYROSCOPE_H_

#include "Arduino.h"
#include "GlobalDefined.h"

#define FINDZERO 512

int32_t gyroADCData[3] = {0,0,0};
int32_t gyroADC[3] = {0,0,0};
int32_t previousGyroADC[3] = {0,0,0};
int32_t seccondPreviousGyroADC[3] = {0,0,0};

double gyroRate[3] = {0.0,0.0,0.0};
int32_t  gyroZero[3] = {0,0,0};
int32_t  gyroSample[3] = {0,0,0};
double gyroScaleFactor = 0.0;
double gyroOneMeterSecADCFactor = 0.0;
double gyroHeading = 0.0;
unsigned long gyroLastMesuredTime = 0;
byte gyroSampleCount = 0;

void measureGyroSum();
void evaluateGyroRate();
void initializeGyro();
void measureGyro();
void gyroUpdateHeading();
boolean calibrateGyro();


void processGyroCommon() {
	
  gyroSampleCount = 0;
  for (byte axis = 0; axis <= ZAXIS; axis++) {
    gyroSample[axis] = 0;
    gyroADCData[axis] = (gyroADC[axis] + previousGyroADC[axis] + seccondPreviousGyroADC[axis]) / 3;
	seccondPreviousGyroADC[axis] = previousGyroADC[axis];
	previousGyroADC[axis] = gyroADC[axis];
	gyroRate[axis] = gyroADCData[axis] * gyroScaleFactor;
  }
  gyroUpdateHeading();
}



#endif