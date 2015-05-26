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

#ifndef _AEROQUAD_ACCELEROMETER_H_
#define _AEROQUAD_ACCELEROMETER_H_

#include "Arduino.h"
#include "GlobalDefined.h"

#define SAMPLECOUNT 400.0

double accelScaleFactor[3] = {0.0,0.0,0.0};
double runTimeAccelBias[3] = {0, 0, 0};
double accelOneG = 0.0;
double meterPerSecSec[3] = {0.0,0.0,0.0};
long accelADC[3] = {0,0,0};
long accelSample[3] = {0,0,0};
long previousAccelSample[3] = {0,0,0};
long previousPreviousAccelSample[3] = {0,0,0};
byte accelSampleCount = 0;
  
void initializeAccel();
void measureAccel();
void measureAccelSum();
void evaluateMetersPerSec();
void computeAccelBias();
  
#endif