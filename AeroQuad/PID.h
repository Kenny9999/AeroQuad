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

#ifndef _AQ_PID_H_
#define _AQ_PID_H_

enum {
  RATE_XAXIS_PID_IDX = 0,
  RATE_YAXIS_PID_IDX,
  ZAXIS_PID_IDX,
  ATTITUDE_XAXIS_PID_IDX,
  ATTITUDE_YAXIS_PID_IDX,
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    BARO_ALTITUDE_HOLD_PID_IDX,
    ZDAMPENING_PID_IDX,
  #endif
  #if defined AltitudeHoldRangeFinder
    SONAR_ALTITUDE_HOLD_PID_IDX,
  #endif
  #if defined UseGPS
    GPSPITCH_PID_IDX,
    GPSROLL_PID_IDX,
    GPSYAW_PID_IDX,
  #endif    

  LAST_PID_IDX  // keep this definition at the end of this enum
};

//// PID Variables
struct PIDdata {
  float P, I, D;
  double lastError;
  int32_t previousPIDTime;
  double integratedError;
  double delta[2];
} PID[LAST_PID_IDX];

// This struct above declares the variable PID[] to hold each of the PID values for various functions
// The following constants are declared in AeroQuad.h
// ROLL = 0, PITCH = 1, YAW = 2 (used for Arcobatic Mode, gyros only)
// ROLLLEVEL = 3, PITCHLEVEL = 4, LEVELGYROROLL = 6, LEVELGYROPITCH = 7 (used for Stable Mode, accels + gyros)
// HEADING = 5 (used for heading hold)
// ALTITUDE = 8 (used for altitude hold)
// ZDAMPENING = 9 (used in altitude hold to dampen vertical accelerations)

float updatePID(double targetPosition, double currentPosition, struct PIDdata *PIDparameters, double errorTreshold = 0) {

  unsigned long currentPidTime = micros();
  const double deltaPIDTime = (currentPidTime - PIDparameters->previousPIDTime) / 1000000.0;
  PIDparameters->previousPIDTime = currentPidTime;  // AKA PID experiments
  
  const double error = targetPosition - currentPosition;

  if (inFlight) {
    PIDparameters->integratedError += (error * deltaPIDTime);
    if (errorTreshold != 0) {
      PIDparameters->integratedError = constrain(PIDparameters->integratedError, -errorTreshold, errorTreshold);
    }
  }
  else {
    PIDparameters->integratedError = 0.0;
  }
  
  double delta = currentPosition - PIDparameters->lastError;
  double deltaSum = PIDparameters->delta[1] + PIDparameters->delta[0] + delta;
  PIDparameters->delta[1] = PIDparameters->delta[0];
  PIDparameters->delta[0] = delta;

  double dTerm = -PIDparameters->D * (deltaSum) / (deltaPIDTime * 100); // dT fix from Honk
  PIDparameters->lastError = currentPosition;
  
  return (PIDparameters->P * error) + (PIDparameters->I * PIDparameters->integratedError) + dTerm;
}


void zeroIntegralError() __attribute__ ((noinline));
void zeroIntegralError() {
  for (byte axis = 0; axis <= ATTITUDE_YAXIS_PID_IDX; axis++) {
    PID[axis].integratedError = 0;
    PID[axis].previousPIDTime = currentTime;
  }
}

#endif // _AQ_PID_H_


