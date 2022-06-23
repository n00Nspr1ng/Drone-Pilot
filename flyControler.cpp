/******************************************************************************
 The commonLib.h in RaspberryPilot project is placed under the MIT license

 Copyright (c) 2016 jellyice1986 (Tung-Cheng Wu)

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "pid.h"
#include "flyControler.h"

#define DEFAULT_ADJUST_PERIOD 1
#define DEFAULT_GYRO_LIMIT 50
#define DEFAULT_ANGULAR_LIMIT 5000


void getAttitudePidOutput(float* rollAttitudeOutput, float* pitchAttitudeOutput, PID_STRUCT* rollAttitudePidSettings, PID_STRUCT* pitchAttitudePidSettings, float Roll, float Pitch, float timeDiff) {

	*rollAttitudeOutput = pidCalculation(rollAttitudePidSettings, Roll, timeDiff, true, true, true);
	*pitchAttitudeOutput = pidCalculation(pitchAttitudePidSettings, Pitch, timeDiff, true, true, true);
	//yawAttitudeOutput =	pidCalculation(&yawAttitudePidSettings, Yaw, true, true, true);

}


void getRatePidOutput(float* rollRateOutput, float* pitchRateOutput, float* yawRateOutput, PID_STRUCT* rollRatePidSettings, PID_STRUCT* pitchRatePidSettings, PID_STRUCT* yawRatePidSettings, float rate_roll, float rate_pitch, float rate_yaw, float timeDiff) {

	*rollRateOutput = pidCalculation(rollRatePidSettings, rate_roll, timeDiff, true, true, true);
	*pitchRateOutput = pidCalculation(pitchRatePidSettings, rate_pitch, timeDiff, true, true, true);
	*yawRateOutput = pidCalculation(yawRatePidSettings, rate_yaw, timeDiff, true, true, true);

}
