/******************************************************************************
 The flyControler.c in RaspberryPilot project is placed under the MIT license

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
//#include <pthread.h>
//#include <Serial.h>
//#include <iostream>
#include <time.h>
#include "commonLib.h"
//#include "motorControl.h"
//#include "systemControl.h"
//#include "MPU9250_9Axis_MotionApps41.h"
//#include "attitudeUpdate.h"
#include "pid.h"
//#include "altHold.h"
#include "flyControler.h"

#define DEFAULT_ADJUST_PERIOD 1
#define DEFAULT_GYRO_LIMIT 50
#define DEFAULT_ANGULAR_LIMIT 5000

//static float* getGyro_Data(void);
//
//pthread_mutex_t controlMotorMutex;

//static bool leaveFlyControler;
//static float rollAttitudeOutput;
//static float pitchAttitudeOutput;
//static float yawAttitudeOutput;
//static float altHoltAltOutput;
//static float yawCenterPoint;
//static float maxThrottleOffset;

//int16_t ax, ay, az, gx, gy, gz, mx, my, mz;

/**
 * Init paramtes and states for flyControler
 *
 * @param
 * 		void
 *
 * @return
 *		bool
 *
 */
//void flyControlerInit() {
//
//	//setLeaveFlyControlerFlag(false);
//	//disenableFlySystem();
//	//rollAttitudeOutput = 0.f;
//	//pitchAttitudeOutput = 0.f;
//	//yawAttitudeOutput = 0.f;
//	//altHoltAltOutput = 0.f;
//	//maxThrottleOffset = 1000.f;
//
//}

/**
 *  get the output of attitude PID controler, this output will become  a input for angular velocity PID controler
 *
 * @param
 * 		void
 *
 * @return 
 *		value
 *
 */
void getAttitudePidOutput(float* rollAttitudeOutput, float* pitchAttitudeOutput, PID_STRUCT* rollAttitudePidSettings, PID_STRUCT* pitchAttitudePidSettings, float Roll, float Pitch, float timeDiff) {

	*rollAttitudeOutput = pidCalculation(rollAttitudePidSettings, Roll, timeDiff, true, true, true);
	*pitchAttitudeOutput = pidCalculation(pitchAttitudePidSettings, Pitch, timeDiff, true, true, true);
	//yawAttitudeOutput =	pidCalculation(&yawAttitudePidSettings, Yaw, true, true, true);

}

/**
 * get the output of angular velocity PID controler
 *
 * @param rollRateOutput
 * 		output of roll angular velocity PID controler
 *
 * @param pitchRateOutput
 * 		output of pitch angular velocity PID controler
 *
 * @param yawRateOutput
 * 		output of yaw angular velocity PID controler
 */
void getRatePidOutput(float* rollRateOutput, float* pitchRateOutput, float* yawRateOutput, PID_STRUCT* rollRatePidSettings, PID_STRUCT* pitchRatePidSettings, PID_STRUCT* yawRatePidSettings, float rate_roll, float rate_pitch, float rate_yaw, float timeDiff) {

//  setPidSp(&rollRatePidSettings, rollAttitudeOutput);
//  setPidSp(&pitchRatePidSettings, pitchAttitudeOutput);
	//setPidSp(&yawRatePidSettings, yawAttitudeOutput);

	//float* Gxyz = getGyro_Data();
	*rollRateOutput = pidCalculation(rollRatePidSettings, rate_roll, timeDiff, true, true, true);
	*pitchRateOutput = pidCalculation(pitchRatePidSettings, rate_pitch, timeDiff, true, true, true);
	*yawRateOutput = pidCalculation(yawRatePidSettings, rate_yaw, timeDiff, true, true, true);

}



//float* getGyro_Data(void)
//{
//	MPU9250.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
//
//	float Gxyz[3];
//	Gxyz[0] = (double)gx * 250 / 32768;
//	Gxyz[1] = (double)gy * 250 / 32768;
//	Gxyz[2] = (double)gz * 250 / 32768;
//
//	return Gxyz;
//}
