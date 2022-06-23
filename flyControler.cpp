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
