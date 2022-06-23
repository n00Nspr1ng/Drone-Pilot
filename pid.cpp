/******************************************************************************
 The pid.c in RaspberryPilot project is placed under the MIT license

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
#include <math.h>
#include <string.h>
#include "commonLib.h"
#include "pid.h"


#define DEFAULT_ROLL_ATTITUDE_P_GAIN 0.0
#define DEFAULT_ROLL_ATTITUDE_I_GAIN 0.0
#define DEFAULT_ROLL_ATTITUDE_D_GAIN 0.0
#define DEFAULT_ROLL_ATTITUDE_I_LIMIT 0.0

#define DEFAULT_PITCH_ATTITUDE_P_GAIN 0.0
#define DEFAULT_PITCH_ATTITUDE_I_GAIN 0.0
#define DEFAULT_PITCH_ATTITUDE_D_GAIN 0.0
#define DEFAULT_PITCH_ATTITUDE_I_LIMIT 0.0

#define DEFAULT_ROLL_ATTITUDE_SP 0.0
#define DEFAULT_PITCH_ATTITUDE_SP 0.0
#define DEFAULT_ROLL_ATTITUDE_SHIFT 0.0
#define DEFAULT_PITCH_ATTITUDE_SHIFT  0.0
#define DEFAULT_ROLL_ATTITUDE_DEADBAND 0.5
#define DEFAULT_PITCH_ATTITUDE_DEADBAND 0.5

PID_STRUCT rollAttitudePidSettings;
PID_STRUCT pitchAttitudePidSettings;
PID_STRUCT yawAttitudePidSettings;

#define DEFAULT_ROLL_RATE_P_GAIN 0.0
#define DEFAULT_ROLL_RATE_I_GAIN 0.0
#define DEFAULT_ROLL_RATE_D_GAIN 0.0
#define DEFAULT_ROLL_RATE_I_LIMIT 0.0

#define DEFAULT_PITCH_RATE_P_GAIN 0.0
#define DEFAULT_PITCH_RATE_I_GAIN 0.0
#define DEFAULT_PITCH_RATE_D_GAIN 0.0
#define DEFAULT_PITCH_RATE_I_LIMIT 0.0

#define DEFAULT_YAW_RATE_P_GAIN 0.0
#define DEFAULT_YAW_RATE_I_GAIN 0.0
#define DEFAULT_YAW_RATE_D_GAIN 0.0
#define DEFAULT_YAW_RATE_I_LIMIT 0.0

#define DEFAULT_ROLL_RATE_SP 0.0
#define DEFAULT_PITCH_RATE_SP 0.0
#define DEFAULT_YAW_RATE_SP 0.0
#define DEFAULT_ROLL_RATE_SHIFT 0.0
#define DEFAULT_PITCH_RATE_SHIFT 0.0
#define DEFAULT_YAW_RATE_SHIFT 0.0
#define DEFAULT_ROLL_RATE_DEADBAND 1.5
#define DEFAULT_PITCH_RATE_DEADBAND 1.5
#define DEFAULT_YAW_RATE_DEADBAND 1.5

PID_STRUCT rollRatePidSettings;
PID_STRUCT pitchRatePidSettings;
PID_STRUCT yawRatePidSettings;


void pidInit() {

	//init PID controler for attitude	
	pidTune(&rollAttitudePidSettings, DEFAULT_ROLL_ATTITUDE_P_GAIN,
		DEFAULT_ROLL_ATTITUDE_I_GAIN, DEFAULT_ROLL_ATTITUDE_D_GAIN,
		DEFAULT_ROLL_ATTITUDE_SP, DEFAULT_ROLL_ATTITUDE_SHIFT,
		DEFAULT_ROLL_ATTITUDE_I_LIMIT, DEFAULT_ROLL_ATTITUDE_DEADBAND);
	pidTune(&pitchAttitudePidSettings, DEFAULT_PITCH_ATTITUDE_P_GAIN,
		DEFAULT_PITCH_ATTITUDE_I_GAIN, DEFAULT_PITCH_ATTITUDE_D_GAIN,
		DEFAULT_PITCH_ATTITUDE_SP, DEFAULT_PITCH_ATTITUDE_SHIFT,
		DEFAULT_PITCH_ATTITUDE_I_LIMIT, DEFAULT_PITCH_ATTITUDE_DEADBAND);
	resetPidRecord(&rollAttitudePidSettings);
	resetPidRecord(&pitchAttitudePidSettings);

	//init PID controler for rate
	pidTune(&rollRatePidSettings, DEFAULT_ROLL_RATE_P_GAIN,
		DEFAULT_ROLL_RATE_I_GAIN, DEFAULT_ROLL_RATE_D_GAIN,
		DEFAULT_ROLL_RATE_SP, DEFAULT_ROLL_RATE_SHIFT,
		DEFAULT_ROLL_RATE_I_LIMIT, DEFAULT_ROLL_RATE_DEADBAND);
	pidTune(&pitchRatePidSettings, DEFAULT_PITCH_RATE_P_GAIN,
		DEFAULT_PITCH_RATE_I_GAIN, DEFAULT_PITCH_RATE_D_GAIN,
		DEFAULT_PITCH_RATE_SP, DEFAULT_PITCH_RATE_SHIFT,
		DEFAULT_PITCH_RATE_I_LIMIT, DEFAULT_PITCH_RATE_DEADBAND);
	pidTune(&yawRatePidSettings, DEFAULT_YAW_RATE_P_GAIN,
		DEFAULT_YAW_RATE_I_GAIN, DEFAULT_YAW_RATE_D_GAIN,
		DEFAULT_YAW_RATE_SP, DEFAULT_YAW_RATE_SHIFT,
		DEFAULT_YAW_RATE_I_LIMIT, DEFAULT_YAW_RATE_DEADBAND);
	resetPidRecord(&rollRatePidSettings);
	resetPidRecord(&pitchRatePidSettings);
	resetPidRecord(&yawRatePidSettings);

}


float pidCalculation(PID_STRUCT *pid, float processValue, float timeDiff ,bool outputP, bool outputI, bool outputD)
{

	float pterm = 0.f;
	float dterm = 0.f;
	float iterm = 0.f;
	float result = 0.f;

	pid->pv = processValue;

	//P term
	if (outputP) {
		pid->err = deadband((pid->sp + pid->spShift) - (pid->pv), pid->deadBand)/2.0;
		pterm = pid->pgain * pid->err;
	}

	//I term
	if (outputI) {
		pid->integral += (pid->err * timeDiff/1000.0);
		pid->integral = LIMIT_MIN_MAX_VALUE(pid->integral, -pid->iLimit, pid->iLimit);
		iterm = pid->igain * pid->integral;
	}

	//D term
	if (outputD) {
		dterm = (pid->err - pid->last_error) / NON_ZERO(timeDiff/1000.0);
		dterm = pid->dgain * dterm;
		pid->last_error = pid->err;
	}

	result = (pterm + iterm + dterm);

	return result;
}


void pidTune(PID_STRUCT *pid, float p_gain, float i_gain, float d_gain, float set_point, float shift, float iLimit, float deadBand) {

	pid->pgain = p_gain;
	pid->igain = i_gain;
	pid->iLimit = iLimit;
	pid->dgain = d_gain;
	pid->sp = set_point;
	pid->spShift = shift;
	pid->deadBand = deadBand;
}


void resetPidRecord(PID_STRUCT *pid) {
	pid->integral = 0.f;
	pid->err = 0.f;
	pid->last_error = 0.f;
	//pid->last_tv = 0;
}


void setPidDeadBand(PID_STRUCT *pi, float value) {
	pi->deadBand = value;
}


float getPidDeadBand(PID_STRUCT *pi) {
	return pi->deadBand;
}


void setPidError(PID_STRUCT *pi, float value) {
	pi->err = value;
}


float getPidSperror(PID_STRUCT *pi) {
	return pi->err;
}


void setPidSp(PID_STRUCT *pid, float set_point) {
	pid->sp = set_point;
}


float getPidSp(PID_STRUCT *pid) {
	return pid->sp;
}


void setName(PID_STRUCT *pid, char *name) {
	strcpy(pid->name, name);
}


char *getName(PID_STRUCT *pid) {
	return pid->name;
}


void setPGain(PID_STRUCT *pid, float gain) {
	pid->pgain = gain;
}


float getPGain(PID_STRUCT *pid) {
	return pid->pgain;
}


void setIGain(PID_STRUCT *pid, float gain) {
	pid->igain = gain;
}


float getIGain(PID_STRUCT *pid) {
	return pid->igain;
}


void setILimit(PID_STRUCT *pid, float v) {
	pid->iLimit = v;
}


float getILimit(PID_STRUCT *pid) {
	return pid->iLimit;
}


void setDGain(PID_STRUCT *pid, float gain) {
	pid->dgain = gain;
}


float getDGain(PID_STRUCT *pid) {
	return pid->dgain;
}
