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
//#include <time.h>
#include "commonLib.h"
#include "pid.h"

/**
 *	Default PID parameter for attitude
 */
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

/**
 *	Default PID parameter for rate
 */
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


/**
 *  Init  PID controler
 *
 * @param
 * 		void
 *
 * @return
 *		 void
 *
 */
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
	//setName(&rollAttitudePidSettings, "ROLL_A");
	//setName(&pitchAttitudePidSettings, "PITCH_A");
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
	//setName(&rollRatePidSettings, "ROLL_R");
	//setName(&pitchRatePidSettings, "PITCH_R");
	//setName(&yawRatePidSettings, "YAW_R");
	resetPidRecord(&rollRatePidSettings);
	resetPidRecord(&pitchRatePidSettings);
	resetPidRecord(&yawRatePidSettings);

}

/**
 * PID conrroler
 *
 * @param pid
 *		 pid entity
 *
 * @param processValue
 *		input of PID controler
 *
 * @return
 *		output of PID controler
 *
 */
float pidCalculation(PID_STRUCT *pid, float processValue, float timeDiff ,bool outputP, bool outputI, bool outputD)
{

	float pterm = 0.f;
	float dterm = 0.f;
	float iterm = 0.f;
	float result = 0.f;

	//float timeDiff = 0.f;
	//struct timeval_1 tv;

	//tv = micros();
	//gettimeofday(&tv, NULL);

	//if (TIME_IS_UPDATED(pid->last_tv)) {

		pid->pv = processValue;

		//timeDiff = (tv - pid->last_tv)/1000.0;
		//timeDiff = GET_SEC_TIMEDIFF(tv, pid->last_tv);

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

	//}

	//pid->last_tv = tv;
	//UPDATE_LAST_TIME(tv, pid->last_tv);

	return result;
}

/**
 * tune PID conrroler
 *
 * @param pid
 *		 pid entity
 *
 * @param p_gain
 *		 P item
 *
 * @param i_gain
 *		 I item
 *
 * @param d_gain
 *		 D item
 *
 * @param set_point
 *		 set point
 *
 * @param shift
 *		 a shift for set point
 *
 * @param iLimit
 *		limition for output of I item
 *
 * @param deadBand
 *		dead band
 *
 * @return
 *		output of PID controler
 *
 */
void pidTune(PID_STRUCT *pid, float p_gain, float i_gain, float d_gain, float set_point, float shift, float iLimit, float deadBand) {

	pid->pgain = p_gain;
	pid->igain = i_gain;
	pid->iLimit = iLimit;
	pid->dgain = d_gain;
	pid->sp = set_point;
	pid->spShift = shift;
	pid->deadBand = deadBand;
}

/**
 *  reset PID record
 *
 * @param pid
 * 		PID record
 *
 * @return
 *		 void
 *
 */
void resetPidRecord(PID_STRUCT *pid) {
	pid->integral = 0.f;
	pid->err = 0.f;
	pid->last_error = 0.f;
	//pid->last_tv = 0;
}

/**
 *  update PID tv
 *
 * @param pid
 * 		PID record
 *
 * @return
 *		 void
 *
 */
//void updatePidTv(PID_STRUCT *pid) {
//
//	//int tv = micros();
//	struct timeval_1 tv;
//	gettimeofday(&tv, NULL);
//
//	//pid->last_tv = tv;
//	UPDATE_LAST_TIME(tv, pid->last_tv);
//}

/**
 *  set dead band of PID controler
 *
 * @param pi
 * 		PID entity
 *
 * @param value
 * 		dead band of PID controler
 *
 * @return
 *		 void
 *
 */
void setPidDeadBand(PID_STRUCT *pi, float value) {
	pi->deadBand = value;
}

/**
 *  get dead band of PID controler
 *
 * @param pi
 * 		PID entity
 *		
 * @return
 *		 dead band of PID controler
 *
 */
float getPidDeadBand(PID_STRUCT *pi) {
	return pi->deadBand;
}

/**
 *  set error to PID controler
 *
 * @param pi
 * 		PID entity
 *
 * @param value
 * 		error for PID controler
 *
 * @return
 *		 void
 *
 */
void setPidError(PID_STRUCT *pi, float value) {
	pi->err = value;
}

/**
 *  get error from PID record
 *
 * @param pi
 * 		PID entity
 *
 * @return
 *		 error
 *
 */
float getPidSperror(PID_STRUCT *pi) {
	return pi->err;
}

/**
 *  set set point
 *
 * @param pid
 * 		PID entity
 *
 * @param set_point
 * 		set point
 *
 * @return
 *		 void
 *
 */
void setPidSp(PID_STRUCT *pid, float set_point) {
	pid->sp = set_point;
}

/**
 *  get set point
 *
 * @param pid
 * 		PID entity
 *
 * @return
 *		 set point
 *
 */
float getPidSp(PID_STRUCT *pid) {
	return pid->sp;
}

/**
 *  name a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @param name
 * 		name
 *
 * @return
 *		 void
 *
 */
void setName(PID_STRUCT *pid, char *name) {
	strcpy(pid->name, name);
}

/**
 *  get the name of a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @return
 *		 name
 *
 */
char *getName(PID_STRUCT *pid) {
	return pid->name;
}

/**
 *  set P for a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @param gain
 * 		P item
 *
 * @return
 *		 void
 *
 */
void setPGain(PID_STRUCT *pid, float gain) {
	pid->pgain = gain;
}

/**
 *  get P from a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @return
 *		 P item
 *
 */
float getPGain(PID_STRUCT *pid) {
	return pid->pgain;
}

/**
 *  set I for a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @param gain
 * 		I item
 *
 * @return
 *		 void
 *
 */
void setIGain(PID_STRUCT *pid, float gain) {
	pid->igain = gain;
}

/**
 *  get I from a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @return
 *		 I item
 *
 */
float getIGain(PID_STRUCT *pid) {
	return pid->igain;
}

/**
 *  set I limitation for a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @param gain
 * 		I limitation
 *
 * @return
 *		 void
 *
 */
void setILimit(PID_STRUCT *pid, float v) {
	pid->iLimit = v;
}

/**
 *  get limitation of I from a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @return
 *		limitation of I item
 *
 */
float getILimit(PID_STRUCT *pid) {
	return pid->iLimit;
}

/**
 *  set D for a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @param gain
 * 		D item
 *
 * @return
 *		 void
 *
 */
void setDGain(PID_STRUCT *pid, float gain) {
	pid->dgain = gain;
}

/**
 *  get D from a pid entity
 *
 * @param pid
 * 		PID entity
 *
 * @return
 *		 D item
 *
 */
float getDGain(PID_STRUCT *pid) {
	return pid->dgain;
}
