/*
 * Control.c
 *
 *  Created on: Jul 9, 2023
 *      Author: harsha
 */

#include "Control.h"
#include "stdlib.h"


void UpdateMotorVoltage(int motorIndex,int pwmValue)
{
	char deltaPwm = 5;
	// needs to be here so that when we run diagnostics we can print these values.
	M[motorIndex].presentRpm = GetRPM(&encs,motorIndex);
	//------------------------------------------

	if (M[motorIndex].pwm < pwmValue){
		if (M[motorIndex].pwm < MAX_PWM){
			M[motorIndex].pwm = M[motorIndex].pwm + deltaPwm;
		}
	}
	if (M[motorIndex].pwm > pwmValue){
		if (M[motorIndex].pwm > MIN_PWM){
			M[motorIndex].pwm = M[motorIndex].pwm - deltaPwm;
		}
	}
}


void updateTargets(uint8_t motorIndex,uint8_t rampUp)
{
	M[motorIndex].presentRpm = GetRPM(&encs,motorIndex);
	if(rampUp == 1){
		if (M[motorIndex].intTarget != M[motorIndex].setRpm){
			M[motorIndex].intTarget = (uint16_t)M[motorIndex].intTarget + M[motorIndex].rampRpm;
			if (M[motorIndex].intTarget >  M[motorIndex].setRpm){
					M[motorIndex].intTarget =  M[motorIndex].setRpm;
				}
		}
	}

	if(rampUp == 0){
		if (M[motorIndex].intTarget != 0){
			M[motorIndex].intTarget = (int16_t)M[motorIndex].intTarget - M[motorIndex].rampRpm;
			if (M[motorIndex].intTarget <=  M[motorIndex].rampRpm){
				M[motorIndex].intTarget =  0;
			}
		}
	}
}


void followTargets(uint8_t motorIndex)
{
	// needs to be here so that when we run diagnostics we can print these values.
	M[motorIndex].presentRpm = GetRPM(&encs,motorIndex);
	M[motorIndex].error = M[motorIndex].intTarget - M[motorIndex].presentRpm;
	if (abs(M[motorIndex].error) >= 300){ // if the error is large
		M[motorIndex].overloadCount++;
		if(M[motorIndex].overloadCount >= 20){
			E.RpmErrorFlag = 1;
			S.errStopReason = MOTORARRAY[motorIndex];
			S.errStopReasonHMI = MOTORARRAY_HMI[motorIndex];
			S.errmotorFault = ERR_RPM_ERROR;
			S.errVal = NO_FLOAT_VAR;
			M[motorIndex].overloadCount = 0;
		}
	}else{
			M[motorIndex].overloadCount = 0;
		}

	if (M[motorIndex].antiWindup == 0){
		M[motorIndex].integralError = M[motorIndex].integralError + M[motorIndex].error;
	}

	M[motorIndex].calcPwm = M[motorIndex].Kp*M[motorIndex].error + M[motorIndex].Ki*M[motorIndex].integralError;
	M[motorIndex].feedforward = (uint16_t)(M[motorIndex].intTarget*M[motorIndex].ff_multiplier) ;
	M[motorIndex].pwm = M[motorIndex].startOffset + M[motorIndex].calcPwm + M[motorIndex].feedforward;

	if (M[motorIndex].pwm > MAX_PWM){
		M[motorIndex].antiWindup = 1;
		M[motorIndex].pwm  = MAX_PWM;
	}else{
		M[motorIndex].antiWindup = 0;
	}

	if (M[motorIndex].pwm < M[motorIndex].startOffset){
		M[motorIndex].pwm = M[motorIndex].startOffset;
	}
}


void RaiseFlyerVirtualTarget(int virtualTarget)
{
	if(M[1].intTarget <= virtualTarget)
	{
		M[1].intTarget = M[1].intTarget + M[1].rampRpm;
		if (M[1].intTarget > virtualTarget){
				M[1].intTarget =  virtualTarget;
			}
	}
}



