
//#include "MotorStruct.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "logicDefines.h"
#include "encoder.h" 
#include "initialize.h"
#include "functionDefines.h"
#include "Structs.h"
#include "CommonConstants.h"
#include "HMI_Constants.h"
#include "HMI_Fns.h"
#include <stdlib.h>
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern int filter1;
extern int startFlag;
extern int currentLayer;
extern float currentLength;

extern char MOTORARRAY[];
extern char MOTORARRAY_HMI[];

int errorCount = 0;

extern int bobbin_DC_Offset;
extern int backRoller_DC_Offset ;

extern int rightpulseCount;
extern int leftpulseCount;



void AllTimerOn(void)
{
		MotorTimer(MOTOR1_TIMER_ON);
		MotorTimer(MOTOR2_TIMER_ON);
		MotorTimer(MOTOR3_TIMER_ON);
		MotorTimer(MOTOR4_TIMER_ON);
		MotorTimer(MOTOR5_TIMER_ON);
}



/********************************************StepMotorPID1 function START*************************************************/
void StepMotorPID(uint8_t motorIndex)
{
	unsigned int calcPwm = 0;
	
	M[motorIndex].presentRpm = GetRPM(&encs, motorIndex);
	
	M[motorIndex].intTarget = M[motorIndex].setRpm;

	M[motorIndex].error = M[motorIndex].intTarget - M[motorIndex].presentRpm;
	
	
	/*if (__fabs(M[motorIndex].error) >= 300)
	{
		errorCount++;
		if(errorCount >= 20)
		{
		E.RpmErrorFlag = 1;
		S.errStopReason = MOTORARRAY[motorIndex];
		S.errStopReasonHMI = MOTORARRAY_HMI[motorIndex];
		S.errmotorFault = ERR_RPM_ERROR;	
		S.errVal = NO_FLOAT_VAR;	
		errorCount = 0;
		}
	}*/
	
	M[motorIndex].integralError = (M[motorIndex].integralError + M[motorIndex].error);
	
	calcPwm = M[motorIndex].Kp*M[motorIndex].error + M[motorIndex].Ki*M[motorIndex].integralError;
	
	if ((calcPwm <= MAX_PWM) && (calcPwm > 0))
	{
		M[motorIndex].pwm = calcPwm;
	}
	if(calcPwm <= 0)
	{
		M[motorIndex].pwm = 0;
	}
}



/********************************************StepMotorPID function START************************************/

void UpdateMotorPID_w_DCOffset(uint8_t motorIndex,int dc_offset)
{
	unsigned int calcPwm = 0;
	M[motorIndex].presentRpm = GetRPM(&encs,motorIndex);
	
	if((startFlag == 1) && ((M[motorIndex].intTarget <= M[motorIndex].setRpm)))
	{
		M[motorIndex].intTarget = M[motorIndex].intTarget + M[motorIndex].rampRpm;
		if (M[motorIndex].intTarget >  M[motorIndex].setRpm){
				M[motorIndex].intTarget =  M[motorIndex].setRpm;
			}	
	}
		
	if((startFlag == 0)&&((M[motorIndex].intTarget > 0)))
	{
		M[motorIndex].intTarget = M[motorIndex].intTarget - M[motorIndex].rampRpm;
		if (M[motorIndex].intTarget <=  M[motorIndex].rampRpm){
				M[motorIndex].intTarget =  0;
			}
	}
	
	M[motorIndex].error = M[motorIndex].intTarget - M[motorIndex].presentRpm;
	
	if (fabs(M[motorIndex].error) >= 300)
	{ M[motorIndex].overloadCount++;
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
	
	
	M[motorIndex].integralError = (M[motorIndex].integralError + M[motorIndex].error);
	
	calcPwm = M[motorIndex].Kp*M[motorIndex].error + M[motorIndex].Ki*M[motorIndex].integralError;
	calcPwm = calcPwm  + dc_offset;
	
	if ((calcPwm <= MAX_PWM) && (calcPwm > dc_offset))
	{
		M[motorIndex].pwm = calcPwm;
	}
	if((calcPwm <= dc_offset) && (calcPwm > 0 ))
	{
		M[motorIndex].pwm = dc_offset;
	}
	
	
}

void StepMotorPID_w_DCOffset(uint8_t motorIndex,int offset)
{
	unsigned int calcPwm = 0;

	M[motorIndex].presentRpm = GetRPM(&encs,motorIndex);
	M[motorIndex].intTarget = M[motorIndex].setRpm;

	M[motorIndex].error = M[motorIndex].intTarget - M[motorIndex].presentRpm;

	//back roller correction
	/*if ((motorIndex == 4) && (M[motorIndex].presentRpm < 100) && (startFlag == 1)){
		if (M[motorIndex].error < 0){
			M[motorIndex].error = 0;
		}
	}*/
	
	if (fabs(M[motorIndex].error) >= 300)
	{
		M[motorIndex].overloadCount++;
		if(M[motorIndex].overloadCount >= 30) //changed from 10 to30 in philly
		{
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
	
	M[motorIndex].integralError = (M[motorIndex].integralError + M[motorIndex].error);
	
	calcPwm = M[motorIndex].Kp*M[motorIndex].error + M[motorIndex].Ki*M[motorIndex].integralError;
	calcPwm = calcPwm + offset;

	if ((calcPwm <= MAX_PWM) && (calcPwm > offset))
	{
		M[motorIndex].pwm = calcPwm;
	}
	if(calcPwm <= offset)
	{
		M[motorIndex].pwm = offset;
	}
}


void UpdateMotorPID(uint8_t motorIndex)
{
	unsigned int calcPwm = 0;
	M[motorIndex].presentRpm = GetRPM(&encs,motorIndex);
	
	if((startFlag == 1) && ((M[motorIndex].intTarget <= M[motorIndex].setRpm)))
	{
		M[motorIndex].intTarget = M[motorIndex].intTarget + 25;
	}
	
	if((startFlag == 1) && ((M[motorIndex].intTarget > M[motorIndex].setRpm)))
	{
		M[motorIndex].intTarget = M[motorIndex].intTarget - 25;
	}
	
	if((startFlag == 0)&&((M[motorIndex].intTarget > 0)))
	{
		M[motorIndex].intTarget = M[motorIndex].intTarget - 25;
	}
	
	M[motorIndex].error = M[motorIndex].intTarget - M[motorIndex].presentRpm;
	
	/*if (__fabs(M[motorIndex].error) >= 300)
	{
		E.RpmErrorFlag = 1;
		S.errStopReason = MOTORARRAY[motorIndex];
		S.errStopReasonHMI = MOTORARRAY_HMI[motorIndex];
		S.errmotorFault = ERR_RPM_ERROR;	
		S.errVal = NO_FLOAT_VAR;	
	}*/
	
	
	M[motorIndex].integralError = (M[motorIndex].integralError + M[motorIndex].error);
	
	calcPwm = M[motorIndex].Kp*M[motorIndex].error + M[motorIndex].Ki*M[motorIndex].integralError;
	
	if (calcPwm <= MAX_PWM)
	{
		M[motorIndex].pwm = calcPwm;
	}
}

char CheckAllMotors(void)
{
	char motorOnSignal = 0;
	if (M[MOTOR1].presentRpm >= (M[MOTOR1].setRpm - 100))
	{
		motorOnSignal = 1;
	}
	return motorOnSignal;
}

void MotorTimer(char index)
{
		switch(index)
	{
		case MOTOR1_TIMER_ON:
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
		break;
		
		case MOTOR1_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4);
		break;
		
		case MOTOR2_TIMER_ON:
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
		break;
		
		case MOTOR2_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
		break;
		
		case MOTOR3_TIMER_ON:
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		break;
		
		case MOTOR3_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
		break;
		
		case MOTOR4_TIMER_ON:
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
		break;
		
		case MOTOR4_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
		break;
		
		case MOTOR5_TIMER_ON:
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
		break;
		
		case MOTOR5_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
		break;
		
		case MOTOR6_TIMER_ON:
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
		break;
		
		case MOTOR6_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
		break;
		
		case MOTOR7_TIMER_ON:
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
		break;
		
		case MOTOR7_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);
		break;
	}
}

void PauseMachine(void)
{
	uint8_t i = 0;
	HAL_Delay(100);
	for(i=1;i<6;i++)
		{
			M[i].pwm = 0;
		}
	TowerLamp(GREEN_OFF);
}

void ResetMotorVariables(void)
{				//WE dont want to reset the Kp,Ki startOFfset 
	
				M[MOTOR1].pwm = 0	;
				M[MOTOR2].pwm = 0;
				M[MOTOR3].pwm = 0;
				M[MOTOR4].pwm = 0;
				M[MOTOR5].pwm = 0;
				
				M[MOTOR1].calcPwm = 0	;
				M[MOTOR2].calcPwm = 0;
				M[MOTOR3].calcPwm = 0;
				M[MOTOR4].calcPwm = 0;
				M[MOTOR5].calcPwm = 0;

				M[MOTOR1].intTarget = 0;
				M[MOTOR2].intTarget = 0;
				M[MOTOR3].intTarget = 0;
				M[MOTOR4].intTarget = 0;
				M[MOTOR5].intTarget = 0;
			
				M[MOTOR1].error = 0;
				M[MOTOR2].error = 0;
				M[MOTOR3].error = 0;
				M[MOTOR4].error = 0;
				M[MOTOR5].error = 0;

				M[MOTOR1].integralError = 0;
				M[MOTOR2].integralError = 0;
				M[MOTOR3].integralError = 0;
				M[MOTOR4].integralError = 0;
				M[MOTOR5].integralError = 0;
				
				M[MOTOR1].antiWindup = 0;
				M[MOTOR2].antiWindup = 0;
				M[MOTOR3].antiWindup = 0;
				M[MOTOR4].antiWindup = 0;
				M[MOTOR5].antiWindup = 0;

}

void calculateRpm(void)
{
		float currentBobbinDia = 0;
		float deliverySpeed = 0;
	//	float rampDeliverySpeed = 0;
		M[MOTOR2].setRpm = ((filter1*36/38)*(1+8.534242f/(fsp.tpi*fsp.bareBobbinDia+currentLayer*fsp.deltaBobbinDia*fsp.rtf)));
	//	M[MOTOR2].rampRpm = rampRpm*(1+8.534242/(P.Tpi*P.BareBobbinDia+currentLayer*P.DeltaBobbin*P.Rtf));
		currentBobbinDia = fsp.bareBobbinDia+currentLayer*fsp.deltaBobbinDia;
		deliverySpeed = (filter1*36/38)/(fsp.tpi*39.3700787f);
	//	rampDeliverySpeed = rampRpm/(P.Tpi*39.3700787f);
		M[MOTOR3].setRpm = ((filter1*36/38)/(fsp.tpi*3.711f)*15.5f);
		M[MOTOR4].setRpm = M[MOTOR3].setRpm/fsp.tensionDraft/15.5f*24.0f;
		M[MOTOR5].setRpm = ((fsp.rovingWidth*deliverySpeed)/(3.141593f*currentBobbinDia))*6000;
//		M[MOTOR5].rampRpm = ((P.RovingWidth*rampDeliverySpeed)/(3.141593*currentBobbinDia))*6000;

}



unsigned int flyerMotorRPM = 0;
unsigned int bobbinMotorRPM = 0;
unsigned int frontRollerMotorRPM = 0;
unsigned int backRollerMotorRPM = 0;
float flyerRPM = 0;  //for calc
float bobbinRPM = 0;  //for calc
float frontRollerRPM = 0;
float deliverySpeed = 0;
float  flyerGearRatio = 0.923076; //36/39;
float multiplier = 1.0;

void calculateRpm2(void)
{
	/* Let spindlespeed, aka flyer speed = 1000, barebobbin Dia= 48, delta bobbin dia = 1.8 and tpi = 1.3
	Machines logic is -> delivery speed from front roller = winding speed.
    delivery speed is linked to flyer speed as such :
	delivery speed = flyer speed (twists/min) / tpi (twists/inch) ) *mtr to inches conversion)
	delivery speed = 1000 * (39.37)/1.3 = 769 inches/min = 19.54 mtrs/min

	winding speed is same as delivery speed
	so winding speed = 19.55 mtrs/min
	winding speed is also = delta rpm btw bobbin and flyer -> * circumfernce of bobbin ( pi * D)
	= delta N * pi * bareBobbinDia/1000
	19.54 = deltaN * pi * 48/1000
	deltaN = 130 rpm
	so bobbin rpm = flyer rpm + deltaN = 1000 + 130 = 1130
	
	generalised formula for bobbin RPM therfore = spindleRPM+((130* (spindleRPM/1000) * (1.3/tpi) * (48/(bareBobbinDia + deltaBobbinDia * Layers))) * rtf)
	
	similarly,for front roller rpm = > circumference of frontroller * rpm = delivery speed
	19.54 = pi*(30/1000) * front roller rpm
	front roller rpm = 207 * (flyer speed/1000) * (1.3/tpi)
	these nos are without gear ratios
	*/
	
	int currentBobbinDia = fsp.bareBobbinDia + fsp.deltaBobbinDia * currentLayer;
	
	flyerMotorRPM = M[MOTOR1].intTarget;
	flyerRPM =flyerMotorRPM * flyerGearRatio;  //will be 36/39
	deliverySpeed = flyerRPM /(fsp.tpi * 39.37f);  //
	
	bobbinRPM = flyerRPM +(130.0f * (flyerRPM/1000.0f) * (1.3f/fsp.tpi) * (48.0f/currentBobbinDia) * (S.runningRTF * S.RTFmultiplier)) ;

	bobbinMotorRPM = ceil(bobbinRPM / flyerGearRatio);
		
	frontRollerRPM = 207 * (flyerRPM/1000.0f) * (1.3f/fsp.tpi)  ;  // 0.265 instead of 0.269

	frontRollerMotorRPM = ceil(frontRollerRPM * 15.5f) ;
	backRollerMotorRPM = ceil((frontRollerRPM/fsp.tensionDraft) * 24.0f);

	M[MOTOR1].setRpm = fsp.spindleSpeed / flyerGearRatio;
	M[MOTOR2].setRpm = bobbinMotorRPM * multiplier;
	// Lift motors
	M[MOTOR5].intTarget = ceil((((fsp.rovingWidth*deliverySpeed)/(3.141593*currentBobbinDia))*6000)*1.041667);
	M[MOTOR2].intTarget = M[MOTOR2].setRpm;
	M[MOTOR3].intTarget =  frontRollerMotorRPM ;
	M[MOTOR4].intTarget =  backRollerMotorRPM;

}
	
char MaxLayerComplete(void)
{
	char out = 0;
	//char errorMotor = 0;
	if(currentLayer >= fsp.lengthLimit)
	{
		out = 1;
	}

	return out;
}

char LengthLimitReached(void)
{
	char out = 0;
	//char errorMotor = 0;
	if(currentLength >= fsp.lengthLimit)
	{
		out = 1;
	}

	return out;
}

// check max limit
char LiftLimitCheck(void)
{
	char out = 0;
	char sensorStatus1 = 0;
	sensorStatus1= InputSensor1();
//	sensorStatus2 = InputSensor2();
//	if((sensorStatus1 == 1) || (sensorStatus2 ==1))
	if(sensorStatus1 == 1)
	{
		out = 1;
	}
	return out;
}


//-------------------------------------//



void TimerLow(char index)
{
	switch(index)
	{
		case T2_CH4:
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4,0);
		break;
		
		case T2_CH3:
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3,0);
		break;
		
		case T1_CH4:
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,0);
		break;
		
		case T1_CH3:
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,0);
		break;
		
		case T1_CH2:
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,0);
		break;
		
		case T1_CH1:
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,0);
		break;
		
		case T3_CH4:
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4,0);
		break;
	}
}

void AllSignalVoltageLow(void)
{
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4,0);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4,0);
	
}



void ApplyPwms(void){
 if ((M[MOTOR1].pwm <= MAX_PWM ) && (M[MOTOR1].pwm >= MIN_PWM))
	{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4,M[MOTOR1].pwm);	
	}
	
	if ((M[MOTOR2].pwm <= MAX_PWM ) && (M[MOTOR2].pwm >= MIN_PWM))
	{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3,M[MOTOR2].pwm);
	}
	
	if ((M[MOTOR3].pwm <= MAX_PWM ) && (M[MOTOR3].pwm >= MIN_PWM)){
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,M[MOTOR3].pwm);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,M[MOTOR3].pwm); // MOT6 connector
	}
	
	if ((M[MOTOR4].pwm <= MAX_PWM ) && (M[MOTOR4].pwm >= MIN_PWM))
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,M[MOTOR4].pwm);
	}
	
	if ((M[MOTOR5].pwm <= MAX_PWM ) && (M[MOTOR5].pwm >= MIN_PWM))
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,M[MOTOR5].pwm);
	}
	
}



void ResetStartOffsetVars(void){
	for (int i=1;i<6; i++){
		M[i].startOffset = M[i].startOffsetOrig;
	}
}

void CalculateRTF_MultiplierValues(void){
		//FFs.rampUpRTFmultiplier and rampDownRTF_multiplier are controls on the RTF specifically when ramping up
		//or ramping down. We input a STARTING MULTIPLIER btw 0.5 and 1.5 for each section in the app, and the RTF value
		//that is used during that section equals rtf*Multiplier where the multiplier itself reaches 1.0 
		//at 80% of the total ramp up or ramp down time.
		
		//RAMPUP
		//We want to know by how much to increment the rtfMultiplier so that it reaches 1 by the time
		//ramp up is complete.
		int noLoops = (M[MOTOR1].setRpm/FFs.rampUpRate) * FFs.settlingTimeConstant ;
		float deltaRTFNeeded = 1.0f  - FFs.rampUpRtfMultiplier; // can be both positive or negative
		FFs.deltaRTF_rampUp = deltaRTFNeeded/noLoops; // can also be positive or negative.
		FFs.currentRTFMultiplier_rampUp = FFs.rampUpRtfMultiplier; // keep the original variable so that we can reset it when needed
	
}

// to be called only just before you want to do the RTF multiplier work, ie: when you have a trigger and are preparing to go into pause,
// or youve triggered and are just going to run.
void ResetRampUp_RTFMultiplier(void){
	FFs.currentRTFMultiplier_rampUp = FFs.rampUpRtfMultiplier;
  FFs.runningRTFMultiplier_bool = 1;
}

