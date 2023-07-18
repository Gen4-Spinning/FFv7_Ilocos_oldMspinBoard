#include "Structs.h"
#include "CommonConstants.h"
#include "initialize.h"
#include "logicDefines.h"
#include "HMI_Constants.h"
#include "CommonConstants.h"
#include <string.h>
//Load the structs here and make them extern in the file you want.

extern int currentLayer;
extern int rampRpm;

void InitializeFFAdvSettings(void)
{
    FFs.rampDownRate = 0;
    FFs.rampDownRtfMultiplier = 0;
    FFs.rampUpRate = 0;
    FFs.rampUpRtfMultiplier = 0;
		FFs.currentRTFMultiplier_rampUp = 0;
		FFs.deltaRTF_rampUp = 0;
		FFs.settlingTimeConstant = 0.80; // during ramp down, by the time intTarget comes to 20% of maxRpm rtfMultipier  will become 1.0
		FFs.runningRTFMultiplier_bool = 0;
		FFs.logRTFMultiplier = 0;
}
void InitializePIDUpdateStruct(){
 P.requestedPIDoptionIdx =0;
 P.updateOptionID =0;
 P.updateAttr1 =0;
 P.updateAttr2 =0;
 P.updateAttr3 =0;
 P.updateAttr4 =0;
 P.updatePIDVals =0;
}
void InitializeStateStruct()
{
	S.current_state = IDLE_STATE;
	S.prev_state = OFF_STATE;
	S.state_change = 0;
	S.first_enter = 1;
	S.oneTimeflag = 0;
	S.errStopReason = NO_VAR;
	S.errmotorFault = NO_VAR;
	S.errStopReasonHMI = NO_VAR;
	S.errVal = NO_VAR;
	S.firstSwitchon = 1;
	S.runMode = HMI_RUN_RAMPUP; 
	S.updateBasePackets = 0;
	S.oneSecTimer = 0;
	S.displayRTF = 0;
	S.runningRTF = 0;
	S.RTFmultiplier = INIT_RTF_MULTIPLIER; 
	S.keyState = ENABLE_D;
	S.loggingEnabled = 0;
	S.logType = 0;
}

void InitializeFlyerSettings()
{
    fsp.spindleSpeed = 1200;
    fsp.tensionDraft = 5.8;
    fsp.tpi = 1.1;
    fsp.rtf = 0.8;
    fsp.lengthLimit = 60 ;
    fsp.bobbinHeight = 345;
    fsp.rovingWidth = 2.5;
    fsp.deltaBobbinDia = 1.5;
    fsp.bareBobbinDia = 52;
    fsp.boostFactor = 50;
    fsp.buckFactor = 45;
    
}

void InitializeUartStruct()
{
	U.TXcomplete =0;
	U.TXtransfer =0;
}
void InitializeDiagnosticsStruct()
{
	D.motorID = 0;
	D.motorID = 0;
	D.targetSignal=0;
	D.testTime = 0;
	D.typeofTest = 0;
	D.isSubAssembly = 0;
	D.subAssembly = 0;
}


//Can only run Once at the beginning.Cant run elseWHERE! cos the KP Ki kd 
//start offset values from the eeprom will get overwritten
void MotorStructInit(void)	
{
		strcpy(M[MOTOR1].name,"FLYER") ;
		M[MOTOR1].presentRpm = 0;
		M[MOTOR1].setRpm = fsp.spindleSpeed *38/36;
		M[MOTOR1].rampRpm = FFs.rampUpRate; //was 10
		M[MOTOR1].piecingRpm = 300;
		M[MOTOR1].error = 0;
		M[MOTOR1].integralError = 0;
		M[MOTOR1].feedforward = 0;
		M[MOTOR1].ff_multiplier = 0.4f;//0
		M[MOTOR1].intTarget = 0;
		M[MOTOR1].calcPwm = 0;
		M[MOTOR1].pwm = 0;
		M[MOTOR1].Ki = 0.1;//was 0.2. changed from 0.6 to 0.4 in Philly 1 FF 1
		M[MOTOR1].Kp = 0.05;//was 0.1
		M[MOTOR1].startOffset = 360;
		M[MOTOR1].steadyState = 0;
 	    M[MOTOR1].overloadDelta = 300; // delta rpm of 300
		M[MOTOR1].overloadTime = 50; // for 5 sec at 0.1 sec interrupt
		M[MOTOR1].overloadCount = 0;
		M[MOTOR1].antiWindup = 0;
		
        strcpy(M[MOTOR2].name,"BOBBN");
		M[MOTOR2].presentRpm = 0;
		M[MOTOR2].setRpm = 0;
		M[MOTOR2].rampRpm = rampRpm*(1+(float)8.534242/(fsp.tpi*fsp.bareBobbinDia+currentLayer*fsp.deltaBobbinDia*fsp.rtf));
   	    M[MOTOR2].piecingRpm = 300;
		M[MOTOR2].error = 0;
		M[MOTOR2].integralError = 0;
		M[MOTOR2].feedforward = 0;
		M[MOTOR2].ff_multiplier = 0.5f;
		M[MOTOR2].intTarget = 0;
		M[MOTOR2].calcPwm = 0;
		M[MOTOR2].pwm = 0;
		M[MOTOR2].Ki = 0.1;//was 0.3 (changed from 1.2 @ Philly-1) Cahnged from 0.6 @ philly1 FF1 (Feb19)
		M[MOTOR2].Kp = 0.05; //was 0.2 (changed from 0.4 @ Philly-1)
		M[MOTOR2].startOffset = 260;
		M[MOTOR2].steadyState = 0;
	    M[MOTOR2].overloadDelta = 300;
		M[MOTOR2].overloadTime = 50;
		M[MOTOR2].overloadCount = 0;
		M[MOTOR2].antiWindup = 0;

		strcpy(M[MOTOR3].name,"FRRLR");
		M[MOTOR3].presentRpm = 0;
		M[MOTOR3].setRpm = 0;
		M[MOTOR3].rampRpm = rampRpm/(fsp.tpi*3.711f)*15.5f;
		M[MOTOR3].piecingRpm = 300;
		M[MOTOR3].error = 0;
		M[MOTOR3].integralError = 0;
		M[MOTOR3].feedforward = 0;
		M[MOTOR3].ff_multiplier = 0.4f;//0
		M[MOTOR3].intTarget = 0;
		M[MOTOR3].calcPwm = 0;
		M[MOTOR3].pwm = 0;
		M[MOTOR3].Ki = 0.3; //was 0.4. Changed from 0.6 to 0.4 in philly1 for FF2 And 0.3 for FF1
		M[MOTOR3].Kp = 0.068;
		M[MOTOR3].startOffset = 160;
		M[MOTOR3].steadyState = 0;
	    M[MOTOR3].overloadDelta = 200;
		M[MOTOR3].overloadTime = 100;
		M[MOTOR3].overloadCount = 0;
		M[MOTOR3].antiWindup = 0;
		
		strcpy(M[MOTOR4].name,"BKRLR");
		M[MOTOR4].presentRpm = 0;
		M[MOTOR4].setRpm = 0;
		M[MOTOR4].rampRpm = 20;
		M[MOTOR4].piecingRpm = 300;
		M[MOTOR4].error = 0;
		M[MOTOR4].integralError = 0;
		M[MOTOR4].feedforward = 0;
		M[MOTOR4].ff_multiplier = 0.8f;//0
		M[MOTOR4].intTarget = 0;
		M[MOTOR4].pwm = 0;
		M[MOTOR4].calcPwm = 0;
		M[MOTOR4].Ki = 0.4;
		M[MOTOR4].Kp = 0.068;
		M[MOTOR4].startOffset = 145;
		M[MOTOR4].steadyState = 0;
	    M[MOTOR4].overloadDelta = 200;
		M[MOTOR4].overloadTime = 100;
		M[MOTOR4].overloadCount = 0;
		M[MOTOR4].antiWindup = 0;
		
		strcpy(M[MOTOR5].name,"LFT_L");
		M[MOTOR5].presentRpm = 0;
		M[MOTOR5].setRpm = 1000;
		M[MOTOR5].rampRpm = 25;
		M[MOTOR5].piecingRpm = 300;
		M[MOTOR5].error = 0;
		M[MOTOR5].integralError = 0;
		M[MOTOR5].feedforward = 0;
		M[MOTOR5].ff_multiplier = 0.8f;//0
		M[MOTOR5].intTarget = 0;
		M[MOTOR5].calcPwm = 0;
		M[MOTOR5].pwm = 0;
		M[MOTOR5].Ki = 0.2;
		M[MOTOR5].Kp = 0.4;
		M[MOTOR5].startOffset = 100;
		M[MOTOR5].steadyState = 0;
	 	M[MOTOR5].overloadDelta = 200;
		M[MOTOR5].overloadTime = 100;
		M[MOTOR5].overloadCount = 0;
		M[MOTOR5].antiWindup = 0;

}
