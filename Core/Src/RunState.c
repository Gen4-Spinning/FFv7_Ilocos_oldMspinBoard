#include "StateFns.h"
#include "Structs.h"
#include "CommonConstants.h"
#include "HMI_Constants.h"
#include "HMI_Fns.h"
#include "stm32f4xx_hal.h"
#include "functionDefines.h"
#include "logicDefines.h"
#include "encoder.h"


extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim11;
extern uint8_t BufferRec[];
extern char BufferTransmit[];
extern char MOTORARRAY[];
extern char MOTORARRAY_HMI[];

//RUN LOGIC variables
extern int allMotorsOn;
extern int startFlag;
extern int leftpulseCount;
extern int currentLayer;
extern int rightpulseCount;
extern float currentLength;
char maxLayersOver = 0;
char liftlimitChk =0;
extern int sliverCut;
extern int keyPress;
// vars to calculate rate at which boost rtf multiplier needs to change..
extern float noLoops ;
extern float deltaRTFNeeded ;

extern long targetPulseCount;
void RunState(void)
{ 
	char sizeofPacket = 0;
	while(1){
		/******************COMMUNICATION**************/
		if (S.updateBasePackets == 1){	// prepare the Run packet Structure for the HMI
			UpdateBasePacket_Modes(CURRENT_MACHINE,HMI_SCREEN_DATA,HMI_RUN_SCREEN,HMI_RUN_NORMAL,HMI_RUN_PACKET_LENGTH_FF,3);
			UpdateRunPacket_FF(HMI_FF_LAYERS_RUN,currentLayer,HMI_FF_RTF,S.runningRTF,HMI_FF_PRODUCTION_RUN,0);
			S.updateBasePackets = 0;
		}

		// start the timer that signals when to send the data.
		if (HAL_TIM_Base_GetState(&htim7) ==  HAL_TIM_STATE_READY){
				HAL_TIM_Base_Start_IT(&htim7); // currently at 1 sec
			}
					
		//Timer that is looking at the FF sliver cut sensor
		if (HAL_TIM_Base_GetState(&htim11) ==  HAL_TIM_STATE_READY){
			HAL_TIM_Base_Start_IT(&htim11); // currently at 0.01 sec
		}
														
							
	//send Run packet
		if ((U.TXcomplete ==1) && (U.TXtransfer == 1)){
			 sizeofPacket = UpdateRunPacketString_FF(BufferTransmit,hsb,hrp,currentLayer,S.displayRTF,currentLength);
			 HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
			 U.TXcomplete = 0;
			 U.TXtransfer = 0;
		}
				
		//Not to be run after PAUSE
		if (S.firstSwitchon == 1){
		// Switch on the FF Motors

			ResetMotorVariables();
			ResetEncoderVariables(&encs);
			ResetStartOffsetVars();
			MotorDrive(ENABLE_D); //FF and BB motors On also.

			//Lights
			TowerLamp(GREEN_ON);
			TowerLamp(AMBER_OFF);
			TowerLamp(RED_OFF);

			// logic variables
			allMotorsOn = 1;
			startFlag = 1;

			leftpulseCount = 0;
			rightpulseCount = 0;
			currentLayer = 0;

			S.firstSwitchon = 0;
			HAL_Delay(500);
		}

		//Check for User KeyPress to take machine into Pause State
		keyPress = Pushbutton();
		if (keyPress){
			S.state_change = TO_PAUSE;
			S.current_state =  PAUSE_STATE;
			S.prev_state = RUN_STATE;
			S.first_enter = 1;
			S.oneTimeflag = 0;
			S.updateBasePackets = 1;
			S.errStopReason = ERR_USER_PAUSE;
			S.errStopReasonHMI = ERR_USER_PAUSE;
			S.errmotorFault = NO_VAR;
			S.errVal = NO_FLOAT_VAR;
			S.keyState = DISABLE;
			HAL_TIM_Base_Stop_IT(&htim7);
			HAL_TIM_Base_Stop_IT(&htim11);
			M[MOTOR1].rampRpm = FFs.rampDownRate; // ramp down RPM
			startFlag = 0; // will make the motor RAMP DOWN
			keyPress = 0;

			HAL_Delay(200);
			break;
		}

			
		/* if all layers over */
		maxLayersOver = LengthLimitReached();
		if (maxLayersOver == 1){
			S.state_change = TO_ENDSTATE;
			S.current_state =  END_STATE;
			S.prev_state = RUN_STATE;
			S.errStopReason = ERR_LAYERS_COMPLETE;
			S.errStopReasonHMI = ERR_LAYERS_COMPLETE;
			S.errmotorFault = NO_VAR; // TO define in both
			S.errVal = NO_FLOAT_VAR;	// for hmi the val is actually an int.so is overwritten inside the get string HMI error Packet fn
			S.updateBasePackets = 1;
			S.keyState = DISABLE; // this will get set when all the motors stop
			S.updateBasePackets = 1;
			HAL_TIM_Base_Stop_IT(&htim7);
			HAL_TIM_Base_Stop_IT(&htim11);
			M[MOTOR1].rampRpm = FFs.rampDownRate; // ramp down RPM;
			startFlag = 0; // will make the motor RAMP DOWN
			break;
		}

				
		/**************************************************
		sliverCut = 0;
		if (sliverCut)
			{
				S.state_change = TO_PAUSE;
				S.current_state =  PAUSE_STATE;
				S.prev_state = RUN_STATE;
				S.first_enter = 1;
				S.oneTimeflag = 0;
				S.errStopReason = ERR_SLIVER_CUT_ERROR;
				S.errStopReasonHMI = ERR_SLIVER_CUT_ERROR;
				S.errmotorFault = NO_VAR;
				S.errVal = NO_VAR;
				HAL_TIM_Base_Stop_IT(&htim7);
				HAL_TIM_Base_Stop_IT(&htim11);
				S.updateBasePackets = 1;
				startFlag = 0; // will make the motor RAMP DOWN
				M[MOTOR1].rampRpm = FFs.rampDownRate; // ramp down RPM;
				TowerLamp(GREEN_ON);
				TowerLamp(AMBER_OFF);
				HAL_Delay(200);
				break;
		}*/

		//Check for RPM ERROR, to go into halt State
		if (E.RpmErrorFlag == 1)
		{ S.state_change = TO_HALT;
			S.current_state = HALT_STATE;
			S.prev_state = RUN_STATE;
			S.first_enter = 1;
			S.oneTimeflag = 0;
			S.updateBasePackets = 1 ;
			HAL_TIM_Base_Stop_IT(&htim7);
			HAL_TIM_Base_Stop_IT(&htim11);
			S.updateBasePackets = 1;
			break;
		}
	} // closes while
}



