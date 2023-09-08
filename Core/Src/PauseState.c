#include "StateFns.h"
#include "Structs.h"
#include "CommonConstants.h"
#include "HMI_Constants.h"
#include "HMI_Fns.h"
#include "stm32f4xx_hal.h"
#include "logicDefines.h"
#include "functionDefines.h"
#include "encoder.h"

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim7;
extern uint8_t BufferRec[];
extern char BufferTransmit[];
extern int keyPress;
extern int allMotorsOn;
extern int startFlag;
extern int sliverCut;

void PauseState(void)
{ 
	char sizeofPacket = 0;
	while(1){
			// and start the timer that signals when to send the data.
			if (HAL_TIM_Base_GetState(&htim7) ==  HAL_TIM_STATE_READY){
					HAL_TIM_Base_Start_IT(&htim7); // currently at 1 sec
				}

			if (S.updateBasePackets == 1){
					// keep it here because this will run once and we want this buffer packet to get created only once.
					// also prepare the Run packet Structure
					UpdateBasePacket_Modes(CURRENT_MACHINE,HMI_SCREEN_DATA,HMI_STOP_SCREEN,HMI_STOP_PAUSE,HMI_STOP_PACKET_LENGTH,3);
					UpdateStopPacket(HMI_STOP_REASON_CODE,NO_VAR,NO_VAR);
				  S.updateBasePackets = 0;
			}
						
			//send Pause packet
			if ((U.TXcomplete ==1) && (U.TXtransfer == 1)){
				sizeofPacket = UpdateStopPacketString(BufferTransmit,hsb,hsp,S.errStopReasonHMI,S.errmotorFault,S.errVal);
				 HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
				 U.TXcomplete = 0;
				 U.TXtransfer = 0;
			}
						
						
			/******************MOTOR LOGIC*******************/
			// NO LOGIC MOTOR WILL HALT IN INTERRUPT LOOP ONLY
			TowerLamp(GREEN_OFF);
			TowerLamp(AMBER_ON);
			TowerLamp(RED_OFF);
			/*************************************************/

												
			//Check for button pres to go back to run state
			keyPress = Pushbutton();
			if ((keyPress==1) && (S.keyState == ENABLE)){
				S.state_change = TO_RUN;
				S.current_state =  RUN_STATE;
				S.prev_state = PAUSE_STATE;
				S.first_enter = 1;  //update the base packets
				S.oneTimeflag = 0;
				S.updateBasePackets = 1;
				S.errStopReason = NO_VAR;
				S.errStopReasonHMI = NO_VAR;
				S.errmotorFault = NO_VAR;
				S.errVal = NO_VAR;
				HAL_TIM_Base_Stop_IT(&htim7);

				TimerLow(T1_CH4);
				ResetMotorVariables();
				ResetEncoderVariables(&encs);
				ResetStartOffsetVars(); // might  need to reset offset Voltage..

				TowerLamp(GREEN_ON);
				TowerLamp(AMBER_OFF);
				TowerLamp(RED_OFF);

				startFlag = 1;
				allMotorsOn = 1;
				sliverCut = 0;

				LedOff(LED1);

				M[MOTOR1].rampRpm = FFs.rampUpRate; //rampUp
				//Reset the RTF multiplier so that the ramp up works properly
				ResetRampUp_RTFMultiplier();
				keyPress = 0;
				HAL_Delay(500);
				break;
			}

			//allow changing settings in pause state also. sueful while tuning and changing PID without homing each time.
			if (S.state_change == TO_SETTINGS){
				S.current_state =  UPDATESETTINGS;
				S.prev_state = PAUSE_STATE;
				S.first_enter = 1;
				S.oneTimeflag = 0;
				S.updateBasePackets = 1;
				HAL_TIM_Base_Stop_IT(&htim7);
				break;
			}

			//Check for RPM ERROR, to go into halt State
			if (E.RpmErrorFlag == 1){
				S.state_change = TO_HALT;
				S.current_state = HALT_STATE;
				S.prev_state = RUN_STATE;
				S.updateBasePackets = 1;
				S.first_enter = 1;
				S.oneTimeflag = 0;
				HAL_TIM_Base_Stop_IT(&htim7);
				break;
			}

	}// closes while
}

