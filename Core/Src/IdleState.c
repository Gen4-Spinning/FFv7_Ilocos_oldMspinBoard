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
extern uint8_t BufferRec[];
extern char BufferTransmit[];
extern int keyPress;
int hommingFlag = 1; //flag to tell idle to go to homing onfirst key press !!!
int machineRun = 0; //flag to tell idle to go to run state

void IdleState(void)
{ 
	int sizeofPacket = 0;
	
	while(1)
	{							
			// start the timer that signals when to send the data.
			if (HAL_TIM_Base_GetState(&htim7) ==  HAL_TIM_STATE_READY){
				HAL_TIM_Base_Start_IT(&htim7); // currently at 1 sec
			}
										
		// every sec, send a Idle screen(if you send it once, if it fails your gone)
			if( (U.TXtransfer == 1)	&& (U.TXcomplete == 1)){
				UpdateBasePacket_Modes(CURRENT_MACHINE,HMI_SCREEN_DATA,HMI_IDLE_SCREEN,NO_VAR,NO_VAR,NO_VAR);
				sizeofPacket = HMI_GetIdlePacketString(BufferTransmit,hsb);
				HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
				U.TXcomplete = 0;
				U.TXtransfer = 0;
				}
				
			/*********CONTROL LOGIC************/
			TowerLamp(GREEN_OFF);
			TowerLamp(AMBER_ON);
			TowerLamp(RED_OFF);

			//HOMING STATE
			keyPress = Pushbutton();
			
			//Start Homing - while loop wont update IDLE screen.
			if((keyPress == 1)&& (hommingFlag ==1))
			{
				S.updateBasePackets = 1 ;
				S.state_change = TO_HOMING;
				S.current_state =  HOMING_STATE;
				S.prev_state = IDLE_STATE;
				S.first_enter = 1;
				S.oneTimeflag = 0;
				HAL_TIM_Base_Stop_IT(&htim7);
				break;
			}

			if((keyPress == 1)&& (machineRun ==1)){
				S.state_change = TO_RUN;
				S.current_state =  RUN_STATE;
				S.prev_state = IDLE_STATE;
				S.first_enter = 1;
				S.oneTimeflag = 0;												
				S.updateBasePackets = 1;
				//update the RTF here
				S.displayRTF = fsp.rtf * 1000;
				S.runningRTF = fsp.rtf;
				keyPress = 0;						
				ResetRampUp_RTFMultiplier();					
				HAL_TIM_Base_Stop_IT(&htim7);
				HAL_Delay(500);	
				break;
			}

			/*******************************************/
							
			if (S.state_change == TO_DIAG){
				S.current_state =  DIAGNOSTIC_STATE;
				S.prev_state = IDLE_STATE;
				S.first_enter = 1;
				S.oneTimeflag = 0;
				HAL_TIM_Base_Stop_IT(&htim7);
				break;
			}
			
			if (S.state_change == TO_SETTINGS){
				S.current_state =  UPDATESETTINGS;
				S.prev_state = IDLE_STATE;
				S.first_enter = 1;
				S.oneTimeflag = 0;
				S.updateBasePackets = 1;
				HAL_TIM_Base_Stop_IT(&htim7);
				break;
			}
			
	} // closes while
}

