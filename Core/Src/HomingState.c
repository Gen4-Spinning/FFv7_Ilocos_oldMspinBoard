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

extern int homeFlag ; // flag for interrupt
extern int hommingFlag; // flag in idle tell to indicate that we need to go to homing when we press the button
extern int machineRun;
extern int leftpulseCount;
extern int rightpulseCount;

char homing_Calib_State = DOWN;
char homing_Normal_State = OFF;
char btm_bed_reached = 0;
char topLimitSwitch = 0;
char btmLimitSwitch = 0;
int leftMotorOn = 0;
int rightMotorOn = 0;
extern int errorCount;
extern int startFlag;

void HomingState(void)
{ 
	int sizeofPacket = 0;
	int8_t towerLamp = 1;
	while(1)
	{
		if (S.first_enter ==1){
			// create the run packet for the HMI
			S.displayRTF = 1.0f;
			homing_Calib_State = DOWN;
			homing_Normal_State = DOWN;  //changed from off to down
			homeFlag = 1;
			startFlag =1;
			MotorDrive(ENABLE_D);   //ONLY FOR FF AND BB All drive Enable pin high
			MotorDrive(REVERSE_D);  // Direction change //changing from forward to reverse
			leftMotorOn = 1;
			S.first_enter = 0;
		}
			
		if (S.updateBasePackets == 1){
			UpdateBasePacket_Modes(CURRENT_MACHINE,HMI_SCREEN_DATA,HMI_RUN_SCREEN,HMI_RUN_HOMING,HMI_RUN_PACKET_LENGTH_FF,3);
			UpdateRunPacket_FF(HMI_FF_LAYERS_RUN,0,HMI_FF_RTF,S.displayRTF,HMI_FF_PRODUCTION_RUN,0);
			S.updateBasePackets  = 0;
		}
			
		// and start the timer that signals when to send the data.
		if (HAL_TIM_Base_GetState(&htim7) ==  HAL_TIM_STATE_READY){
				HAL_TIM_Base_Start_IT(&htim7); // currently at 1 sec
			}
						

			// every sec, send a Homing screen(if you send it once, if it fails your gone)
		if((U.TXtransfer == 1)	&& (U.TXcomplete == 1)){
			sizeofPacket = UpdateRunPacketString_FF(BufferTransmit,hsb,hrp,0,S.displayRTF,0.1f);
			HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
			U.TXcomplete = 0;
			U.TXtransfer = 0;

			towerLamp *= -1;
			if (towerLamp == -1){
				TowerLamp(AMBER_OFF); // actually ON
			}else{
				TowerLamp(AMBER_ON);
			}
		}

		if (homing_Normal_State == DOWN){
			homeFlag = 1;
			startFlag =1;
			btm_bed_reached = HomingSensor();
			leftpulseCount = 0;
			if (btm_bed_reached == 1){
				homing_Normal_State = OFF;
				homeFlag = 0;
				M[MOTOR5].setRpm = 0;
				TimerLow(T1_CH2);
				TimerLow(T1_CH1);
				TimerLow(T3_CH4);
				ResetEncoderVariables(&encs);
				ResetMotorVariables();
				MotorDrive(DISABLE_D);
				MotorDrive(FORWARD_D); //Reverse to forward
				HAL_Delay(500);
				S.state_change = INTERNAL_TO_IDLE;
				machineRun = 1;
				hommingFlag = 0;
			}
		}

		topLimitSwitch = TopLimitSensor();
		btmLimitSwitch = BtmLimitSensor();
		/*if ((topLimitSwitch)||(btmLimitSwitch)){
			E.RpmErrorFlag = 1;
			S.errStopReason = FF_LIFTLEFT;
			S.errStopReasonHMI = HMI_FF_LIFTLEFT;
			S.errmotorFault = ERR_RPM_ERROR;
			S.errVal = NO_FLOAT_VAR;
		}*/

		if (S.state_change == INTERNAL_TO_IDLE){
				S.current_state =  IDLE_STATE;
				S.prev_state = HOMING_STATE;
				S.first_enter = 1; //  ALLOW THE RPI TO GET A MESG WHEN U GO BACK FROM HOMING
				S.oneTimeflag = 0;
				HAL_TIM_Base_Stop_IT(&htim7);
				U.TXcomplete = 1;
				break;
			}

		//Check for RPM ERROR, to go into halt State
		if (E.RpmErrorFlag == 1)
			{ S.state_change = TO_HALT;
				S.current_state = HALT_STATE;
				S.prev_state = HOMING_STATE;
				S.first_enter = 1;
				S.oneTimeflag = 0;
				S.updateBasePackets = 1 ;
				homeFlag = 0;
				HAL_TIM_Base_Stop_IT(&htim7);
				HAL_Delay(200);
				break;
			}
				
		} // closes while
}
		

