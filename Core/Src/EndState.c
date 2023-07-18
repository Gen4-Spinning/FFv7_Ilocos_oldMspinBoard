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
extern TIM_HandleTypeDef htim6;
extern uint8_t BufferRec[];
extern char BufferTransmit[];

extern int homeFlag ;
extern char homing_Normal_State ;
extern char btm_bed_reached ;
extern int leftpulseCount;
extern int rightpulseCount;
extern int allMotorsOn;

int goToHoming = 0;

//CANT GET OUT OF END STATE WITHOUT RESTARTING MACHINE, (APP WILL  show halt, and machine will slow down and then HOME
void EndState(void)
{ 
	int sizeofPacket = 0;
	while(1)
			{				
						// and start the timer that signals when to send the data.
						if (HAL_TIM_Base_GetState(&htim7) ==  HAL_TIM_STATE_READY) // if switched off
							{
								HAL_TIM_Base_Start_IT(&htim7); // currently at 1 sec								
							}
							
						/* Motor Logic if any */
						TowerLamp(GREEN_ON);
						TowerLamp(RED_ON);//on
						TowerLamp(AMBER_ON);
						
						//check for all the motors slowed down.
						if (S.keyState == ENABLE){
							//Stop everything
							AllSignalVoltageLow();
							MotorDrive(DISABLE_D);
							//Stop the UpdateTImer
							allMotorsOn = 0;
							//HAL_TIM_Base_Stop_IT(&htim6);	
							S.keyState = DISABLE; //  so that we dont again come back and set the home flag
						}
							
						/*********COMM LOGIC CONTINUED********/
							
						if (S.updateBasePackets == 1)
						{	
							// keep it here because this will run once and we want this buffer packet to get created only once.
							// prepare the Stop packet Structure
						  UpdateBasePacket_Modes(CURRENT_MACHINE,HMI_SCREEN_DATA,HMI_STOP_SCREEN,HMI_STOP_HALT,HMI_STOP_PACKET_LENGTH,3);
						  UpdateStopPacket(HMI_STOP_REASON_CODE,NO_VAR,NO_VAR);		
						  S.updateBasePackets = 0;							
						}
						
					//send Stop packet
						if ((U.TXcomplete ==1) && (U.TXtransfer == 1)) // TxTransfer signals when to send the data
						{
							 sizeofPacket = UpdateStopPacketString(BufferTransmit,hsb,hsp,S.errStopReasonHMI,S.errmotorFault,S.errVal);
							 HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
							 U.TXcomplete = 0; 
							 U.TXtransfer = 0;
						}
					
			
			} //closes while
					
}
							
