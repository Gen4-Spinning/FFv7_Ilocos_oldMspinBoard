#include "StateFns.h"
#include "Structs.h"
#include "CommonConstants.h"
#include "HMI_Constants.h"
#include "HMI_Fns.h"
#include "stm32f4xx_hal.h"
#include "eeprom.h"
#include "encoder.h"
#include "functionDefines.h"
#include "logicDefines.h"
#include "main.h" 
#include "math.h"

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim7;
extern uint8_t BufferRec[];
extern char BufferTransmit[];

//logic varaibles for diagnostic test
extern char MOTORARRAY[7];
extern char MOTORARRAY_HMI[7];
extern char DIAGNOSIS_HMI[8];

extern int startFlag;
extern int leftpulseCount;
extern int rightpulseCount;
//global variables for testing defined here
int testMode = 0;
long diag_pwm = 0;
int idxMotor_diag = 0;	
int current_pwm_percent = 0;
char compareMotorID = 0;
int rpi_type_ofTest   = 0;
char finalStatus = 0;
int finalErrorDiag = 0;
int lift_mm = 0;

void DiagnosticsState(void){
	int sizeofPacket = 0;
	while(1){
		//start the timer
		if (HAL_TIM_Base_GetState(&htim7) ==  HAL_TIM_STATE_READY){
			HAL_TIM_Base_Start_IT(&htim7); // currently at 1 sec
		}
						
		// if youve got the signal that youve got the test vals from the HMI
		if (S.state_change == RUN_DIAG_TEST){ 			//Reset the one sec timer to keep track of how long the test should run
			S.oneSecTimer = 0;						// so that we dont keep restarting the timer
			S.state_change = TO_DIAG;
			UpdateBasePacket_Modes(CURRENT_MACHINE,HMI_BG_DATA,HMI_DIAG_RESULTS,NO_VAR,19,4);	// set the HMIbasepacket values
			for (int i=0;i< sizeof(DIAGNOSIS_HMI);i++){// start the test mode after setting the variables to the correct vals
				compareMotorID = (char) D.motorID;
				if (DIAGNOSIS_HMI[i] == compareMotorID){
					if (i > 4){
						D.isSubAssembly = 1;
						D.subAssembly = DIAGNOSIS_HMI[i];
						D.liftOver = 999; // not a lift subAssembly. Gets overWritten if it is.
						if (D.subAssembly == HMI_LIFT){
							//Lift test type is sent in target Signals place
							//D.targetRpm is Lift Direction
							//lift Distance in mm is in D.testTime. only ue left Lift on
							if ((D.targetSignal == HMI_LIFT_LEFT_ONLY) || (D.targetSignal == HMI_LIFT_BOTH)){
								D.leftLiftOn = 1;
							}
							D.liftDirection = D.targetRPM;
							// Need to Check.
							D.targetLiftPulseCount = D.testTime * MOTOR_LIFT_MM_TO_PULSE; // = * mm To pulses.
							D.liftOver = 0;
							D.testTime = 65000; // this is to ensure that it never trips because of the time check when deciding when the run is over.
						    S.oneSecTimer = 0; // same as above
							leftpulseCount = 0;
						}
					}else{
						D.isSubAssembly = 0;
						D.subAssembly = 0;
						D.liftOver = 999; // not a lift Setup
						idxMotor_diag = i;
									
						//set the open loop val
						diag_pwm =  (MAX_PWM * D.targetSignal)/100;
						//set the target RPM
						M[idxMotor_diag].setRpm = D.targetRPM;
					}
					break;
				}
			}// closes the loop where we're findinf the motor ID
				
			//set the test mode and start the test
			testMode = 1;
		}
					
		if (testMode == 1){
			//when the onesec timer goes more than the test time,stop the motor
			if ((S.oneSecTimer > D.testTime) || (D.liftOver == 1)){
				//if your running lift, update the text on the app with the latest readings once.
				if (D.liftOver ==1){
					if (D.leftLiftOn == 0){
						lift_mm = rightpulseCount / MOTOR_LIFT_MM_TO_PULSE ;//* (mm converision)
					}else{
						lift_mm = leftpulseCount / MOTOR_LIFT_MM_TO_PULSE ;//* (mm converision)
					}
					sizeofPacket = UpdateDiagPacketString(BufferTransmit,hsb,hdp,D.typeofTest,D.motorID,lift_mm,0);
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
				}
							
				// now check what kind of success message to send
				//if your running subAssembly also, since closed loop function anyway checks for delta RPM with target
				// if we re done with the time we just send out success ! (TO CHECK)
				if ((D.typeofTest == HMI_DIAG_OPEN_LOOP) || (D.isSubAssembly == 1)){
					finalStatus = HMI_DIAG_TEST_SUCCESSFUL;
				}
				else{
					finalErrorDiag = M[idxMotor_diag].presentRpm - D.targetRPM ;
					// send test over mesg, send a rpi msg, reset the motor variables to their defaults.
					if (fabs(finalErrorDiag) <= 50) {
						finalStatus = HMI_DIAG_TEST_SUCCESSFUL;
					}
					else{
						finalStatus = HMI_DIAG_TEST_FAIL;
					}
				}
				testMode = 0;
				startFlag = 0;
				MotorDrive(DISABLE_D); //actually disable
				AllSignalVoltageLow(); // switch off all motors.
				ResetMotorVariables(); // put back original values
				ResetEncoderVariables(&encs);//reset encoder vals
				// send test over mesg, send a rpi msg, reset the motor variables to their defaults.
				sizeofPacket = HMI_Get_DiagOver_PacketString(BufferTransmit,hsb,finalStatus);

				HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
				U.TXcomplete = 0;
				U.TXtransfer = 0;

				//Reset the Diag Packets
				idxMotor_diag = 0;
				diag_pwm = 0;
				D.isSubAssembly = 0;
				D.subAssembly = 0;

				//lift stuff
				D.leftLiftOn = 0;
				D.rightLiftOn	 = 0;
				D.liftDirection = 0;
				D.targetLiftPulseCount = 0;
				M[MOTOR5].setRpm = 0;
		}
		else{ 														// if time is not over
			if (D.isSubAssembly){									// if subAssembly
				if (D.subAssembly == HMI_DRAFTING){					//if drafting
					//type of test and motorID are unused
					sizeofPacket = UpdateDiagPacketString(BufferTransmit,hsb,hdp,D.typeofTest,D.motorID,M[3].presentRpm,M[4].presentRpm);
				}else if (D.subAssembly == HMI_WINDING){			// if winding
					//type of test and motorID are unused
					sizeofPacket = UpdateDiagPacketString(BufferTransmit,hsb,hdp,D.typeofTest,D.motorID,M[1].presentRpm,M[2].presentRpm);
				}else{																									// if lift
				//for lift send something
					lift_mm = leftpulseCount / MOTOR_LIFT_MM_TO_PULSE ;//* (mm converision)

					sizeofPacket = UpdateDiagPacketString(BufferTransmit,hsb,hdp,D.typeofTest,D.motorID,lift_mm,0);
				}
			}
			else{									// if not subassembly, ie, single motor
				current_pwm_percent = ((M[idxMotor_diag].pwm * 100)/MAX_PWM);
				if (D.typeofTest == HMI_DIAG_CLOSED_LOOP){							// if closed loop
					sizeofPacket = UpdateDiagPacketString(BufferTransmit,hsb,hdp,D.typeofTest,D.motorID,current_pwm_percent,M[idxMotor_diag].presentRpm);
					}
				if (D.typeofTest == HMI_DIAG_OPEN_LOOP){								// if open loop
					sizeofPacket = UpdateDiagPacketString(BufferTransmit,hsb,hdp,D.typeofTest,D.motorID,current_pwm_percent,M[idxMotor_diag].presentRpm);
				}
			}
			if ((U.TXcomplete ==1) && (U.TXtransfer == 1)){						// send the data!
				HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
				U.TXcomplete = 0;
				U.TXtransfer = 0;
			}
		} 	// closes if time not over
	}		//closes if test mode == 1

			
		if (S.state_change == TO_DIAG_SWITCH_OFF){
		 // stop the test
				testMode = 0;
				startFlag = 0;

				MotorDrive(DISABLE_D);
				AllSignalVoltageLow(); // switch off all motors.

				ResetMotorVariables(); // put back original values
				ResetEncoderVariables(&encs); // Reset encoder vals
				//Reset the Diag Packets
				idxMotor_diag = 0;
				diag_pwm = 0;
				D.isSubAssembly = 0;
				D.subAssembly = 0;

				//lift stuff
				D.leftLiftOn = 0;
				D.rightLiftOn	 = 0;
				D.liftDirection = 0;
				D.targetLiftPulseCount = 0;
				D.liftOver = 999;
				M[MOTOR5].setRpm = 0;
				S.state_change = TO_DIAG;
			}
						
			if (S.state_change == TO_IDLE)
				{
					S.current_state =  IDLE_STATE;
					S.prev_state = DIAGNOSTIC_STATE;
					S.first_enter = 1; // ALLOW THE RPI TO GET A MESG WHEN U GO BACK FROM DIAGNOSTICS
					S.oneTimeflag = 0;
					HAL_TIM_Base_Stop_IT(&htim7);
					U.TXcomplete = 1;
					//reset the pulse Counts
					leftpulseCount = 0;
					rightpulseCount = 0;
					//Reset incase youve found an error!
					E.RpmErrorFlag = 0;
					break;
				}

			if (S.state_change == TO_SETTINGS){
			S.current_state =  UPDATESETTINGS;
			S.prev_state = DIAGNOSTIC_STATE;
			S.first_enter = 1;
			S.oneTimeflag = 0;
			HAL_TIM_Base_Stop_IT(&htim7);
			U.TXcomplete = 1;

			//Reset incase youve found an error!
			E.RpmErrorFlag = 0;
			break;
			}

					
			if (E.RpmErrorFlag == 1) {
				//switch of the test
				testMode = 0;
				MotorDrive(DISABLE_D);
				AllSignalVoltageLow(); // switch off all motors.
				ResetMotorVariables(); // put back original values
				ResetEncoderVariables(&encs); // Reset encoder vals

				finalStatus = HMI_DIAG_TEST_MOTORERROR;
				sizeofPacket = HMI_Get_DiagOver_PacketString(BufferTransmit,hsb,finalStatus);
				if ((U.TXcomplete ==1) && (U.TXtransfer == 1))
				{ HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
					U.TXcomplete = 0;
					U.TXtransfer = 0;
				}

				//Reset the Diag Packets
				idxMotor_diag = 0;
				diag_pwm = 0;
				D.isSubAssembly = 0;
				D.subAssembly = 0;
				//reset the pulse Counts
				leftpulseCount = 0;
				rightpulseCount = 0;

			}
						
	}
							
}


