/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "functionDefines.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include "HMI_Fns.h"
#include "HMI_Constants.h"
#include "CommonConstants.h"
#include "Structs.h"
#include "encoder.h"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern char BufferRec[];
extern char BufferTransmit[];
int Cap1;
extern int Cap2;
extern int Cap3;
extern int Cap4;
extern int Cap5;
extern int Cap6;
extern int leftpulseCount;
extern int currentLayer;
extern int bobbinBed_dir;
extern int testMode;

long targetPulseCount = 0;

char msgType = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim11;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_0))
	{
		encs.flyer.rawCount++;
		Cap1++;
	}
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cap1_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_1))
	{
		encs.bobbin.rawCount++;
	}
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cap2_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_2))
	{
		//encs.FR.rawCount++;
	}
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cap3_Pin);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_3))
	{
		encs.BR.rawCount++;
	}
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cap4_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_4))
	{
		encs.lift.rawCount++;
		leftpulseCount++;
		if (testMode!= 1){//so that diag lift doesnt cause an error
			targetPulseCount = (fsp.bobbinHeight*MOTOR_LIFT_MM_TO_PULSE);
			if(leftpulseCount >= targetPulseCount){
			currentLayer = currentLayer+1;
			fsp.bobbinHeight = (fsp.bobbinHeight-(2*fsp.rovingWidth));
			leftpulseCount = 0;
			HAL_GPIO_TogglePin(GPIOD,RelayJ18_Pin);
			bobbinBed_dir = bobbinBed_dir * -1;
			}			
		}
	}
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cap5_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
  
  if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_5)){
		encs.FR.rawCount++; // FR connected to M6
  // Cap6++;
  // rightpulseCount++;
  }  
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cap6_Pin);
  HAL_GPIO_EXTI_IRQHandler(Cap7_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim11);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) == SET){
	// ABORT THE UART RECEIVE
	 HAL_UART_AbortReceive_IT(&huart1);
	 msgType = HMI_BasePacket_Decode(BufferRec);

	 if ((int)msgType == FROM_HMI_IM_PAIRED){
		//stop sending any messages
		HAL_TIM_Base_Stop_IT(&htim7);
		U.TXcomplete = 0;
		U.TXtransfer = 0;

		//And then immedietly send the Settings Packet
		//update the transmit buffer with the Settings Data
		UpdateBasePacket_Modes(CURRENT_MACHINE,HMI_BG_DATA,HMI_SETTINGS_INFO,NO_VAR,HMI_SETTINGS_PACKET_LEN_FF,1);
		// transmit packet size is a global. always need to update before we run
		int sizeofPacket =HMI_GetFlyerSettingsAllPacketString(BufferTransmit,hsb, fsp);
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
		U.TXcomplete = 0;
				 
		S.updateBasePackets = 1; // if we go directly to RUN mode from here we need this flag to get the run screen.is harmless in the IDLE MODE
		S.first_enter = 1; // to update the base packets in any mode. can remove after implementing update base packets properly
		// to goback to idle screen from Diagnostics mode/. Will automatically switch off if motors are running in diag mode
		if ((S.current_state ==  DIAGNOSTIC_STATE) || (S.current_state ==  UPDATESETTINGS)){
			S.state_change = TO_IDLE;
		}
		HAL_TIM_Base_Start_IT(&htim7);
	 }
			 
	 // is signal to go to settings from IDLE
	 if ((int)msgType == FROM_HMI_IN_SETTINGS_CHANGE){
		 S.state_change = TO_SETTINGS;
	 }
			
	 // go from idle to diagnostics state
	 if ((int)msgType == FROM_HMI_IN_DIAGNOSTICS_SET){
		 S.state_change = TO_DIAG;
	 }
			 
	 // will be processed inside the Settings state Fn
	 if ((int)msgType == FROM_HMI_UPDATED_SETTINGS){
		 S.state_change = TO_UPDATE_SETTINGS;
	 }
	 // will go back to idle settings from settings or diagnostics page
	 if ((int)msgType == FROM_HMI_BACK_TO_IDLE){
		 S.state_change = TO_IDLE;
	 }
			 
	 if ((int)msgType == FROM_DIAG_UPDATED_TEST_DETAILS){
		  S.state_change = RUN_DIAG_TEST;
	 }

	 if ((int)msgType == FROM_DIAG_STOP_TEST){
		  S.state_change = TO_DIAG_SWITCH_OFF;
	 }

	if ((int)msgType == SEND_PID_VALS){
		UpdateBasePacket_Modes(CURRENT_MACHINE,HMI_BG_DATA,HMI_PID_INFO,NO_VAR,17,1);
		int sizeofPacket = HMI_PIDSettingsPacket(BufferTransmit,hsb,P.requestedPIDoptionIdx);
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
		U.TXcomplete = 0;
		S.updateBasePackets = 1; // since we ve changed the base packets
		P.requestedPIDoptionIdx = 0;
	}
			 
	 if ((int)msgType == SAVE_PID_VALS){
		if (S.current_state == UPDATESETTINGS){
			S.state_change = TO_UPDATE_PIDSETTINGS;
		 }
	 }

				
			 if ((int)msgType == ADD_TO_RTF)
			 {
				  S.displayRTF = S.displayRTF + 10;
					S.runningRTF = S.displayRTF/1000.0f;
			 }
			 
			 if ((int)msgType == SUBTRACT_FROM_RTF)
			 {	
				  S.displayRTF = S.displayRTF - 10;
				  S.runningRTF = S.displayRTF/1000.0f;
			 }
			HAL_UART_Receive_IT(&huart1,(uint8_t *)BufferRec,MAX_PACKET_SIZE_FROM_HMI);	 // start the receive again.
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);
			__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
		
	}
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cap8_Pin);
  HAL_GPIO_EXTI_IRQHandler(Cap9_Pin);
  HAL_GPIO_EXTI_IRQHandler(Cap10_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	 if (huart->Instance == USART1)
	 { 
		 U.TXcomplete = 1;
	 }
	 
 }

 

/* USER CODE END 1 */
