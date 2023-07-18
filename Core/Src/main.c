/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Structs.h"
#include "initialize.h"
#include "CommonConstants.h"
#include "HMI_Fns.h"
#include "HMI_Constants.h"
#include "StateFns.h"
#include "logicDefines.h"
#include "encoder.h" 
#include "initialize.h"
#include "functionDefines.h"
#include "eeprom.h"
#include "Logger.h"
#include "math.h"

#include "encoder.h"
#include "Control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
struct State S;
struct Uart U;
struct FlyerSettings fsp; 
struct Diagnostics D;
// HMI STRUCTS
struct HMI_InfoBase hsb;
struct HMI_RunPacket hrp;
struct HMI_StopPacket hsp;
struct HMI_DiagPacket hdp;
//LOGIC
struct Motor M[7];
struct Error E;
struct PIDUpdate P;
struct FF_AdvSettings FFs;
//NEW STRUCTS added in Ilocos
encoders encs;

//Buffers
char BufferRec [MAX_PACKET_SIZE_FROM_HMI];//BUFFER  FOR RECEIVE
const char* startBufferRec = BufferRec; // Pointer to start of Buffer Rec
char BufferTransmit[130] ;// Buffer for Transmit

//Order of names in Array have to be same as order of motors(motor1,motor2..etc) //TO CHECK ORDER WITH SIVA
char MOTORARRAY[6] = {0,FF_FLYER,FF_BOBBIN,FF_FRONTROLLER,FF_BACKROLLER,FF_LIFTLEFT};
char MOTORARRAY_HMI[6] = {0,HMI_FF_FLYER,HMI_FF_BOBBIN,HMI_FF_FRONTROLLER,HMI_FF_BACKROLLER,HMI_FF_LIFTLEFT};
char DIAGNOSIS_HMI[8] = {0,HMI_FF_FLYER,HMI_FF_BOBBIN,HMI_FF_FRONTROLLER,HMI_FF_BACKROLLER,HMI_LIFT,HMI_DRAFTING,HMI_WINDING};

//FOR STORING THESE ADDRESSES IN THE EEPROM
const int EEPROM_KP_ADDRESSES[6] = {EXTRA,FLYER_KP,BOBBIN_KP,FR_ROLLER_KP,BK_ROLLER_KP,LIFT_LEFT_KP};
const int EEPROM_KI_ADDRESSES[6] = {EXTRA,FLYER_KI,BOBBIN_KI,FR_ROLLER_KI,BK_ROLLER_KI,LIFT_LEFT_KI};
const int EEPROM_START_OFFSET_ADDRESSES[6] = {EXTRA,FLYER_START_OFFSET,BOBBIN_START_OFFSET,FR_ROLLER_START_OFFSET,
BK_ROLLER_START_OFFSET,LIFT_LEFT_START_OFFSET};
const int EEPROM_FEEDFORWARD_ADDRESSES[6] = {EXTRA,FLYER_FF_CONST,BOBBIN_FF_CONST,FR_FF_CONST,
BR_FF_CONST,LIFT_FF_CONST};

//Logic Variables
int keyPress = 0;
int allMotorsOn = 2;
int restartMachine = 0;
int startFlag = 0;
int leftpulseCount = 0;
int rightpulseCount = 0;
int bobbinBed_dir = 1;

//machine States
int homeFlag = 0; // flag to indicate to interrupt to switch on only 2 motors.

extern int logCounter;
//main running variables
int currentLayer = 0;
int rampRpm = 5;
//flag for switching on REceive buffer properly
char firstTimeRXOn = 0;

//diag variables - these are defined in the diagnostic state File
extern unsigned int idxMotor_diag;
extern long diag_pwm;
extern int testMode;
 
//currentLength
float lengthin100ms = 0.01;
float currentLength = 0;

//homing variables
extern int rightMotorOn;
extern int leftMotorOn;

//Variables for through beam sensor
int sensor = 0;
int sensorCount = 0;
int sliverCut;

//Variables for correction of RTF during RAMPUP
float noLoops = 0;
float deltaRTFNeeded  = 0;
int startTimer1 = 0;

//settings check
int settingsEepromCheck = 0;
int PIDsettingsEepromCheck  = 0;
int FF_settingsCheck = 0;

//variables to manage both drives
extern int Cap1 ;
float rpmF = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t sliver= 0;
float FR_RPM= 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  {
	// A 1 sec TIMER
	if (htim->Instance==TIM7){
		LedToggle(LED2);
		U.TXtransfer = 1;
		S.oneSecTimer += 1;
	}

	if (htim->Instance==TIM11){ //Read through beam sensor
		if (S.current_state ==  RUN_STATE){
			sensor = HAL_GPIO_ReadPin(GPIOB,Inp9_Pin);
			if (sensor == 0){
				sensorCount++;
				if (sensorCount > 6){
					sliverCut = 1;
					}
			}
			else{
				sensorCount  = 0;
			}
		}
	}

	//100 ms timer
	if (htim->Instance==TIM6){
		UpdateRPM_Nuvoton_48PPR(&encs);
		UpdateRPM_Tachometric(&encs);

		// will log in Diag and Run
		if (S.loggingEnabled){
			//log Motor running only in Run and Diag.but settings can be logged in idle(1 time)
			if (S.logType == 2){
				logCounter ++;
				LogMotorVariables4();
			}
			if (S.logType == 1){
				LogPIDVals();
				logCounter = 0;
				S.logType = 2;}
			if (S.logType == 0){
				LogSettings();
				S.logType = 1;}
			}

		if (testMode == 1){
			if (D.isSubAssembly){
				if (D.subAssembly == HMI_WINDING){
					updateTargets(MOTOR1,1); //rampUp and continuous
					calculateRpm2();
					followTargets(MOTOR1);
					followTargets(MOTOR2);
					ApplyPwms();
				}
				if (D.subAssembly == HMI_DRAFTING){ // old drives.
					RaiseFlyerVirtualTarget(D.targetRPM);
					calculateRpm2();
					followTargets(MOTOR3);
					followTargets(MOTOR4);
					ApplyPwms();
				}

				//TO DO LIFT ASSEMBLY
				if (D.subAssembly == HMI_LIFT){ // nuvoton drives
					MotorDrive(ENABLE_D);
					// if lift subAssembly
					if (D.liftDirection == HMI_LIFT_UP){
						MotorDrive(FORWARD_D);				// go up
					}
					if (D.liftDirection == HMI_LIFT_DOWN){	// go down
						MotorDrive(REVERSE_D);
					}
					if (D.leftLiftOn){
						UpdateMotorVoltage(MOTOR5,1000); //open loop lift motor travel in diagnostics.
						if (leftpulseCount >= D.targetLiftPulseCount){	// only for Lift check if lift is over here itself
							D.liftOver = 1;
						}
					}
					ApplyPwms();
				} //closes Lift
			}
			else{
				if (D.typeofTest == HMI_DIAG_CLOSED_LOOP){
					updateTargets(idxMotor_diag,1);
					if (idxMotor_diag <=2){ // FF and Bobbin. TODO later LIFT
						MotorDrive(ENABLE_D); // only for FF and bobbin
					}
					followTargets(idxMotor_diag);
					ApplyPwms();
				}
				if (D.typeofTest == HMI_DIAG_OPEN_LOOP){
					if (idxMotor_diag <=2){ // FF and Bobbin
						MotorDrive(ENABLE_D); // only for FF and bobbin
					}
					UpdateMotorVoltage(idxMotor_diag,diag_pwm);
					ApplyPwms();
				}
			} // closes else

		}//closes TestMode

		if (homeFlag == 1){
			LedToggle(LED1);
			MotorDrive(ENABLE_D);
			UpdateMotorVoltage(MOTOR5,1200); // medium homing down
			ApplyPwms();
		}

		if(allMotorsOn == 1){
			LedToggle(LED3);
			if(startFlag == 1){
				M[MOTOR1].rampRpm = FFs.rampUpRate;
				updateTargets(MOTOR1,1);
				calculateRpm2();
				followTargets(MOTOR1);
				followTargets(MOTOR2);
				followTargets(MOTOR3);
				followTargets(MOTOR4);
				followTargets(MOTOR5);
			} //closes startFlag = 1

			if(startFlag == 0)
			{
				if (M[MOTOR1].startOffset>=20) {M[MOTOR1].startOffset = M[MOTOR1].startOffset-2;} //flyer
				if (M[MOTOR2].startOffset>=20) {M[MOTOR2].startOffset = M[MOTOR2].startOffset-2;} //bobbin
				if (M[MOTOR3].startOffset>=20) {M[MOTOR3].startOffset = M[MOTOR3].startOffset-5;} //front roller
				if (M[MOTOR4].startOffset>=20) {M[MOTOR4].startOffset = M[MOTOR4].startOffset-5;} //back roller
				if (M[MOTOR5].startOffset>=20) {M[MOTOR5].startOffset = M[MOTOR5].startOffset-5;} //left lift

				S.RTFmultiplier = FFs.rampDownRtfMultiplier; //constant rtf while ramping down
				FFs.logRTFMultiplier = (int)(S.RTFmultiplier * 100);

				M[MOTOR1].rampRpm = FFs.rampDownRate;
				updateTargets(MOTOR1,0);
				calculateRpm2();
				followTargets(MOTOR1);
				followTargets(MOTOR2);
				followTargets(MOTOR3);
				followTargets(MOTOR4);
				followTargets(MOTOR5);

				if(M[MOTOR1].intTarget <= 50){
					ResetMotorVariables(); // reset all motor struct variables (err,pwm) // not Kp ki and start offset
					ResetEncoderVariables(&encs);
					M[MOTOR1].setRpm = 0;
					M[MOTOR1].pwm = 0;
					M[MOTOR2].setRpm = 0;
					M[MOTOR2].pwm = 0;
					M[MOTOR3].setRpm = 0;
					M[MOTOR3].pwm = 0;
					M[MOTOR4].setRpm = 0;
					M[MOTOR4].pwm = 0;
					M[MOTOR5].setRpm = 0;
					M[MOTOR5].pwm = 0;
					allMotorsOn = 0;
					S.keyState = ENABLE;
					startTimer1 = 0;
				}

			}  // closes startFlag = 0

			ApplyPwms();
			//calculate Total Production
			FR_RPM = (M[3].presentRpm/ILOCOS_FR_GEAR_RATIO);
			lengthin100ms = (FR_CIRCUMFERENCE_M * FR_RPM * 0.10f)/60.0f;	 //production in 100ms
			currentLength  = currentLength + lengthin100ms;
		} // closes all motorson == 1


	}// closes tim6 int
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  MX_UART4_Init();
  MX_TIM11_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  InitializeStateStruct(); // sets state to idle and paired =0
  InitializeFlyerSettings();
  ReadFlyerSettingsFromEeprom(); //UpdateFlyerFrameSettings from EEPROM
  settingsEepromCheck = CheckSettingsReadings();
  if (settingsEepromCheck == 0){
	  LoadFactoryDefaultSettings();
  }

  //Initialize the HMI Structs
  Create_HMI_BasePacket();
  Create_HMI_Run_Packet();
  Create_HMI_StopPacket();
  Create_HMI_DiagPacket();
	
  InitializeFFAdvSettings();
	
  //Init the Motor Struct
  MotorStructInit();
	
  ReadAllPIDSettingsFromEeprom();
  PIDsettingsEepromCheck = CheckPIDSettings();
  if (PIDsettingsEepromCheck == 0){
	  WriteDefaultPIDSettingsIntoEepromAndStruct();
  }
  ResetStartOffsetVars(); // move startOffsetOrig into StartOffset

  //*********************************//
	
  ReadFF_AdvSettingsFromEeprom();
  FF_settingsCheck = CheckFF_AdvSettings();
  if (FF_settingsCheck == 0 ) {
	WriteDefaultFFAdvSettingsIntoEepromAndStruct();
  }
  M[MOTOR1].rampRpm = FFs.rampUpRate; //Now that the correct ramp Up val is there reset it in MotorStruct
  CalculateRTF_MultiplierValues();
			
  //Interrupts on HMI UART- UART1
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);//interrupt on receive buffer not empty(buffer full)
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_TC );//interrupt on Transmission Complete
  HAL_UART_Receive_IT(&huart1,(uint8_t *)BufferRec,MAX_PACKET_SIZE_FROM_HMI);	 // start the receive here.
 	
  HAL_TIM_Base_Start_IT(&htim6); // MAIN TIMER FOR MOTOR CONTROL PWM - 250ms
  //HAL_TIM_Base_Start_IT(&htim11); // the Sliver cut timer. Dont switch on here.
	
  // Switch on all the timers for the PWM , and set them low immediately
  AllTimerOn();
  AllSignalVoltageLow();
	
  //Switch off all tower lamps
  TowerLamp(GREEN_OFF);
  TowerLamp(RED_OFF);
  TowerLamp(AMBER_ON);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim9);

  MotorDrive(DISABLE_D); // to be used for FF and BB only.

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {			
  	if (S.current_state == IDLE_STATE){
  		IdleState();
		}
		
	if (S.current_state == HOMING_STATE){
		HomingState();
	}
		
	if (S.current_state == RUN_STATE){
		RunState();
	}
		
	if (S.current_state == HALT_STATE){
		HaltState();
	}

	if (S.current_state == PAUSE_STATE){
		PauseState();
	}
		
	if (S.current_state == UPDATESETTINGS){
		SettingsState();
	}
		
	if (S.current_state == DIAGNOSTIC_STATE){
		DiagnosticsState();
	}
		
	if (S.current_state == END_STATE){
		EndState();
	}
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 839;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 8400;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8399;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 499;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 9999;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 84;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Relay2_Pin|Relay1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TowerGreen_Pin|TowerRed_Pin|TowerAmber_Pin|RelayJ15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RelayJ17_Pin|RelayJ18_Pin|Led1_Pin|Led2_Pin
                          |Led3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Cap3_Pin Cap4_Pin Cap5_Pin Cap6_Pin
                           Cap7_Pin Cap1_Pin Cap2_Pin */
  GPIO_InitStruct.Pin = Cap3_Pin|Cap4_Pin|Cap5_Pin|Cap6_Pin
                          |Cap7_Pin|Cap1_Pin|Cap2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Cap8_Pin Cap9_Pin Cap10_Pin */
  GPIO_InitStruct.Pin = Cap8_Pin|Cap9_Pin|Cap10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay2_Pin Relay1_Pin */
  GPIO_InitStruct.Pin = Relay2_Pin|Relay1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TowerGreen_Pin TowerRed_Pin TowerAmber_Pin RelayJ15_Pin */
  GPIO_InitStruct.Pin = TowerGreen_Pin|TowerRed_Pin|TowerAmber_Pin|RelayJ15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RelayJ17_Pin RelayJ18_Pin Led1_Pin Led2_Pin
                           Led3_Pin */
  GPIO_InitStruct.Pin = RelayJ17_Pin|RelayJ18_Pin|Led1_Pin|Led2_Pin
                          |Led3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Sig5V_Pin Sig12V_Pin Inp3_Pin Inp4_Pin
                           Inp5_Pin Inp6_Pin Inp7_Pin Inp8_Pin */
  GPIO_InitStruct.Pin = Sig5V_Pin|Sig12V_Pin|Inp3_Pin|Inp4_Pin
                          |Inp5_Pin|Inp6_Pin|Inp7_Pin|Inp8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Sig48V_Pin Inp2_Pin */
  GPIO_InitStruct.Pin = Sig48V_Pin|Inp2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Inp1_Pin */
  GPIO_InitStruct.Pin = Inp1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Inp1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Inp9_Pin */
  GPIO_InitStruct.Pin = Inp9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Inp9_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
