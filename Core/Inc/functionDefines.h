#ifndef __FUNCTIONDEFINES_H
#define __FUNCTIONDEFINES_H

void TimerLow(char index);
void AllSignalVoltageLow(void);
void ApplyPwms(void);
void ResetStartOffsetVars(void);
void MotorTimer(char index);	
void RaiseFlyerVirtualTarget(int virtualTarget);

void MotorStructInit(void);
void UpdateMotorPID(uint8_t motorIndex);
void UpdateMotorPiecing(uint8_t motorIndex);
char RpmError(void);
char CheckAllMotors(void);
void AllTimerOn(void);
void StepMotorPID(uint8_t motorIndex);
void StepMotorPID_w_DCOffset(uint8_t motorIndex,int offset);
void ResetMotorVariables(void);
void UpdateMotorPID_w_DCOffset(uint8_t motorIndex,int dc_offset);
void calculateRpm(void);

char Pushbutton(void);

void LedOn(char index);
void LedOff(char index);
void LedToggle(char index); 

char RightCalibSensor(void);
char LeftCalibSensor(void);
char InputSensor1(void);
char InputSensor2(void);
char SliverCutSense(void);

void BedCalibSensorStatus(void);

void CalculateDeltaPulse(void);
void BobbinBedAutoCalib(void);
void AutoCorrectBobbinBed(void);

int BobbinBedAutoCalib2(void);
void ResetBobbinBedAutoCalib2(void);
int GetCorrectionPulseNo(int finalDeltaTimeCounter);
int GetTargetPulseDiff(int correctionDeltaPulseDiff);
void SetCorrectionRPMinMotor(void);
void ResetCorrectionRPMinMotor(void);
int CheckCorrectionOver(void);

void TowerLamp(char index);

void MotorDrive(char index);
void PauseMachine(void);
void calculateRpm(void);
void calculateRpm2(void);

char InputVoltageSense(void);
void RunMachine(void);


char MaxLayerComplete(void);
char LiftLimitCheck(void);
char LengthLimitReached(void);

void CalculateRTF_MultiplierValues(void);
void ResetRampUp_RTFMultiplier(void);


void ApplyPwms_BhagwatDrives(void);
void ApplyPwms_FFAndBB(void);

char HomingSensor(void);
char TopLimitSensor(void);
char BtmLimitSensor(void);

#endif 
