#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "HMI_Fns.h"
#include "HMI_Constants.h"
#include "CommonConstants.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"

extern struct Motor M[7];
extern char MOTORARRAY_HMI[7];
int debug = 0;
void Create_HMI_BasePacket(void)
{
    hsb.start = HMI_START_END_CHAR;
    hsb.sender = MACHINE_TO_HMI;
    hsb.packetLength = 12; // gets updated
    hsb.machineID = CURRENT_MACHINE_ID;
    hsb.machineType = CURRENT_MACHINE; // gets updated
    hsb.msgType = HMI_SCREEN_DATA; // gets updated
    hsb.nextscreen =  HMI_RUN_SCREEN; // gets updated
    hsb.screen_substate = HMI_RUN_RAMPUP; // gets updated
    hsb.attributeCount = 3; // gets updated
    hsb.endChar = HMI_START_END_CHAR;
}

void Create_HMI_Run_Packet(void)
{
    hrp.tlv1Code = HMI_FF_LAYERS_RUN;
    hrp.tlv1length = 2;
    hrp.tlv1_val = 1;
    hrp.tlv2Code = HMI_FF_RTF;
    hrp.tlv2length = 4;
    hrp.tlv2_val = 1.2;
    hrp.tlv3Code = HMI_FF_PRODUCTION_RUN;
    hrp.tlv3length = 4;
    hrp.tlv3_val = 1.1;

}

void Create_HMI_StopPacket(void)
{
    hsp.tlv1Code = HMI_STOP_REASON_CODE;
    hsp.tlv1length = 2;
    hsp.tlv1_val = FF_FRONTROLLER;
    hsp.tlv2Code = HMI_MOTOR_FAULT_CODE;
    hsp.tlv2length = 2;
    hsp.tlv2_val = ERR_RPM_ERROR;
    hsp.tlv3Code = HMI_ERROR_VAL_CODE;
    hsp.tlv3length =4;
    hsp.tlv3_val = 40;
};

void Create_HMI_DiagPacket(void)
{
    hdp.tlv1Code = HMI_DIAG_TEST_CODE;
    hdp.tlv1length = 2;
    hdp.tlv1_val = 0;
    hdp.tlv2Code = HMI_DIAG_MOTOR_ID_CODE;
    hdp.tlv2length = 2;
    hdp.tlv2_val = 0;
    hdp.tlv3Code = HMI_DIAG_SIGNAL_VOLTAGE_CODE;
    hdp.tlv3length = 2;
    hdp.tlv3_val = 0;
    hdp.tlv4Code = HMI_DIAG_TEST_TIME_CODE;
    hdp.tlv4length = 2;
    hdp.tlv4_val = 0;
}

void UpdateBasePacket_Modes(char machineType,char msgType,char state,char runsubMode,char packetLength,char attributeCount)
{
    hsb.msgType = msgType;
    hsb.machineType = machineType;
    hsb.nextscreen = state;
    hsb.screen_substate = runsubMode;
    hsb.packetLength = packetLength;
    hsb.attributeCount = attributeCount;
}

void UpdateRunPacket_FF(char tlv1Code,int tlv1Val,char tlv2Code,float tlv2Val,char tlv3Code,int tlv3Val)
{
    hrp.tlv1Code = tlv1Code;
    hrp.tlv1length = 2;
    hrp.tlv1_val = tlv1Val;
    hrp.tlv2Code = tlv2Code;
    hrp.tlv2_val = tlv2Val;
    hrp.tlv3Code = tlv3Code;
    hrp.tlv3_val = tlv3Val;

  
}

void UpdateStopPacket(char tlv1Code,char tlv2Code,char tlv3Code)
{
    hsp.tlv1Code = tlv1Code;
    hsp.tlv2Code = tlv2Code;
    hsp.tlv3Code = tlv3Code;
		hsp.tlv3length = 2;
		
}

int UpdateRunPacketString_FF(char *buffer,struct HMI_InfoBase hsb,struct HMI_RunPacket hrp,int layers,float rtf,float prod_total)
{    int sizeofPacket = 0;
		 if (prod_total < 0.1){
			prod_total = 0.1;
		 }
     sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%04X%02X%02X%X%02X%02X%X%02X\r",hsb.start,hsb.sender,hsb.packetLength,
		 hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,hrp.tlv1Code,
		 hrp.tlv1length,layers,hrp.tlv2Code,hrp.tlv2length,*(unsigned int*)&rtf,hrp.tlv3Code,hrp.tlv3length,
		 *(unsigned int*)&prod_total,hsb.endChar);
		 sizeofPacket = 53;
  
    return sizeofPacket;
}

int UpdateStopPacketString(char *buffer,struct HMI_InfoBase hsb,struct HMI_StopPacket hsp,int stop_reason,int motor_fault,int error_val)
{   int sizeofPacket = 0;
	
		int errorVal1 = 0;
		sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%04X%02X%02X%04X%02X%02X%04X%02X\r",hsb.start,hsb.sender,hsb.packetLength,
		hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,hsp.tlv1Code,
		hsp.tlv1length,stop_reason,hsp.tlv2Code,hsp.tlv2length,motor_fault,hsp.tlv3Code,hsp.tlv3length,
		errorVal1,hsb.endChar);
		sizeofPacket = 45;
		
    return sizeofPacket;
}

int UpdatePausePacketString(char *buffer,struct HMI_InfoBase hsb,struct HMI_StopPacket hsp,int stop_reason,int motor_fault,float error_val)
{   int sizeofPacket = 0;
	
		sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%04X%02X%02X%04X%02X%02X%08X%02X\r",hsb.start,hsb.sender,hsb.packetLength,
		hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,hsp.tlv1Code,
		hsp.tlv1length,stop_reason,hsp.tlv2Code,hsp.tlv2length,motor_fault,hsp.tlv3Code,hsp.tlv3length,
		*(unsigned int*)&error_val,hsb.endChar);
		sizeofPacket = 49;
		
    return sizeofPacket;
}


int HMI_GetFlyerSettingsAllPacketString(char *buffer,struct HMI_InfoBase hsb,struct FlyerSettings fsp)
{ int sizeofPacket = 0;
  sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02d%04X%X%X%X%04X%04X%X%X%04X%04X%04X%02X\r",hsb.start,hsb.sender,
         hsb.packetLength,hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,
         HMI_FF_ALL_SETTING_CODE,HMI_FF_ALL_SETTINGS_PKT_LEN,fsp.spindleSpeed,*(unsigned int*)&fsp.tensionDraft,
				*(unsigned int*)&fsp.tpi,*(unsigned int*)&fsp.rtf,fsp.lengthLimit,fsp.bobbinHeight,
					*(unsigned int*)&fsp.rovingWidth,*(unsigned int*)&fsp.deltaBobbinDia,fsp.bareBobbinDia,fsp.boostFactor,
						fsp.buckFactor,hsb.endChar);
  sizeofPacket = 89;
  return sizeofPacket;
}

int HMI_PIDSettingsPacket(char *buffer,struct HMI_InfoBase hsb,int motorIndex)
{ int sizeofPacket = 0;
	
	unsigned int attr1,attr2,attr3,attr4,motorCode = 0;
	if (motorIndex < 7 ){
		 attr1 = M[motorIndex].Kp*100;
		 attr2 = M[motorIndex].Ki*100;
		 attr3 = M[motorIndex].startOffsetOrig;
		 attr4 = M[motorIndex].ff_multiplier*100;
		 motorCode  = MOTORARRAY_HMI[motorIndex];
	}else if (motorIndex == 7){
		attr1 = FFs.rampUpRtfMultiplier * 100;
		attr2 = FFs.rampDownRtfMultiplier *100 ;
		attr3 =0;
		attr4 =0;
		motorCode = 33;
	}else{
		attr1 = FFs.rampUpRate;
		attr2 = FFs.rampDownRate ;
		attr3 = 0 ;
		attr4 = 0;
		motorCode = 34;
	}
	
  sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02d%04X%04X%04X%04X%04X%02X\r",hsb.start,hsb.sender,
         hsb.packetLength,hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,
         HMI_PID_ATTRIBUTES,PID_ATTRIBUTE_LEN,motorCode,attr1,attr2,attr3,attr4,hsb.endChar);
  sizeofPacket = 45;
  return sizeofPacket;
}


int HMI_GetIdlePacketString(char *buffer,struct HMI_InfoBase hsb)
{	int sizeofPacket = 0;
  sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\r",hsb.start,hsb.sender,
         hsb.packetLength,hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,
         hsb.endChar);
  sizeofPacket = 21;
  return sizeofPacket;
	
}

int HMI_GetSettingsACKPacketString(char *buffer,struct HMI_InfoBase hsb)
{	int sizeofPacket = 0;
	
  sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%04X%02X\r",hsb.start,hsb.sender,
         hsb.packetLength,hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,
         0,0,0,hsb.endChar);
  sizeofPacket = 29;
  return sizeofPacket;
	
}

int UpdateDiagPacketString(char *buffer,struct HMI_InfoBase hsb,struct HMI_DiagPacket hdp,int typeOfTest,int motorID,int signalVoltage,int rpm)
{    int sizeofPacket = 0;
    //tlv1-type of test,tlv2 - motorID .tlv3 - sigal voltage ,tlv4 - current trpm
		sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%04X%02X%02X%04X%02X%02X%04X%02X%02X%04X%02X\r",hsb.start,hsb.sender,hsb.packetLength,
		hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,hdp.tlv1Code,
		hdp.tlv1length,typeOfTest,hdp.tlv2Code,hdp.tlv2length,motorID,hdp.tlv3Code,hdp.tlv3length,
    signalVoltage,hdp.tlv4Code,hdp.tlv4length,rpm,hsb.endChar);
    sizeofPacket = 53;
    
    return sizeofPacket;
}

int HMI_Get_DiagOver_PacketString(char *buffer,struct HMI_InfoBase hsb,char status)
{	int sizeofPacket = 0;
	
  sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%04X%02X\r",hsb.start,hsb.sender,
         hsb.packetLength,hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,status,1,
         HMI_DIAG_END_OF_TEST_CODE,2,0,hsb.endChar);
  sizeofPacket = 29;
  return sizeofPacket;
	
}
//--------------------------HELPER FNS------------------------------------
unsigned char charToHexDigit(char c)
{
	if(islower(c)){
            /* Convert lower case character to upper case 
              using toupper function */
							c = toupper(c);
							} 
  if (c >= 'A')
    return c - 'A' + 10;
  else
    return c - '0';
}

unsigned int StringToHex(char c[2])
{
  return charToHexDigit(c[1]) + 16 * charToHexDigit(c[0]);
}

unsigned int stringToINT(char c[4])
{
  return charToHexDigit(c[3]) + 16 * charToHexDigit(c[2]) + 256 * charToHexDigit(c[1])  + 4096 * charToHexDigit(c[0]);
}

unsigned int StringDecodeAsInteger(char c[4])
{
  return charToHexDigit(c[3]) + charToHexDigit(c[2])*10 + charToHexDigit(c[1]) * 100 + charToHexDigit(c[0])*1000;

}

float stringToFLOAT(char c[8])
{
  return charToHexDigit(c[7]) + 16 * charToHexDigit(c[6]) + 256 * charToHexDigit(c[5])  + 4096 * charToHexDigit(c[4]) + 65536 * charToHexDigit(c[3]) + 1048576 * charToHexDigit(c[2]) + 16777216 * charToHexDigit(c[1])  +  268435456 * charToHexDigit(c[0]);
}
//----------------------------------------------------------------------------



char state;
int hex_msgInfo;
int hex_rtfUpdate;
char rtfUpdate [4];
unsigned int pidmsgType;
char HMI_BasePacket_Decode(char *receiveBuffer)
{
	char machineID[2];
	char machineType[2];
	char msgInfo[2];
	char disable_start_msg [2];
	//settings
  char spindleSpeed [4];
  char tensionDraft [8];
  char tpi[8];
  char rtf[8];
  char lengthLimit[4] ;
  char bobbinHeight[4];
  char rovingWidth[8];
  char deltaBobbinDia[8];
	char bareBobbinDia[4];
	char boostFactor[4];
	char buckFactor[4];
  

	
	char motorID[4];
	char typeOfTest[4];
	char targetRPM[4];
	char targetSignalVoltage[4];
	char testTime[4];

	char pidMotorID[4];
	char pidMotorMsgType[2];
	char pidupdateAttr2[4];
	char pidupdateAttr1[4];
	char pidupdateAttr3[4];
	char pidupdateAttr4[4];

	int hex_disable_start_msg;

	
	//settings
	unsigned int hexSpindleSpeed;
	unsigned long hex_tensionDraft ;
	unsigned long hex_tpi ;
	unsigned long hex_rtf ;
	unsigned int hex_maxRovingLayers ;
	unsigned int hex_bobbinHeight ;
	unsigned long hex_rovingWidth ;
	unsigned long hex_deltaBobbinDia ;
	unsigned int hex_bareBobbinDia;
	unsigned int hex_boostFactor;
	unsigned int hex_buckFactor;

	float tensionDraft_F;
	float tpi_F;
	float rtf_F;
	float rovingWidth_F;
	float deltaBobbinDia_F;
	//diag
	unsigned int	D_motorID ;
	unsigned long D_typeOfTest ;
	unsigned int	D_targetRPM ;
	unsigned int	D_targetSignalVoltage ;
	unsigned int	D_testTime ;
	
	//pid
	unsigned int PID_motorID;
	
	//start char will be 7E, dont take that out
	//right now we re getting:
	//7E02B0101029900010002007E  - > im paired MSG -> here we want the 99
	//7E020B010102030001010200017E  -> do not start packet -> here we want the 01/02/00 before the 7E(telling us which screen teh app is in)
	//7E0021010102040001102400DC4020000004DD4093333303203F99999A000C01C200787E -> settings change packet. the identifier is 
	// the 04, and then the vals come.

	strncpy(machineID,receiveBuffer+6,2);
	strncpy(machineType,receiveBuffer+8,2);
	strncpy(msgInfo,receiveBuffer+12,2);
	strncpy(disable_start_msg,receiveBuffer+24,2);
	strncpy(rtfUpdate,receiveBuffer+22,4);

	hex_msgInfo = StringToHex(msgInfo);
	hex_disable_start_msg = StringToHex(disable_start_msg);
	hex_rtfUpdate = stringToINT(rtfUpdate);
		
	if ((int)hex_msgInfo == FROM_HMI_IM_PAIRED)
		{
		state =1;
		return hex_msgInfo;
		}
	else if ((int)hex_msgInfo == FROM_HMI_DISABLE_MACHINE_START) //siable start can 0,1, or 2
		{
			state = 2;
		return hex_disable_start_msg;
		}
  else if ((int)hex_msgInfo == FROM_HMI_ENABLE_MACHINE_START)  
		{
			state = 3;
		return hex_disable_start_msg;
		}	
	else if ((int)hex_msgInfo == FROM_HMI_CHANGE_PROCESS_PARAMS)
		{
			state = 4;
			
			// update the update struct
			//flyer settings msg
			// 7E 01 25 01 03 02 04 00 01 60 24 0320 410ccccd 3f666666 3f2e147b 0041 0136 3f8ccccd 3f99999a 0030 7E

			//delivery speed comes as float, but we make it int
			strncpy(spindleSpeed,receiveBuffer+22,4);
			strncpy(tensionDraft,receiveBuffer+26,8);
			strncpy(tpi,receiveBuffer+34,8);
			strncpy(rtf,receiveBuffer+42,8);
			strncpy(lengthLimit,receiveBuffer+50,4);
			strncpy(bobbinHeight,receiveBuffer+54,4);
			strncpy(rovingWidth	,receiveBuffer+58,8);
			strncpy(deltaBobbinDia	,receiveBuffer+66,8);
			strncpy(bareBobbinDia	,receiveBuffer+74,4);
			strncpy(boostFactor	,receiveBuffer+78,4);
			strncpy(buckFactor	,receiveBuffer+82,4);
			
			hexSpindleSpeed = stringToINT(spindleSpeed);
			hex_tensionDraft = stringToFLOAT(tensionDraft);
			hex_tpi = stringToFLOAT(tpi);
			hex_rtf = stringToFLOAT(rtf);
			hex_maxRovingLayers = stringToINT(lengthLimit);
			//if((hex_maxRovingLayers % 2) == 0) // always keep it such that you have an odd no of layets so you will always stop up
			//	hex_maxRovingLayers = hex_maxRovingLayers+1;
			
			hex_bobbinHeight = stringToINT(bobbinHeight);
			hex_rovingWidth = stringToFLOAT(rovingWidth);
			hex_deltaBobbinDia = stringToFLOAT(deltaBobbinDia);
			hex_bareBobbinDia = stringToINT(bareBobbinDia);
			hex_boostFactor = stringToINT(boostFactor);
			hex_buckFactor = stringToINT(buckFactor);
			
			tensionDraft_F = *((float*)&hex_tensionDraft);
			tpi_F = *((float*)&hex_tpi);
			rtf_F = *((float*)&hex_rtf);
			rovingWidth_F = *((float*)&hex_rovingWidth);
			deltaBobbinDia_F = *((float*)&hex_deltaBobbinDia);

			//update fsp directly
			fsp.spindleSpeed = hexSpindleSpeed;
			fsp.tensionDraft = tensionDraft_F;
			fsp.tpi = tpi_F;
			fsp.rtf = rtf_F;
			fsp.lengthLimit = hex_maxRovingLayers;
			fsp.bobbinHeight = hex_bobbinHeight;
			fsp.rovingWidth = rovingWidth_F;
			fsp.deltaBobbinDia = deltaBobbinDia_F;
			fsp.bareBobbinDia = hex_bareBobbinDia;
			fsp.boostFactor = hex_boostFactor;
			fsp.buckFactor = hex_buckFactor;
			
			return FROM_HMI_UPDATED_SETTINGS;
		}
	else if ((int)hex_msgInfo == FROM_HMI_DIAGNOSTIC_TEST)
	{
		state = 5;
		// Update the Diagnostics Struct
		//7E 02 55 01 01 02 05 00 05 01 02 0002 02 02 0005 03 02 0000 04 02 0078 05 02 0025 7E

		strncpy(typeOfTest,receiveBuffer+22,4);
		strncpy(motorID,receiveBuffer+30,4);
		strncpy(targetSignalVoltage,receiveBuffer+38,4);
		strncpy(targetRPM,receiveBuffer+46,4);
		strncpy(testTime,receiveBuffer+54,4);

		D_typeOfTest = stringToINT(typeOfTest);
		D_motorID = StringDecodeAsInteger(motorID);
		D_targetSignalVoltage = stringToINT(targetSignalVoltage);
		D_targetRPM = stringToINT(targetRPM);
		D_testTime = stringToINT(testTime);
		
		D.typeofTest = D_typeOfTest;
		D.motorID = D_motorID;
		D.targetSignal = D_targetSignalVoltage;
		D.targetRPM = D_targetRPM;
		D.testTime = D_testTime;
		
		return FROM_DIAG_UPDATED_TEST_DETAILS;
	}
	
	else if ((int)hex_msgInfo == FROM_HMI_PID_PROCESS){
		strncpy(pidMotorMsgType,receiveBuffer+18,2);
		pidmsgType = StringToHex(pidMotorMsgType);
		if  (pidmsgType == FROM_HMI_PID_REQUEST){
			strncpy(pidMotorID,receiveBuffer+22,4);
			PID_motorID = StringDecodeAsInteger(pidMotorID);
			//figure out which motor we ve asked details for
			int motorIdx = 0;
			for (int i=0;i< sizeof(MOTORARRAY_HMI);i++){
				if (MOTORARRAY_HMI[i] == (char)PID_motorID){
					motorIdx = i;
					P.requestedPIDoptionIdx = motorIdx;
					break;
				}
			}

			 //if not a motor then it must be something else, either start or stop vars
			if (motorIdx == 0){
				if ((char)PID_motorID == HMI_PID_START_VARS){
					P.requestedPIDoptionIdx = 7;
				}
				if ((char)PID_motorID == HMI_PID_STOP_VARS){
					P.requestedPIDoptionIdx = 8;
				}
			}
			return SEND_PID_VALS;
		}

		if  (pidmsgType == FROM_HMI_PID_NEWSETTINGS){
			strncpy(pidMotorID,receiveBuffer+22,4);
			strncpy(pidupdateAttr1,receiveBuffer+26,4);
			strncpy(pidupdateAttr2,receiveBuffer+30,4);
			strncpy(pidupdateAttr3,receiveBuffer+34,4);
			strncpy(pidupdateAttr4,receiveBuffer+38,4);

			P.updateOptionID = StringDecodeAsInteger(pidMotorID);
			P.updateAttr1 = stringToINT(pidupdateAttr1);
			P.updateAttr2 = stringToINT(pidupdateAttr2);
			P.updateAttr3 = stringToINT(pidupdateAttr3);
			P.updateAttr4 = stringToINT(pidupdateAttr4);
			P.updatePIDVals = 1;
			return SAVE_PID_VALS;
			}
		}
	else if ((int)hex_msgInfo == FROM_HMI_ENABLE_LOGGING){
		S.logType =0; // everytime you start logging you ll get some settings also
		S.loggingEnabled = 1;
		state = 7;
	}
	else if ((int)hex_msgInfo == FROM_HMI_DISABLE_LOGGING){
		S.loggingEnabled = 0;
		S.logType = 0;
		state = 8;
	}
	
	else if (( int)hex_msgInfo == FROM_HMI_RTF_UPDATE){ // if you get an RTF update msg
		state = 5;
		if (S.current_state == HOMING_STATE){
			return HOMING_RTF_CHANGE;
		}
		else{
			if (hex_rtfUpdate == RTF_ADD){
				return ADD_TO_RTF;}
			else if (hex_rtfUpdate == RTF_SUBTRACT){
				return SUBTRACT_FROM_RTF;}
			else{
				return NO_VAR;}
		}
	}
		
	else{
		state =6;
		return NOTHING;
	}
	
	 return NOTHING;
	
}


