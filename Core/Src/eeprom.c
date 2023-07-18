
#include "eeprom.h"
#include "Structs.h"
#include "CommonConstants.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c3;
extern int EEPROM_KP_ADDRESSES[6];
extern int EEPROM_KI_ADDRESSES[6];
extern int EEPROM_START_OFFSET_ADDRESSES[6];
extern int EEPROM_FEEDFORWARD_ADDRESSES[6];


void EepromWriteInt(unsigned position,unsigned int data)
{
	int count = 0;
	if (data>255)
	{
		while(data>255)
		{
			data = data-255;
			count++;
		}
	}
	
	if(count>0)
	{
		HAL_I2C_Mem_Write(&hi2c3,EEPROM_ADDRESS, position,0xFF,(uint8_t*)&count,1,1);
		HAL_Delay(5);
		HAL_I2C_Mem_Write(&hi2c3,EEPROM_ADDRESS, position+1,0xFF,(uint8_t*)&data,1,1);
		HAL_Delay(5);
	}
	else
	{	
		HAL_I2C_Mem_Write(&hi2c3,EEPROM_ADDRESS, position,0xFF,(uint8_t*)&count,1,1);
		HAL_Delay(5);
		HAL_I2C_Mem_Write(&hi2c3,EEPROM_ADDRESS, position+1,0xFF,(uint8_t*)&data,1,1);
		HAL_Delay(5);
	}

}

unsigned int EepromReadInt(unsigned position)
{
	int count =0;
	unsigned int data = 0;
	HAL_I2C_Mem_Read(&hi2c3,EEPROM_ADDRESS,position,0xFF,(uint8_t*)&count,1,1);
	HAL_I2C_Mem_Read(&hi2c3,EEPROM_ADDRESS,position+1,0xFF,(uint8_t*)&data,1,1);
	data = data+(count*255);
	return data;
}


void EepromWriteFloat(unsigned position,float data)
{
	unsigned int data_in = 0;
	int count3 = 0;
	int count2 = 0;
	int count1 = 0;
	data_in =  *(unsigned int*)&data;
	if (data_in>16777216)
	{
		while(data_in>16777216)
		{
			data_in = data_in-16777216;
			count3++;
		}
	}
		if (data_in>65536)
	{
		while(data_in>65536)
		{
			data_in = data_in-65536;
			count2++;
		}
	}
	if (data_in>255)
	{
		while(data_in>255)
		{
			data_in = data_in-255;
			count1++;
		}
	}
	
	EepromWriteInt(position,count3);
	EepromWriteInt(position+2,count2);
	EepromWriteInt(position+4,count1);
	EepromWriteInt(position+6,data_in);
}


unsigned int EepromReadFloat(unsigned position)
{
	long data = 0;
	unsigned int count3_in = 0;
	unsigned int count2_in = 0;
	unsigned int count1_in = 0;
	unsigned int count0_in = 0;
	count3_in = EepromReadInt(position);
	count2_in = EepromReadInt(position+2);
	count1_in = EepromReadInt(position+4);
	count0_in = EepromReadInt(position+6);
	data = count0_in+(count1_in*255)+(count2_in*65536)+(count3_in*16777216);
	return data;  //data has to be converted to float use this *(float*)&data
}

void ReadFlyerSettingsFromEeprom(void)
{
	long data_out = 0;
	fsp.spindleSpeed = EepromReadInt(FF_SPINDLE_SPEED_ADDR);
	data_out = EepromReadInt(FF_TENSION_DRAFT_ADDR);
	fsp.tensionDraft= ((float)data_out)/(float)100;
	data_out = EepromReadInt(FF_TPI_ADDR);
	fsp.tpi= ((float)data_out)/(float)100;
	data_out = EepromReadInt(FF_RTF_ADDR);
	fsp.rtf= ((float)data_out)/(float)100;
	fsp.lengthLimit = EepromReadInt(FF_LENGTH_LIMIT_ADDR);
	fsp.bobbinHeight = EepromReadInt(FF_BOBBIN_HEIGHT_ADDR);
	data_out = EepromReadInt(FF_ROVING_WIDTH_ADDR);
	fsp.rovingWidth = ((float)data_out)/(float)100;
	data_out = EepromReadInt(FF_DELTA_BOBBIN_DIA_ADDR);
	fsp.deltaBobbinDia = ((float)data_out)/(float)100;
	fsp.bareBobbinDia = EepromReadInt(FF_BARE_BOBBIN_DIA_ADDR);	
	fsp.boostFactor = EepromReadInt(FF_BOOSTFACTOR_ADDR);	
	fsp.buckFactor = EepromReadInt(FF_BUCKFACTOR_ADDR);	
}

void WriteFlyerSettingsIntoEeprom(void)
{

	EepromWriteInt(FF_SPINDLE_SPEED_ADDR,fsp.spindleSpeed);
	EepromWriteInt(FF_TENSION_DRAFT_ADDR,(int)(fsp.tensionDraft*100));
	EepromWriteInt(FF_TPI_ADDR,(int)(fsp.tpi*100));
	EepromWriteInt(FF_RTF_ADDR,(int)(fsp.rtf*100));
	EepromWriteInt(FF_LENGTH_LIMIT_ADDR,fsp.lengthLimit);
	EepromWriteInt(FF_BOBBIN_HEIGHT_ADDR,fsp.bobbinHeight);
	EepromWriteInt(FF_ROVING_WIDTH_ADDR,(int)(fsp.rovingWidth*100));
	EepromWriteInt(FF_DELTA_BOBBIN_DIA_ADDR,(int)(fsp.deltaBobbinDia*100));
	EepromWriteInt(FF_BARE_BOBBIN_DIA_ADDR,fsp.bareBobbinDia);
	EepromWriteInt(FF_BOOSTFACTOR_ADDR,fsp.boostFactor);
	EepromWriteInt(FF_BUCKFACTOR_ADDR,fsp.buckFactor);
}

void WritePIDSettingsIntoEeprom(int motorIndex)
{
	EepromWriteInt(EEPROM_KI_ADDRESSES[motorIndex],(int)(M[motorIndex].Ki*100));
	EepromWriteInt(EEPROM_KP_ADDRESSES[motorIndex],(int)(M[motorIndex].Kp*100));
	EepromWriteInt(EEPROM_START_OFFSET_ADDRESSES[motorIndex],M[motorIndex].startOffsetOrig);
	EepromWriteInt(EEPROM_FEEDFORWARD_ADDRESSES[motorIndex],M[motorIndex].ff_multiplier* 100);
}

int CheckPIDSettings(void){
	for (int i=1;i<6;i++){
		if (M[i].Ki > 5.0f){
			return 0;
		}
		if (M[i].Kp > 5.0f){
			return 0;
		}
		if (M[i].startOffsetOrig > 700){
			return 0;
		}
		if (M[i].ff_multiplier > 5){
			return 0;
		}
	}
	return 1;
}


int CheckSettingsReadings(void){
		//typically when something goes wrong with the eeprom you get a value that is very high..
		//to allow for changes place to place without changing this code, we just set the thresholds to  2* maxRange.
		// dont expect in any place the nos to go higher than this..NEED TO PUT LOWER BOUNDS FOR EVERYTHING
		if ((fsp.spindleSpeed > 1500 ) || (fsp.spindleSpeed < 10)){
			return 0;
		}
		if (fsp.tensionDraft > 16.0f){
			return 0;
		}
		if (fsp.tpi > 15.0f){
			return 0;
		}
		if (fsp.rtf > 4.0f){
			return 0;
		}
		if ((fsp.lengthLimit > 3500) || (fsp.spindleSpeed < 100)){
			return 0;
		}
		if (fsp.bobbinHeight > 500){
			return 0;
		}
		if (fsp.rovingWidth > 5){
			return 0;
		}
		if (fsp.deltaBobbinDia > 5){
			return 0;
		}
		if ((fsp.bareBobbinDia > 100) || (fsp.spindleSpeed < 10)){
			return 0;
		}
	return 1;
}


void ReadPIDSettingsFromEeprom(int motorIndex){
	long data_out = 0;
	data_out= EepromReadInt(EEPROM_KI_ADDRESSES[motorIndex]);
	M[motorIndex].Ki = ((float)data_out)/(float)100;

	data_out= EepromReadInt(EEPROM_KP_ADDRESSES[motorIndex]);
	M[motorIndex].Kp = ((float)data_out)/(float)100;

	M[motorIndex].startOffsetOrig = EepromReadInt(EEPROM_START_OFFSET_ADDRESSES[motorIndex]);

	data_out= EepromReadInt(EEPROM_FEEDFORWARD_ADDRESSES[motorIndex]);
	M[motorIndex].ff_multiplier = ((float)data_out)/(float)100;
}

long data_out = 0;
void ReadFF_AdvSettingsFromEeprom(void)
{
	data_out= EepromReadInt(FF_RAMPUP_RTF_MULTIPLIER);
	FFs.rampUpRtfMultiplier = ((float)data_out)/(float)100;

	data_out= EepromReadInt(FF_RAMPDOWN_RTF_MULTIPLIER);
	FFs.rampDownRtfMultiplier = ((float)data_out)/(float)100;
	
	FFs.rampUpRate = EepromReadInt(FF_RAMPUP_RATE);
	FFs.rampDownRate = EepromReadInt(FF_RAMPDOWN_RATE);
	
}

int CheckFF_AdvSettings(void){
		if (FFs.rampUpRtfMultiplier > 1.5f){
			return 0;
		}
		if (FFs.rampDownRtfMultiplier > 1.5f){
			return 0;
		}
		if (FFs.rampUpRate > 100){
			return 0;
		}
		if (FFs.rampDownRate > 100){
			return 0;
		}
	return 1;
}

void WriteDefaultFFAdvSettingsIntoEepromAndStruct(void){
	FFs.rampUpRtfMultiplier = 1.0;
	FFs.rampDownRtfMultiplier = 1.0;
	FFs.rampUpRate = 10;
	FFs.rampDownRate = 15;
	WriteFF_RampRTFMultiplierSettingsIntoEeprom();
	WriteFF_RampRateSettingsIntoEeprom();
}

void WriteFF_RampRTFMultiplierSettingsIntoEeprom(void){
	EepromWriteInt(FF_RAMPUP_RTF_MULTIPLIER,(int)(FFs.rampUpRtfMultiplier*100));
	EepromWriteInt(FF_RAMPDOWN_RTF_MULTIPLIER,(int)(FFs.rampDownRtfMultiplier*100));

}

void WriteFF_RampRateSettingsIntoEeprom(void){
	EepromWriteInt(FF_RAMPUP_RATE,FFs.rampUpRate);
	EepromWriteInt(FF_RAMPDOWN_RATE,FFs.rampDownRate);
}
	


void WriteAllPIDSettingsIntoEeprom(void){
	for (int i=1;i<6;i++){
		WritePIDSettingsIntoEeprom(i);
	}
}

void ReadAllPIDSettingsFromEeprom(void){
	for (int i=1;i< 6;i++){
		ReadPIDSettingsFromEeprom(i);
	}
}

void WriteDefaultPIDSettingsIntoEepromAndStruct(void){
	for (int i=1;i<6;i++){
		M[i].Kp = 0.2;
		M[i].Ki = 0.01;
		M[i].startOffsetOrig = 50;
		M[i].ff_multiplier = 0.5;
		WritePIDSettingsIntoEeprom(i);
	}
}


void LoadFactoryDefaultSettings(void){
	fsp.spindleSpeed = DEFAULT_SPINDLESPEED;
	fsp.tensionDraft = DEFAULT_DRAFT;
	fsp.tpi = DEFAULT_TPI;
	fsp.rtf = DEFAULT_RTF;
	fsp.lengthLimit = DEFAULT_ROVINGLAYERS;
	fsp.bobbinHeight = DEFAULT_MAXCONTENT_HEIGHT;
	fsp.rovingWidth = DEFAULT_ROVINGWIDTH;
	fsp.deltaBobbinDia = DEFAULT_DELTA_BOBBIN_DIA;
	fsp.bareBobbinDia = DEFAULT_BARE_BOBBIN_DIA;
	fsp.boostFactor = 10;
	fsp.buckFactor = 10;
}
