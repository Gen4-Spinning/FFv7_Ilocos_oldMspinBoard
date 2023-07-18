#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "Structs.h"
#include "Logger.h"
#include "stdio.h"
#include "encoder.h"

extern UART_HandleTypeDef huart1;

char out[100] ;
int logCounter = 0;

int LogInt(int integer,int bufferIndex){
	out[bufferIndex] = integer>>8;
	out[bufferIndex+1] = integer ;	
	return bufferIndex+2;
}

int LogChar(char val,int bufferIndex){
	out[bufferIndex] = val;
	return bufferIndex+1;
}



int EndLine(int bufferIndex ){
	out[bufferIndex] = 0x0A;
	out[bufferIndex+1] = '\r';
	out[bufferIndex+2] = '\r';

	return bufferIndex+3;
}

/*
void LogMotorVariables2(void){
		//size of output is 2 *no of variables 
		int bufferIndex = 0;
		bufferIndex = LogInt(65533,bufferIndex); //StartIndicator
		bufferIndex = LogInt(logCounter,bufferIndex);
	
		bufferIndex = LogInt(M[1].intTarget,bufferIndex);
		bufferIndex = LogInt(filter1,bufferIndex);
		bufferIndex = LogInt(M[1].pwm,bufferIndex);
	
		bufferIndex = LogInt(M[2].intTarget,bufferIndex);
	  bufferIndex = LogInt(filter2,bufferIndex);
		bufferIndex = LogInt(M[2].pwm,bufferIndex);
	
		bufferIndex = LogInt(filter3,bufferIndex);
		bufferIndex = LogInt(M[3].pwm,bufferIndex);
	
		bufferIndex = LogInt(filter4,bufferIndex);
		bufferIndex = LogInt(M[4].pwm,bufferIndex);
	
		bufferIndex = LogInt(filter5,bufferIndex);
		bufferIndex = LogInt(M[5].pwm,bufferIndex);
		
		bufferIndex = LogInt(filter6,bufferIndex);
		bufferIndex = LogInt(M[6].pwm,bufferIndex);

		//other variables
		bufferIndex = LogChar(S.current_state,bufferIndex);
		bufferIndex = LogChar(S.errStopReasonHMI,bufferIndex);
		bufferIndex = LogChar(leftSensorStatus,bufferIndex);
		bufferIndex = LogChar(rightSensorStatus,bufferIndex);
		bufferIndex = LogInt((int)(deltaDistance*100),bufferIndex);
		//bufferIndex = EndLine(bufferIndex);
		HAL_UART_Transmit(&huart1,(uint8_t*)out,bufferIndex,100);
}

void LogMotorVariables1(void){
		//size of output is 2 *no of variables 
		int bufferIndex = 0;
		bufferIndex = LogInt(65533,bufferIndex); //StartIndicator
		bufferIndex = LogInt(logCounter,bufferIndex);
	
		bufferIndex = LogInt(filter1,bufferIndex);
	  bufferIndex = LogInt(filter2,bufferIndex);
		bufferIndex = LogInt(filter3,bufferIndex);
		bufferIndex = LogInt(filter4,bufferIndex);
		bufferIndex = LogInt(filter5,bufferIndex);
		bufferIndex = LogInt(filter6,bufferIndex);
		for (int i=1;i<7;i++){
				bufferIndex = LogInt(M[i].intTarget,bufferIndex);
				bufferIndex = LogInt(M[i].pwm,bufferIndex);
			}

		//other variables
		bufferIndex = LogChar(S.current_state,bufferIndex);
		bufferIndex = LogChar(S.errStopReasonHMI,bufferIndex);
		bufferIndex = LogChar(leftSensorStatus,bufferIndex);
		bufferIndex = LogChar(rightSensorStatus,bufferIndex);
		bufferIndex = LogInt((int)(deltaDistance*100),bufferIndex);
	  bufferIndex = EndLine(bufferIndex);
		HAL_UART_Transmit(&huart1,(uint8_t*)out,bufferIndex,100);
}

void LogMotorVariables(void){
		//size of output is 2 *no of variables 
		int bufferIndex = 0;
		bufferIndex = LogInt(65533,bufferIndex); //StartIndicator
		bufferIndex = LogInt(filter1,bufferIndex);
	  bufferIndex = LogInt(filter2,bufferIndex);
		bufferIndex = LogInt(filter3,bufferIndex);
		bufferIndex = LogInt(filter4,bufferIndex);
		bufferIndex = LogInt(filter5,bufferIndex);
		bufferIndex = LogInt(filter6,bufferIndex);
		for (int i=1;i<7;i++){
				bufferIndex = LogInt(M[i].intTarget,bufferIndex);
				bufferIndex = LogInt(M[i].pwm,bufferIndex);
			}
		//other variables
		bufferIndex = LogInt(S.current_state,bufferIndex);
		bufferIndex = LogInt(S.errStopReasonHMI,bufferIndex);
		bufferIndex = LogInt((int)leftSensorStatus,bufferIndex);
		bufferIndex = LogInt((int)rightSensorStatus,bufferIndex);
		bufferIndex = LogInt((int)(deltaDistance*100),bufferIndex);
		bufferIndex = EndLine(bufferIndex);
		HAL_UART_Transmit(&huart1,(uint8_t*)out,bufferIndex,100);

}
*/

void LogMotorVariables4(void){
		//size of output is 2 *no of variables 	
		uint8_t deltaDistance = 0;
		sprintf(out,"%02X%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%02d%02d%02d%02d%04d%02X\r",170,logCounter,M[1].intTarget,
		M[1].presentRpm,M[1].pwm,M[2].intTarget,M[2].presentRpm,M[2].pwm,M[3].intTarget,M[3].presentRpm,M[3].pwm,
		M[4].intTarget,M[4].presentRpm,M[4].pwm,M[5].intTarget,M[5].presentRpm,M[5].pwm,M[1].feedforward,M[1].calcPwm,M[2].feedforward,M[2].calcPwm,S.current_state,
		S.errStopReasonHMI,0,0,FFs.logRTFMultiplier,255);
		HAL_UART_Transmit(&huart1,(uint8_t*)out,97,100);
}
void LogSettings(void){
		//size of output is 2 *no of variables CC
		sprintf(out,"%02X%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%02X\r",204,
		fsp.spindleSpeed,(int)(fsp.tensionDraft*100),(int)(fsp.tpi*100),(int)(fsp.rtf*100),fsp.lengthLimit,fsp.bobbinHeight,
		(int)(fsp.rovingWidth*100),(int)(fsp.deltaBobbinDia*100),fsp.bareBobbinDia,(int)(FFs.rampUpRtfMultiplier*100),(int)(FFs.rampDownRtfMultiplier*100),
		(int)(FFs.rampUpRate),(int)(FFs.rampDownRate),255);
		HAL_UART_Transmit(&huart1,(uint8_t*)out,57,100);
	}
	 
/*void LogSettings(void){
		//size of output is 2 *no of variables CC
	
		sprintf(out,"%02X%04d%04d%04d%04d%04d%04d%04d%04d%04d%02X\r",204,
		fsp.spindleSpeed,(int)(fsp.tensionDraft*100),(int)(fsp.tpi*100),(int)(fsp.rtf*100),fsp.lengthLimit,fsp.bobbinHeight,
		(int)(fsp.rovingWidth*100),(int)(fsp.deltaBobbinDia*100),fsp.bareBobbinDia,255);
		HAL_UART_Transmit(&huart1,(uint8_t*)out,41,100);
		int bufferIndex = 0;
		bufferIndex = LogInt(65534,bufferIndex); //StartIndicator
		bufferIndex = LogInt(fsp.spindleSpeed ,bufferIndex);
	  bufferIndex = LogInt((int)(fsp.tensionDraft*100),bufferIndex);
		bufferIndex = LogInt((int)(fsp.tpi*100),bufferIndex);
		bufferIndex = LogInt((int)(fsp.rtf*100),bufferIndex);
		bufferIndex = LogInt(fsp.lengthLimit,bufferIndex);
		bufferIndex = LogInt(fsp.bobbinHeight,bufferIndex);
		bufferIndex = LogInt((int)(fsp.rovingWidth*100),bufferIndex);
		bufferIndex = LogInt((int)(fsp.deltaBobbinDia*100),bufferIndex);
		bufferIndex = LogInt(fsp.bareBobbinDia,bufferIndex);
		bufferIndex = EndLine(bufferIndex);
		HAL_UART_Transmit(&huart1,(uint8_t*)out,bufferIndex,100);
}*/


void LogPIDVals(void){
		//size of output is 2 *no of variables BB 
		sprintf(out,"%02X%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%02X\r",187,
		(int)(M[1].Kp*100),(int)(M[1].Ki*100),M[1].startOffsetOrig,(int)(M[2].Kp*100),(int)(M[2].Ki*100),
		M[2].startOffsetOrig,(int)(M[3].Kp*100),(int)(M[3].Ki*100),M[3].startOffsetOrig,(int)(M[4].Kp*100),
		(int)(M[4].Ki*100),M[4].startOffsetOrig,(int)(M[5].Kp*100),(int)(M[5].Ki*100),M[5].startOffsetOrig,
		(int)(M[6].Kp*100),(int)(M[6].Ki*100),M[6].startOffsetOrig,255);
		HAL_UART_Transmit(&huart1,(uint8_t*)out,77,100);

	/*	int bufferIndex = 0;
		bufferIndex = LogInt(65535,bufferIndex); //StartIndicator
		for (int i=1;i<7;i++){
				bufferIndex = LogInt((int)(M[i].Kp*100),bufferIndex);
				bufferIndex = LogInt((int)(M[i].Ki*100),bufferIndex);
				bufferIndex = LogInt(M[i].startOffsetOrig,bufferIndex);
			}
		bufferIndex = EndLine(bufferIndex);
		HAL_UART_Transmit(&huart1,(uint8_t*)out,bufferIndex,100);*/
}



