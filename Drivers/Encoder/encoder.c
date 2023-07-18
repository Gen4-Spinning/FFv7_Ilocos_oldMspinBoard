
#include "encoder.h"
#include "logicDefines.h"
#include "functionDefines.h"

void UpdateRPM(encoders *enc){
	/* if not 12 readings, just take the raw reading as the averaged reading and keep summing the count.
	 * else the subtract the average from the summed count, and then add the new value for a new summed count
	 * and then divide by 12 for averaged count for the next reading.
	 */
	if ((enc->averagingIDX < 12) && (enc->averagingStarted == 0)){
		enc->flyer.averagedCount = enc->flyer.rawCount;
		enc->flyer.summedCount += enc->flyer.rawCount;

		enc->bobbin.averagedCount = enc->bobbin.rawCount;
		enc->bobbin.summedCount += enc->bobbin.rawCount;

		enc->lift.averagedCount = enc->lift.rawCount;
		enc->lift.summedCount += enc->lift.rawCount;

		enc->averagingStarted = 1;
	}else{
		enc->flyer.summedCount = enc->flyer.summedCount  - enc->flyer.averagedCount + enc->flyer.rawCount;
		enc->flyer.averagedCount = enc->flyer.summedCount/12;

		enc->bobbin.summedCount = enc->bobbin.summedCount  - enc->bobbin.averagedCount + enc->bobbin.rawCount;
		enc->bobbin.averagedCount = enc->bobbin.summedCount/12;

		enc->lift.summedCount = enc->lift.summedCount  - enc->lift.averagedCount + enc->lift.rawCount;
		enc->lift.averagedCount = enc->lift.summedCount/12;
	}

	enc->averagingIDX ++;


	enc->flyer.RPM = (uint16_t)(enc->flyer.averagedCount * TO_RPM_50MS_768_PPR);
	enc->flyer.rawCount = 0;

	enc->bobbin.RPM = (uint16_t)(enc->bobbin.averagedCount* TO_RPM_50MS_768_PPR);
	enc->bobbin.rawCount = 0;

	enc->lift.RPM = (uint16_t)(enc->lift.averagedCount* TO_RPM_50MS_768_PPR);
	enc->lift.rawCount = 0;

}

void UpdateRPM_Nuvoton_48PPR(encoders *enc){
	encs.flyer.RPM = encs.flyer.rawCount*TO_RPM_100MS_48_PPR;
	encs.bobbin.RPM = encs.bobbin.rawCount*TO_RPM_100MS_48_PPR;
	encs.lift.RPM = encs.lift.rawCount*TO_RPM_100MS_48_PPR;
	encs.flyer.rawCount = 0;
	encs.bobbin.rawCount = 0;
	encs.lift.rawCount = 0;
}

void UpdateRPM_Tachometric(encoders *enc){
	enc->FR.RPM = (uint16_t)(enc->FR.rawCount * TO_RPM_100MS_8_PPR);
	enc->FR.rawCount = 0;

	enc->BR.RPM = (uint16_t)(enc->BR.rawCount * TO_RPM_100MS_8_PPR);
	enc->BR.rawCount = 0;
}


uint16_t GetRPM(encoders *enc,uint8_t index){
	if (index == MOTOR1){
		return ((uint16_t)enc->flyer.RPM);
	}
	if (index == MOTOR2){
		return ((uint16_t)enc->bobbin.RPM);
	}
	if (index == MOTOR3){
		return ((uint16_t)enc->FR.RPM);
	}
	if (index == MOTOR4){
		return ((uint16_t)enc->BR.RPM);
	}
	if (index == MOTOR5){
		return ((uint16_t)enc->lift.RPM);
	}
	return 0;
}


void ResetEncoderVariables(encoders *enc){
	enc->averagingIDX = 0;
	enc->averagingStarted = 0;

	enc->flyer.summedCount = 0;
	enc->flyer.averagedCount = 0;
	enc->flyer.RPM = 0;
	enc->flyer.rawCount = 0;

	enc->bobbin.summedCount = 0;
	enc->bobbin.averagedCount = 0;
	enc->bobbin.RPM = 0;
	enc->bobbin.rawCount = 0;

	enc->lift.summedCount = 0;
	enc->lift.averagedCount = 0;
	enc->lift.RPM = 0;
	enc->lift.rawCount = 0;

	enc->FR.RPM = 0;
	enc->FR.rawCount = 0;

	enc->BR.RPM = 0;
	enc->BR.rawCount = 0;

}













