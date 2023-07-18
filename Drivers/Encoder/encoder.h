#ifndef __ENCODER_H
#define __ENCODER_H

#include "stdio.h"

#define TO_RPM_50MS_768_PPR 1.5625 // RPM = counts*(20*60/768)
#define TO_RPM_100MS_8_PPR 75   // RPM = counts*(10*60/8)
#define TO_RPM_100MS_48_PPR 12.5

typedef struct MotorEncoder{
	uint16_t rawCount;
	uint16_t averagedCount;
	uint16_t summedCount;
	uint16_t RPM;
}encMotor;

typedef struct enc{
	encMotor flyer;
	encMotor bobbin;
	encMotor lift;
	encMotor FR;
	encMotor BR;
	uint8_t averagingStarted;
	uint8_t averagingIDX;
}encoders;

extern encoders encs;


void UpdateRPM(encoders *enc);
uint16_t GetRPM(encoders *enc,uint8_t index);
void ResetEncoderVariables(encoders *enc);
void UpdateRPM_Tachometric(encoders *enc);
void UpdateRPM_Nuvoton_48PPR(encoders *enc);

#endif /* __ENCODER_H */

