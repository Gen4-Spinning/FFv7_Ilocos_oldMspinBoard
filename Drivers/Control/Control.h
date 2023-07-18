/*
 * Control.h
 *
 *  Created on: Jul 9, 2023
 *      Author: harsha
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include "Structs.h"
#include "stdio.h"
#include "encoder.h"
#include "logicDefines.h"
#include "CommonConstants.h"

extern char MOTORARRAY[6];
extern char MOTORARRAY_HMI[6];

void updateTargets(uint8_t motorIndex,uint8_t rampUp);
void followTargets(uint8_t motorIndex);
void UpdateMotorVoltage(int motorIndex,int pwmValue);

void RaiseFlyerVirtualTarget(int virtualTarget);
#endif /* CONTROL_H_ */
