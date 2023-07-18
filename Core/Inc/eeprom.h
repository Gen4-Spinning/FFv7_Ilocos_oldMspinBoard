#ifndef __EEPROM_H
#define __EEPROM_H

#define EEPROM_ADDRESS 0xAC//left is 0xa8, right is 0xAC

#define FF_SPINDLE_SPEED_ADDR 0X02
#define FF_TENSION_DRAFT_ADDR 0X04
#define FF_TPI_ADDR 0X08
#define FF_RTF_ADDR 0X0C
#define FF_LENGTH_LIMIT_ADDR 0X10
#define FF_BOBBIN_HEIGHT_ADDR 0X12
#define FF_ROVING_WIDTH_ADDR 0X14
#define FF_DELTA_BOBBIN_DIA_ADDR 0X18
#define FF_BARE_BOBBIN_DIA_ADDR 0X1D
#define FF_BOOSTFACTOR_ADDR 0X20
#define FF_BUCKFACTOR_ADDR 0X22

//All INTS
#define FLYER_KP 0X30
#define FLYER_KI 0X32
#define FLYER_START_OFFSET 0X34
#define BOBBIN_KP 0X36
#define BOBBIN_KI 0X38
#define BOBBIN_START_OFFSET 0X3A
#define FR_ROLLER_KP 0X3C
#define FR_ROLLER_KI 0X40
#define FR_ROLLER_START_OFFSET 0X42
#define BK_ROLLER_KP 0X44
#define BK_ROLLER_KI 0X46
#define BK_ROLLER_START_OFFSET 0X048
#define LIFT_LEFT_KP 0X5A
#define LIFT_LEFT_KI 0X5C
#define LIFT_LEFT_START_OFFSET 0X60
#define FLYER_FF_CONST 0x62
#define BOBBIN_FF_CONST 0x64
#define LIFT_FF_CONST 0x66
#define FR_FF_CONST 0x68
#define BR_FF_CONST 0x70

#define FF_RAMPUP_RTF_MULTIPLIER 0X72 // all ints
#define FF_RAMPDOWN_RTF_MULTIPLIER 0X74

#define FF_RAMPUP_RATE 0X76
#define FF_RAMPDOWN_RATE 0X78

#define EXTRA  0x80

void EepromWriteInt(unsigned position,unsigned int data);
unsigned int EepromReadInt(unsigned position);
void EepromWriteFloat(unsigned position,float data);
unsigned int EepromReadFloat(unsigned position);

void WriteFlyerSettingsIntoEeprom(void);
void ReadFlyerSettingsFromEeprom(void);
//PID settings Fns
void ReadAllPIDSettingsFromEeprom(void);
void WriteAllPIDSettingsIntoEeprom(void);
void ReadPIDSettingsFromEeprom(int motorIndex);
void WritePIDSettingsIntoEeprom(int motorIndex);
int CheckSettingsReadings(void);
int CheckPIDSettings(void);
void WriteDefaultPIDSettingsIntoEepromAndStruct(void);
void LoadFactoryDefaultSettings(void);

void ReadFF_AdvSettingsFromEeprom(void);
void WriteFF_RampRTFMultiplierSettingsIntoEeprom(void);
void WriteFF_RampRateSettingsIntoEeprom(void);
int CheckFF_AdvSettings(void);
void WriteDefaultFFAdvSettingsIntoEepromAndStruct(void);

#endif /* __ENCODER_H */

