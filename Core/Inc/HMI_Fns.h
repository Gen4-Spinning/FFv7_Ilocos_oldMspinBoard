#ifndef __HMIFNS_H
#define __HMIFNS_H

#include "HMI_Constants.h"
#include "Structs.h"


struct HMI_InfoBase
{
    char start;
    char sender;
    char packetLength;
    char machineType;
    char machineID;
    char msgType;
    char nextscreen;
    char screen_substate;
    char attributeCount;
    char endChar;
};


struct HMI_RunPacket
{
		char tlv1Code;
    char tlv1length;
    int tlv1_val;
    char tlv2Code;
    char tlv2length;
    float tlv2_val;
    char tlv3Code;
    char tlv3length;
    float tlv3_val;
};

struct HMI_DiagPacket
{
    char tlv1Code;
    char tlv1length;
    int tlv1_val;
    char tlv2Code;
    char tlv2length;
    int tlv2_val;
    char tlv3Code;
    char tlv3length;
    int tlv3_val;
		char tlv4Code;
    char tlv4length;
    int tlv4_val;
};

struct HMI_StopPacket
{
    char tlv1Code;
    char tlv1length;
    int tlv1_val;
    char tlv2Code;
    char tlv2length;
    int tlv2_val;
    char tlv3Code;
    char tlv3length;
    int tlv3_val;
};



void Create_HMI_BasePacket(void);
void Create_HMI_Run_Packet(void);
void Create_HMI_StopPacket(void);
void Create_HMI_DiagPacket(void);

void UpdateBasePacket_Modes(char machineType,char msgType,char state,char runsubMode,char packetLength,char attributeCount);
void UpdateRunPacket_FF(char tlv1Code,int tlv1Val,char tlv2Code,float tlv2Val,char tlv3Code,int tlv3Val);
void UpdateStopPacket(char tlv1Code,char tlv2Code,char tlv3Code);

int UpdateRunPacketString_FF(char *buffer,struct HMI_InfoBase hsb,struct HMI_RunPacket hrp,int layers,float rtf,float prod_total);
int UpdateStopPacketString(char *buffer,struct HMI_InfoBase hsb,struct HMI_StopPacket hsp,int stop_reason,int motor_fault,int error_val); // Continuously
int UpdatePausePacketString(char *buffer,struct HMI_InfoBase hsb,struct HMI_StopPacket hsp,int stop_reason,int motor_fault,float error_val); //differnet fn bcos 00 needs to be sent as 00000000
int UpdateDiagPacketString(char *buffer,struct HMI_InfoBase hsb,struct HMI_DiagPacket hdp,int typeOfTest,int motorID,int signalVoltage,int rpm);
int HMI_GetFlyerSettingsAllPacketString(char *buffer,struct HMI_InfoBase hsb,struct FlyerSettings fsp);
int HMI_GetIdlePacketString(char *buffer,struct HMI_InfoBase hsb);
int HMI_GetSettingsACKPacketString(char *buffer,struct HMI_InfoBase hsb);
int HMI_Get_DiagOver_PacketString(char *buffer,struct HMI_InfoBase hsb,char status);
char HMI_BasePacket_Decode(char *receiveBuffer);
int HMI_PIDSettingsPacket(char *buffer,struct HMI_InfoBase hsb,int motorIndex);

//helper fns
float stringToFLOAT(char c[8]);
unsigned int stringToINT(char c[4]);
unsigned char charToHexDigit(char c);
unsigned int StringToHex(char c[2]);
unsigned int StringDecodeAsInteger(char c[4]);

extern struct HMI_InfoBase hsb;
extern struct HMI_RunPacket hrp;
extern struct HMI_StopPacket hsp;
extern struct HMI_DiagPacket hdp;
#endif

