#ifndef __LOGGER_H
#define __LOGGER_H

int LogInt(int integer,int bufferIndex);
int LogChar(char val,int bufferIndex);
void LogMotorVariables(void);
void LogMotorVariables1(void);
void LogMotorVariables2(void);
void LogMotorVariables4(void);
void LogSettings(void);
void LogPIDVals(void);
int EndLine(int bufferIndex );

#endif
