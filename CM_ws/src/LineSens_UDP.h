 /*
******************************************************************************
**  Description : Line simulation model
**
**  Copyright (C)   IPG Automotive Korea
******************************************************************************
*/
#ifndef LINESIMMODEL_H_
#define LINESIMMODEL_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <CarMaker.h>
#include <Car/Car.h>
#include <Vehicle/Sensor_Line.h>
#include "sUDP.h"

#define LINE_MAX_SCANPOINT_SIZE 100

#pragma pack(push, 1)
typedef struct Line_Data_point {
    double x; // [m]
	double y; 		  
    double z;	
}tLine_Data_point;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct Line_SendData {
    uint64_t Timestamp;
	int Npoint_L0;
    tLine_Data_point DataPoint_L0[LINE_MAX_SCANPOINT_SIZE];
	int Npoint_L1;
    tLine_Data_point DataPoint_L1[LINE_MAX_SCANPOINT_SIZE];
	int Npoint_R0;
    tLine_Data_point DataPoint_R0[LINE_MAX_SCANPOINT_SIZE];
	int Npoint_R1;
    tLine_Data_point DataPoint_R1[LINE_MAX_SCANPOINT_SIZE];
	unsigned short Footer;
}tLine_SendData;
#pragma pack(pop)

typedef struct LineRSI_Param {
    int	UpdRate;
	int sensorCycleTime;
    int sensorCycleOffset;
}tLine_Param;

typedef struct LineRSI_SensorUnit {
	char SensorValid;
    int socketOut;
	
    tLine_Param SensorParam;
	tLine_SendData Line_SendData;
	tLineSensor *CM_LineSens;
}tLineRSI_SensorUnit;

typedef struct Line_Config {
    int nSensors;
	uint64_t Timestamp_Init;
	double Noise; // [m]
    tLineRSI_SensorUnit* LineSensor;
} tLine_Config;

extern tLine_Config LineConfig;

int Line_Config_TestRun_Start_atEnd(void);
int Line_Config_Out(const unsigned CycleNo);
int Line_Config_TestRun_End(void);
int Line_Config_Cleanup (void);


#endif /* LINESIMMODEL_H_ */
