/**
 * Description : Lidar simulation model
 * Lidar model : Velarray A1D
 *
 *
 * COPYRIGHT 2020 HYUNDAI MOTOR GROUP, ALL RIGHTS RESERVED.
 */
#ifndef LIDARSIMMODEL_H_
#define LIDARSIMMODEL_H_

#include <stdint.h>

#include <CarMaker.h>
#include <Car/Car.h>
#include <Vehicle/Sensor_LidarRSI.h>
#include "sUDP.h"

#define LIDAR_MAX_SCANPOINT_SIZE 70000
#define LIDAR_MAX_SENDPOINT_SIZE 4000

#define VLP_16_NUMBER_OF_POINTS 28800
#define VLP_16_POINTS_OF_PACKET 2880
#define VLP_16_NUMBER_OF_PACKET 10

#define OS1_64_NUMBER_OF_POINTS 65536
#define OS1_64_POINTS_OF_PACKET 2900
#define OS1_64_POINTS_OF_LAST_PACKET 1736
#define OS1_64_NUMBER_OF_PACKET 23
#define Max_Intensity 65000

#pragma pack(push, 1)
typedef struct Data_point {
    unsigned char 	LayerIndex;
	unsigned short 	Angle; 	//(Min : 0, Max : 35999) 0.01 deg	  
    unsigned short 	Range;	// 0.01 m
    unsigned short	Intensity; // nW
}tData_point;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct LidarRSI_SendData {
    uint64_t Timestamp;
	unsigned int Rotation_Count;
	unsigned char Frame_Count;
    tData_point SendDataPointInfo[LIDAR_MAX_SENDPOINT_SIZE];
	unsigned short Footer;
}tLidarRSI_SendData;
#pragma pack(pop)

typedef struct LidarRSI_ScanData {
    tData_point ScanDataPointInfo[LIDAR_MAX_SCANPOINT_SIZE];
}tLidarRSI_ScanData;

typedef struct LidarRSI_Param {
    tInfos *Inf_Sensor;
    char SensorBeamFile[255];
    int	sensorCycleTime;
    int sensorCycleOffset;
	int nTotal;
	double **Lidar_Beams;
}tLidarRSI_Param;

typedef struct LidarRSI_SensorUnit {
	char SensorKind[20];
	char SensorValid;
    int socketOut_Lidar;
	
    tLidarRSI_Param SensorParam;
	tLidarRSI_ScanData Lidar_ScanData;
	tLidarRSI_SendData Lidar_SendData;
	tLidarRSI *CM_LidarSens;
}tLidarRSI_SensorUnit;

typedef struct LidarRSI_Config {
    int nSensors;
	uint64_t Timestamp_Init;
    tLidarRSI_SensorUnit* LidarSensor;
} tLidarRSI_Config;

extern tLidarRSI_Config LidarConfig;

int LidarRSI_Config_TestRun_Start_atEnd(void);
int LidarRSI_Config_Out(const unsigned CycleNo);
int LidarRSI_Config_TestRun_End(void);
int LidarRSI_Config_Cleanup (void);

#endif /* LIDARSIMMODEL_H_ */