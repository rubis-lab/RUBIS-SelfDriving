 /*
******************************************************************************
**  Description : Traffic Sign sensor simulation model
**
**  Copyright (C)   IPG Automotive Korea
******************************************************************************
*/
#ifndef TSINGSIMMODEL_H_
#define TSINGSIMMODEL_H_

#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#include <CarMaker.h>
#include <Car/Car.h>
#include <Vehicle/Sensor_TSign.h>
#include "sUDP.h"

#pragma pack(push, 1)
typedef struct tTSign_SendData {
    unsigned char MsgID; // 0x53
	double Distance; 	// [m]
	double Value; 		// [km/h]
}tTSign_SendData;
#pragma pack(pop)

typedef struct tTSign_Param {
    int	UpdRate;
	int sensorCycleTime;
    int sensorCycleOffset;
}tTSign_Param;

typedef struct tTSign_SensorUnit {
	char SensorValid;
    int socketOut;
	
    tTSign_Param SensorParam;
	tTSign_SendData TSign_SendData;
	tTSignSensor *CM_TSignSens;
}tTSign_SensorUnit;

typedef struct tTSign_Config {
    int nSensors;
    tTSign_SensorUnit* TSignSensor;
} tTSign_Config;

extern tTSign_Config TSign_Config;

int TSign_Config_TestRun_Start_atEnd(void);
int TSign_Config_Out(const unsigned CycleNo);
int TSign_Config_TestRun_End(void);
int TSign_Config_Cleanup (void);


#endif /* TSINGSIMMODEL_H_ */
