/*
******************************************************************************
**  CarMaker - Version 9.0.2
**
**  Copyright (C)   IPG Automotive Korea
******************************************************************************
*/

#ifndef TRAFFIC_LIGHT_UDP_H__
#define TRAFFIC_LIGHT_UDP_H__

#include <Environment.h>
#include <CarMaker.h>
#include <Vehicle.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tTrafficLight_UDP {
	int Socket;
	int CycleTime;
} tTrafficLight_UDP;

extern tTrafficLight_UDP TrafficLight_UDP;

#pragma pack (push, 1)
typedef struct tTrafficLight_UDP_Send {
		unsigned char MsgID; // 0x54
		char TrafficLightState;   
		double TrafficLightDist; // 0: All off, 1: Green, 2: Yellow, 3: Red 
} tTrafficLight_UDP_Send; // [m]
#pragma pack(pop)

extern tTrafficLight_UDP_Send TrafficLight_UDP_Send;

int TrafficLight_UDP_User_TestRun_atEnd (void);
void TrafficLight_UDP_User_Out (const unsigned CycleNo);
int TrafficLight_UDP_TestRun_End (void);
void TrafficLight_UDP_Cleanup (void);

#ifdef __cplusplus
}
#endif

#endif	/* #ifndef TRAFFIC_LIGHT_UDP_H__ */

