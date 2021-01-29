/*
******************************************************************************
**  CarMaker - Version 9.0.2
**
**  Copyright (C)   IPG Automotive Korea
******************************************************************************
*/

#ifndef VEHICLE_INFO_UDP_H__
#define VEHICLE_INFO_UDP_H__

#include <Vehicle/Sensor_GNav.h>
#include <Vehicle/Sensor_Inertial.h>

#ifdef __cplusplus
extern "C" {
#endif

#define m2UTM 0.0000155

typedef struct tVehicleInfo_UDP {
	int Socket;
	int CycleTime_VehInf;
	int CycleTime_GNSS;
	int Error_GNSS; // [m]
} tVehicleInfo_UDP;

extern tVehicleInfo_UDP VehicleInfo_UDP;

#pragma pack (push, 1)
typedef struct tVehicleInfo_UDP_Send {
	unsigned char MsgID; // 0x56
	double Roll; // [deg]  
	double Pitch; //  
	double Yaw;
	double Vx;
	double Vy;
	double Vz;
	double RollVel;
	double PitchVel;
	double YawVel;
	double Ax;
	double Ay;
	double Az;
	double RollAcc;
	double PitchAcc;
	double YawAcc;
	double SteerAng;
	double AccPedal;
	double BrakePedal;
	double GearNum;
	double S_Vx;
	double S_Vy;
	double S_Vz;
	double S_Roll;
	double S_Pitch;
	double S_Yaw;
	double S_Ax;
	double S_Ay;
	double S_Az;
	double S_RollAcc;
	double S_PitchAcc;
	double S_YawAcc;
} tVehicleInfo_UDP_Send;
#pragma pack(pop)

extern tVehicleInfo_UDP_Send VehicleInfo_UDP_Send;
	
#pragma pack (push, 1)

typedef struct tGNSS_UDP_Send {
	unsigned char MsgID; // 0x47
	double Long;
	double Lat;
	double Elev;
} tGNSS_UDP_Send;
#pragma pack(pop)

extern tGNSS_UDP_Send GNSS_UDP_Send;



int VehicleInfo_UDP_User_TestRun_atEnd (void);
void VehicleInfo_UDP_User_Out (const unsigned CycleNo);
int VehicleInfo_UDP_TestRun_End (void);
void VehicleInfo_UDP_Cleanup (void);

#ifdef __cplusplus
}
#endif

#endif	/* #ifndef VEHICLE_INFO_UDP_H__ */

