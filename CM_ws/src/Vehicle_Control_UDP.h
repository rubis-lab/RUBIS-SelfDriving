/*
******************************************************************************
**  CarMaker - Version 9.0.2
**
**  Copyright (C)   IPG Automotive Korea
******************************************************************************
*/

#ifndef _UDP_H__
#define _UDP_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tUDP_PC {
	unsigned short Port_In;
	char *Socket_In;
	int VC_SwitchOn;
	int GearNo;
} tUDP_PC;

extern tUDP_PC UDP_PC;

#pragma pack (push, 1)
typedef struct tUDP_Input {
	struct {
		unsigned char MsgID;
	} Header;
	
    struct {
		double SteeringWheel;   // rad
		double Ax; 				// [m/s^2]
		int GearNo; 			// 1: Driving, 0: Stop, -1: Back, -9: Parking
		char Light; 			// 0: Off, 1: Left indicator, 2: Right indicator, 3: Hazard
    } DriveCont;
} tUDP_Input;
#pragma pack(pop)

extern tUDP_Input UDP_Input;

extern tUDP_PC UDP_PC;

int Vehicle_Control_UDP_User_TestRun_atEnd (void);
void Vehicle_Control_UDP_User_DeclQuants (void);
int Vehicle_Control_UDP_User_TestRun_Start_Finalize (void);
void Vehicle_Control_UDP_User_In (const unsigned CycleNo);
int Vehicle_Control_UDP_User_VehicleControl_Calc (double dt);
int Vehicle_Control_UDP_User_Calc (void);
void Vehicle_Control_UDP_User_Out (const unsigned CycleNo);
int Vehicle_Control_UDP_User_TestRun_End_First (void);
int Vehicle_Control_UDP_TestRun_End (void);
void Vehicle_Control_UDP_Cleanup (void);


#ifdef __cplusplus
}
#endif

#endif	/* #ifndef _UDP_H__ */
