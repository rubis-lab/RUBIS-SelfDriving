/*
******************************************************************************
**  CarMaker - Version 5.0.2
**  Vehicle Dynamic Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    http://www.ipg.de
******************************************************************************
**
**  CarMaker Interface - UDP communication Sample Server Side
**
******************************************************************************
*/

#include <Global.h>

#if defined(WIN32) && !defined(INTIME)
#  include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <CarMaker.h>
#include <Car/Vehicle_Car.h>

#include "Vehicle_Control_UDP.h"
#include <Vehicle/VehicleControlApps.h>
#include "sUDP.h"

tUDP_PC UDP_PC;
tUDP_Input UDP_Input;

int length_chk = 0;

double P_ax = 0;
double I_ax = 0;
double I_ax_integral = 0;

static void
Vehicle_Control_UDP_ReadFnc (void *buf, int len, unsigned short int srcPort)
{
	length_chk = len;
	//UDP 길이 체크
	if (len < sizeof(tUDP_Input))	return;
	
	//체크 버퍼 저장
	tUDP_Input UDP_Chk;
	memcpy(&UDP_Chk, buf, sizeof(UDP_Chk));

	//헤더 프로토콜 체크
	int Protocol_chk = 1;
	if(UDP_Chk.Header.MsgID != 0xFF) Protocol_chk = 0;						 
	
	//데이터 저장
	if( Protocol_chk == 1 )    memcpy(&UDP_Input, buf, sizeof(UDP_Input));
    return;
}


int Vehicle_Control_UDP_User_TestRun_atEnd (void)
{
	if (SimCore.State <= SCState_Start) {
	return 0;
    }
	
	int buf_chk;
	
    UDP_PC.Port_In = (unsigned short) iGetLongOpt(SimCore.Vhcl.Inf, "VC.UDPSocket", 60001);
	P_ax = iGetDblOpt(SimCore.Vhcl.Inf, "VC.R_P_Gain", 0.5);
	I_ax = iGetDblOpt(SimCore.Vhcl.Inf, "VC.R_I_Gain", 0.1);
	UDP_PC.VC_SwitchOn = iGetDblOpt(SimCore.Vhcl.Inf, "VC.Switch", 0);
	
	// Create Input socket
    if( (buf_chk = UDP_New_InSocket(UDP_PC.Port_In, &Vehicle_Control_UDP_ReadFnc)) < 0)
	{
		LogErrF(EC_Init, "Can't open input port\n");
	}
	Log("\nVehicle control module:\n");
	Log("Port of UDP_InSocket of Vehicle control: %d\n",UDP_PC.Port_In);
	Log("Reverse Gain P/I: %.3f/%.3f\n",P_ax,I_ax);
	
	// Initialize Accel control parameter
	I_ax = I_ax/1000;
	I_ax_integral = 0;
	
	// Initialize UDP parameter
	UDP_Input.Header.MsgID 				= 0;
	UDP_Input.DriveCont.SteeringWheel 	= 0;
	UDP_Input.DriveCont.Ax 				= 0;
	UDP_Input.DriveCont.GearNo 			= 1;
	UDP_Input.DriveCont.Light 			= 0;
	
    return 0;
}


void Vehicle_Control_UDP_User_DeclQuants (void)
{
    /* Communication */
    DDefDouble(NULL, "UDP.WheelAng", "rad", &UDP_Input.DriveCont.SteeringWheel, DVA_IO_In);
    DDefDouble(NULL, "UDP.Ax", "m/s^2", &UDP_Input.DriveCont.Ax, DVA_IO_In);
	DDefInt(NULL, "UDP.GearNo", "", &UDP_Input.DriveCont.GearNo, DVA_IO_In);
	DDefChar(NULL, "UDP.Light", "", &UDP_Input.DriveCont.Light, DVA_IO_In);
    DDefInt(NULL, "UDP.VC_SwitchOn", "", &UDP_PC.VC_SwitchOn, DVA_IO_In);
    DDefInt(NULL, "UDP.Length", "", &length_chk, DVA_None);
}

int Vehicle_Control_UDP_User_TestRun_Start_Finalize (void)
{
	
    return 0;
}

void Vehicle_Control_UDP_User_In (const unsigned CycleNo)
{
    /*** Handle Write Access (first action after IO_In() */
	UDP_In();
}

int Vehicle_Control_UDP_User_VehicleControl_Calc (double dt)
{
	double E_ax;
	double Cont_ax;
	
    if (SimCore.State != SCState_Simulate) return 0;
	
	if(UDP_PC.VC_SwitchOn == 1)
	{
		// Error
		VehicleControl.Steering.Ang = UDP_Input.DriveCont.SteeringWheel;  	// (rad)
		VehicleControl.SelectorCtrl = UDP_Input.DriveCont.GearNo;
		
		// PI Control
		E_ax = UDP_Input.DriveCont.Ax - Car.ConBdy1.a_1[0];
		I_ax_integral += I_ax * E_ax;
		if(I_ax_integral > 1.0) 		I_ax_integral = 1.0;
		else if(I_ax_integral < -1.0) I_ax_integral = -1.0;
		
		Cont_ax = P_ax * E_ax + I_ax_integral;
		
		// GearNo
		if( UDP_Input.DriveCont.GearNo == 1)
		{
			AccelCtrl.DesrAx = UDP_Input.DriveCont.Ax;
		}
		
		else if( UDP_Input.DriveCont.GearNo == -1)
		{
			AccelCtrl.DesrAx = -99999;
			if(Cont_ax < 0)
			{
				VehicleControl.Gas = -Cont_ax;
				if(Cont_ax < -1) 	VehicleControl.Gas = 1;
				VehicleControl.Brake = 0;
			}
			else if(Cont_ax >= 0)
			{
				VehicleControl.Brake = Cont_ax;
				if(Cont_ax > 1) 	VehicleControl.Brake = 1;
				VehicleControl.Gas = 0;
			}
		}
		
		else if( UDP_Input.DriveCont.GearNo == 0)
		{
			AccelCtrl.DesrAx = -99999;
			VehicleControl.Gas = 0;
			VehicleControl.Brake = 0.5;
		}
		
		else if( UDP_Input.DriveCont.GearNo == -9)
		{
			AccelCtrl.DesrAx = -99999;
			VehicleControl.Gas = 0;
			VehicleControl.Brake = 1;
		}
		
		// Light
		if( UDP_Input.DriveCont.Light == 1 )
		{
			DrivMan.Lights.Indicator = 1;
			DrivMan.Lights.Hazard = 0;
		}
		else if( UDP_Input.DriveCont.Light == 2 )
		{
			DrivMan.Lights.Indicator = -1;
			DrivMan.Lights.Hazard = 0;
		}
		else if( UDP_Input.DriveCont.Light == 3 )
		{
			DrivMan.Lights.Indicator = 0;
			DrivMan.Lights.Hazard = 1;
		}
		else
		{
			DrivMan.Lights.Indicator = 0;
			DrivMan.Lights.Hazard = 0;
		}

	}
	
	return 0;
}

int Vehicle_Control_UDP_User_Calc (void)
{
	
    return 0;
}


void
Vehicle_Control_UDP_User_Out (const unsigned CycleNo)
{

}

int Vehicle_Control_UDP_User_TestRun_End_First (void)
{
	return 0;
}

/*
** User_End ()
**
** End all models of the user module
**
** Call:
** - one times at end of program
** - no realtime conditions
*/

int Vehicle_Control_UDP_TestRun_End (void)
{
    UDP_Delete();
    return 0;
}

void Vehicle_Control_UDP_Cleanup (void)
{
    UDP_Cleanup();
    return;
}

