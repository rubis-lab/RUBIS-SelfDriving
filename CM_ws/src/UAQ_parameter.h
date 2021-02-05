/*
******************************************************************************
**
** iGLAD code classification
**
** 2020.04.24 by bel
**
******************************************************************************
*/


#include <Global.h>
#include <Vehicle/MBSUtils.h>
#include <Vehicle.h>

#ifdef __cplusplus
extern "C" {
#endif


int TL_id;
double TL_dist;

int TL_flag;
int TL_call;
int TL_Over_flag;
int TL_SpeedOver_flag;
int TL_Ign_flag;
int Lateral_flag;
int Collision_Traffic_Attribute;
int Collision_Traffic_ID;
int CollisionCnt;
int CollisionCnt_Old;
double time_th;
int flag;
int ColliisionOutput;

typedef struct tIGLAD_Data {
	int State;
	int State2;
	int State3;
	
	double onJunction;
	double onJunction1;
	double onJunction2;
		
	double Trigger1;
	double Trigger2;
	double Trigger3;

	int Link_Id;
	int Lane_Id;	
	double tMidLane_Old;
	
	int Link_Id1;
	int Lane_Id1;
	double tMidLane_Old1;
	
	double Junction_In_Yaw;
	double Junction_Out_Yaw;
	int Junction_In_Indicator[2];
	double JuncDist_In_Dist;
	int Junction_On;
	
	double Collision_Relative_Velocity;
	double Collision_Traffic_Vel[2];
	double Collision_Relative_Vel[2];

} tIGLAD_Data;

typedef struct tIGLAD {
	tIGLAD_Data Ego;
} tIGLAD;

extern tIGLAD IGLAD;

double tMidLane;
double tMidLane1;
double lanetype;
double lanetype1;
double lanetype_old;
double lanetype1_old;
double lanewidth;
double lanewidth1;

int lObjId;
int lsObjId;
int laneObjId; 	
int lObjId1;
int lsObjId1;
int laneObjId1;

void Name_DeclQuants (void);
void Name_TestRun_Start_atend (void);
void Name_Calc (void);

#ifdef __cplusplus
}
#endif


