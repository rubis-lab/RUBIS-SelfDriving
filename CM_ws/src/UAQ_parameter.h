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
double Vh_roll;
int Lateral_flag;
int Collision_Traffic_Attribute;
int Collision_Traffic_ID;
int CollisionCnt;
double time_th;
int flag;
int count_tmp;
int ColliisionOutput;

void Name_DeclQuants	(void);
void Name_TestRun_Start_atend	(void);
void Name_Calc			(void);


#ifdef __cplusplus
}
#endif


