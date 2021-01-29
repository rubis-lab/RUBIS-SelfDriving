1. CM -> create project

2. linux_v2 (차량 모델 배포)

3. Test_IPG (예제모델)

4. Line_Sensor (ROS 배포)

5. VDS (ROS 배포)

6. SanganRoad (3차 도로모델 배포)

//7. linux_v2에서 Vehicle_Control_UDP.c, Vehicle_Control_UDP.h 복사해서 붙여넣기(?)
Light가 linux에만 적용되어있는데;

8. UAQ_parameter(예제미션 배포)에서 UAQ_parameter.o, UAQ_parameter.h 붙여넣기

9. Test_IPG (예제모델)
    Data/Road, Data/TestRun, Data/Vehicle, Movie

/ros/ros1_ws/hellocm_msgs

/msg
    새로 만들 msg file 추가

CMakeLists.txt
    add_message_files(
        사용할 msg추가
    )

->build할때 헤더파일 생성해줌



/ros/ros1_ws/hellocm_cmnode/src

CMNode_ROS1_HelloCM.cpp

추가한 msg 헤더파일 include
    #include <hellocm_msgs/{msg name}.h>

struct 추가
    tRosIF_TpcSub<hellocm_msgs::Ext2CM_Test> Ext2CM_Test;

int CMRosIF_CMNode_Init ()
    //Test1 이름으로 토픽 발행
    CMNode.Topics.Pub.CM2Ext_Test.Pub           = node->advertise<hellocm_msgs::CM2Ext_Test>("Test1", CMNode.Cfg.QueuePub);
    CMNode.Topics.Pub.CM2Ext_Test.Job           = CMCRJob_Create("CM2Ext_Test");
    //Cycletime
    CMNode.Topics.Pub.CM2Ext_Test.CycleTime     = 50;
    CMNode.Topics.Pub.CM2Ext_Test.CycleOffset   = 0;

    CMNode.Topics.Sub.Ext2CM_Test.Sub    = node->subscribe("Test2", CMNode.Cfg.QueueSub, {subscribe method name);

int CMRosIF_CMNode_Out ()
    //output
    
    auto out2 = &CMNode.Topics.Pub.CM2Ext_Test;
    out2->Msg.test1 = 123;
    out2->Pub.publish(out2->Msg);


subscribe method

static void cmnode_Ext2CM_Test_TpcIn (const hellocm_msgs::Ext2CM_Test::ConstPtr &msg)
{
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled)
	return;
    
    int rv;
    auto in = &CMNode.Topics.Sub.Ext2CM_Test;

    in->Msg.test2  = msg->test2;

    if (CMNode.Cfg.SyncMode == CMNode_SyncMode_Tpc && (rv = CMCRJob_DoPrep_SetDone(in->Job, CMNode.CycleNoRel)) != CMCRJob_RV_Success) {
	LogErrF(EC_Sim, "CMNode: Error on DoPrep_SetDone for Job '%s'! rv=%s", CMCRJob_GetName(in->Job), CMCRJob_RVStr(rv));
    }

    /* Remember cycle for debugging */
    CMNode.Model.CycleLastIn = CMNode.CycleNoRel;
}

Vehicle control

UDP_Input에 ROS_topic assign.

Calc
    UDP_Input.DriveCont.Ax = CMNode.Topics.Sub.Ext2CM_Test.Msg.acc;
    UDP_Input.DriveCont.SteeringWheel = CMNode.Topics.Sub.Ext2CM_Test.Msg.steer;
	// UDP_Input.DriveCont.SteeringWheel = 1;
    // UDP_Input.DriveCont.Ax = 1;
	UDP_Input.DriveCont.GearNo = 1;
    UDP_PC.VC_SwitchOn = 1;
	
	DrivMan.Lights.Hazard = 1;
	DrivMan.Lights.Indicator = 1;

In CM Client, AVVehicle 선택


keyboard Control.
    python3 {Prj home}/keyboard_example.py