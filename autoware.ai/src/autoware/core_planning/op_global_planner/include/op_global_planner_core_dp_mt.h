/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef OP_GLOBAL_PLANNER
#define OP_GLOBAL_PLANNER

#include<limits>
#include <thread>
#include <mutex>
#include <csignal>
#include <pthread.h>

#include <ros/ros.h>


#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/LaneArray.h"
#include "vector_map_msgs/NodeArray.h"
#include "vector_map_msgs/StopLineArray.h"
#include "vector_map_msgs/DTLaneArray.h"
#include "vector_map_msgs/LineArray.h"
#include "vector_map_msgs/AreaArray.h"
#include "vector_map_msgs/SignalArray.h"
#include "vector_map_msgs/StopLine.h"
#include "vector_map_msgs/VectorArray.h"

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <std_msgs/Int8.h>
#include "libwaypoint_follower/libwaypoint_follower.h"
#include "autoware_can_msgs/CANInfo.h"
#include <visualization_msgs/MarkerArray.h>

#include "op_planner/PlannerCommonDef.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/PlannerH.h"

# define M_PI           3.14159265358979323846

namespace GlobalPlanningNS
{

#define MAX_GLOBAL_PLAN_DISTANCE 100000
#define _ENABLE_VISUALIZE_PLAN
#define REPLANNING_DISTANCE 30
#define REPLANNING_TIME 5
#define ARRIVE_DISTANCE 5
#define CLEAR_COSTS_TIME 15 // seconds

class WayPlannerParams
{
public:
  std::string KmlMapPath;
  bool bEnableSmoothing;
  bool bEnableLaneChange;
  bool bEnableHMI;
  bool bEnableRvizInput;
  bool bEnableReplanning;
  double pathDensity;
  PlannerHNS::MAP_SOURCE_TYPE  mapSource;
  bool bEnableDynamicMapUpdate;

  WayPlannerParams()
  {
      bEnableDynamicMapUpdate = false;
    bEnableReplanning = false;
    bEnableHMI = false;
    bEnableSmoothing = false;
    bEnableLaneChange = false;
    bEnableRvizInput = true;
    pathDensity = 0.5;
    mapSource = PlannerHNS::MAP_KML_FILE;
  }
};


static std::mutex mtx, mtx2;
static std::vector<std::thread> thread_vec;
static int finished_seq_num = 0;
static int finished_thread = 0;

static std::vector< WpPtrIdVec > m_WayPointSequences;
static std::vector< WpPtrIdVec > m_allWaypointCandidates; // dim 0: waypoint id(index), dim 1: candidates for target waypoint
static std::vector< PairPath > m_PairPaths;
static std::vector< std::pair<int, int> > m_PairIds;
static std::vector< std::pair<PlannerHNS::WayPoint, PlannerHNS::WayPoint> > m_PairPoints;

static std::vector< std::pair<int, std::vector<WpVec> > > m_PathCandidates; 
static std::vector< pthread_t > m_pThreadVec;
static std::vector<std::vector<PlannerHNS::WayPoint> > m_GeneratedTotalPaths;
static bool m_ThreadWorking = false;

static WayPlannerParams m_params;
static int m_GlobalPathID;
static PlannerHNS::PlannerH m_PlannerH;
static PlannerHNS::RoadNetwork m_Map;
static int m_ThreadNum;

void threadMain(int start_idx, int end_idx, int total_size);
void clearUnnecessarySequences(int current_seq_idx, int end_idx, int fail_idx, WpPtrIdVec& planned_waypoint_pointers);
bool GenerateWaypointsGlobalPlan(std::vector<PlannerHNS::WayPoint>& wayPoints, std::vector<std::vector<PlannerHNS::WayPoint> >& generatedTotalPaths, int& fail_idx);
void signalHandler(int signum);
bool getLocalPathFromPairPaths(int start_id, int goal_id, std::vector<WpVec>& local_path);
bool GenerateGlobalPlan(PlannerHNS::WayPoint& startPoint, PlannerHNS::WayPoint& goalPoint, std::vector<std::vector<PlannerHNS::WayPoint> >& generatedTotalPaths);

class GlobalPlanner
{

public:
  int m_iCurrentGoalIndex;
protected:

  PlannerHNS::WayPoint m_CurrentPose;
  std::vector<PlannerHNS::WayPoint> m_GoalsPos;
  geometry_msgs::Pose m_OriginPos;
  PlannerHNS::VehicleState m_VehicleState;
  std::vector<int> m_GridMapIntType;
  std::vector<std::pair<std::vector<PlannerHNS::WayPoint*> , timespec> > m_ModifiedMapItemsTimes;
  timespec m_ReplnningTimer;


  bool m_bFirstStart;

  ros::NodeHandle nh;

  ros::Publisher pub_MapRviz;
  ros::Publisher pub_Paths;
  ros::Publisher pub_PathsRviz;
  ros::Publisher pub_TrafficInfo;
  //ros::Publisher pub_TrafficInfoRviz;
  //ros::Publisher pub_StartPointRviz;
  //ros::Publisher pub_GoalPointRviz;
  //ros::Publisher pub_NodesListRviz;
  ros::Publisher pub_GoalsListRviz;

  ros::Subscriber sub_robot_odom;
  ros::Subscriber sub_start_pose;
  ros::Subscriber sub_goal_pose;
  ros::Subscriber sub_current_pose;
  ros::Subscriber sub_current_velocity;
  ros::Subscriber sub_can_info;
  ros::Subscriber sub_road_status_occupancy;

  // Added by PHY
  bool m_EnableWaypoints;
  bool m_is_waypoint_received;
  bool m_isCurrentPoseReceived;
  std::vector<PlannerHNS::WayPoint> m_WayPoints;
  int m_WaypointCandidateNum;
public:
  GlobalPlanner();
  ~GlobalPlanner();
  void MainLoop();

private:
  PlannerHNS::WayPoint* m_pCurrGoal;

  void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);

  // Callback function for subscriber.
  void callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackGetStartPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input);
  void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
  void callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg);
  void callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg);
  void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
  void callbackGetRoadStatusOccupancyGrid(const nav_msgs::OccupancyGridConstPtr& msg);
  // Added by PHY
  void callbackGetGlobalWaypoints(const geometry_msgs::PoseArray& msg);
  bool GenerateWayPointSequences();
  void seq_DFS(int num_of_sequence, int depth, WpPtrIdVec entry);
  void GeneratePairPath();

  protected:
    bool  m_bKmlMap;

    
    void VisualizeAndSend(const std::vector<std::vector<PlannerHNS::WayPoint> > generatedTotalPaths);
    void VisualizeDestinations(std::vector<PlannerHNS::WayPoint>& destinations, const int& iSelected);
    void SaveSimulationData();
    int LoadSimulationData();
    void ClearOldCostFromMap();


    //Mapping Section

    UtilityHNS::MapRaw m_MapRaw;

  ros::Subscriber sub_lanes;
  ros::Subscriber sub_points;
  ros::Subscriber sub_dt_lanes;
  ros::Subscriber sub_intersect;
  ros::Subscriber sup_area;
  ros::Subscriber sub_lines;
  ros::Subscriber sub_stop_line;
  ros::Subscriber sub_signals;
  ros::Subscriber sub_vectors;
  ros::Subscriber sub_curbs;
  ros::Subscriber sub_edges;
  ros::Subscriber sub_way_areas;
  ros::Subscriber sub_cross_walk;
  ros::Subscriber sub_nodes;
  // Added by PHY
  ros::Subscriber sub_waypoints;


  void callbackGetVMLanes(const vector_map_msgs::LaneArray& msg);
  void callbackGetVMPoints(const vector_map_msgs::PointArray& msg);
  void callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg);
  void callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg);
  void callbackGetVMAreas(const vector_map_msgs::AreaArray& msg);
  void callbackGetVMLines(const vector_map_msgs::LineArray& msg);
  void callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg);
  void callbackGetVMSignal(const vector_map_msgs::SignalArray& msg);
  void callbackGetVMVectors(const vector_map_msgs::VectorArray& msg);
  void callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg);
  void callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg);
  void callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg);
  void callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg);
  void callbackGetVMNodes(const vector_map_msgs::NodeArray& msg);

};

}

#endif
