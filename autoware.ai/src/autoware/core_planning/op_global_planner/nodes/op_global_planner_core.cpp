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

#include "op_global_planner_core.h"
#include "op_ros_helpers/op_ROSHelpers.h"

namespace GlobalPlanningNS
{


GlobalPlanner::GlobalPlanner()
{
  m_pCurrGoal = 0;
  m_iCurrentGoalIndex = 0;
  m_bKmlMap = false;
  m_bFirstStart = false;
  m_GlobalPathID = 1;
  m_EnableWaypoints = false;
  m_isCurrentPoseReceived = false;
  m_WaypointCandidateNum = 1;
  m_ThreadNum = 5;
  UtilityHNS::UtilityH::GetTickCount(m_ReplnningTimer);

  nh.getParam("/op_global_planner/pathDensity" , m_params.pathDensity);
  nh.getParam("/op_global_planner/enableSmoothing" , m_params.bEnableSmoothing);
  nh.getParam("/op_global_planner/enableLaneChange" , m_params.bEnableLaneChange);
  nh.getParam("/op_global_planner/enableRvizInput" , m_params.bEnableRvizInput);
  nh.getParam("/op_global_planner/enableReplan" , m_params.bEnableReplanning);
  nh.getParam("/op_global_planner/enableDynamicMapUpdate" , m_params.bEnableDynamicMapUpdate);
  nh.getParam("/op_global_planner/mapFileName" , m_params.KmlMapPath);
  nh.getParam("/op_global_planner/enableWaypoints", m_EnableWaypoints);
  nh.getParam("/op_global_planner/waypointCandidateNum", m_WaypointCandidateNum);
  nh.getParam("/op_global_planner/threadNum", m_ThreadNum);

  bool use_static_goal = false;
  double goal_pose_x, goal_pose_y, goal_pose_z, goal_ori_x, goal_ori_y, goal_ori_z, goal_ori_w;
  nh.param<bool>("/op_global_planner/use_static_goal", use_static_goal, false);
  nh.param<double>("/op_global_planner/goal_pose_x", goal_pose_x, -1);
  nh.param<double>("/op_global_planner/goal_pose_y", goal_pose_y, -1);
  nh.param<double>("/op_global_planner/goal_pose_z", goal_pose_z, -1);
  nh.param<double>("/op_global_planner/goal_ori_x", goal_ori_x, -1);
  nh.param<double>("/op_global_planner/goal_ori_y", goal_ori_y, -1);
  nh.param<double>("/op_global_planner/goal_ori_z", goal_ori_z, -1);
  nh.param<double>("/op_global_planner/goal_ori_w", goal_ori_w, -1);  

  int iSource = 0;
  nh.getParam("/op_global_planner/mapSource", iSource);
  if(iSource == 0)
    m_params.mapSource = PlannerHNS::MAP_AUTOWARE;
  else if (iSource == 1)
    m_params.mapSource = PlannerHNS::MAP_FOLDER;
  else if(iSource == 2)
    m_params.mapSource = PlannerHNS::MAP_KML_FILE;

  tf::StampedTransform transform;
  PlannerHNS::ROSHelpers::GetTransformFromTF("map", "world", transform);
  m_OriginPos.position.x  = transform.getOrigin().x();
  m_OriginPos.position.y  = transform.getOrigin().y();
  m_OriginPos.position.z  = transform.getOrigin().z();

  pub_Paths = nh.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1, true);
  pub_PathsRviz = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_rviz", 1, true);
  pub_MapRviz  = nh.advertise<visualization_msgs::MarkerArray>("vector_map_center_lines_rviz", 1, true);
  pub_GoalsListRviz = nh.advertise<visualization_msgs::MarkerArray>("op_destinations_rviz", 1, true);

  if(m_params.bEnableRvizInput && !m_EnableWaypoints)
  {
    sub_start_pose = nh.subscribe("/initialpose", 1, &GlobalPlanner::callbackGetStartPose, this);
    sub_goal_pose = nh.subscribe("move_base_simple/goal", 1, &GlobalPlanner::callbackGetGoalPose, this);
  }
  else if(m_EnableWaypoints){
    sub_waypoints = nh.subscribe("/global_waypoints", 1, &GlobalPlanner::callbackGetGlobalWaypoints, this);
  }
  else
  {
    LoadSimulationData();
  }


  sub_current_pose = nh.subscribe("/current_pose", 10, &GlobalPlanner::callbackGetCurrentPose, this);

  int bVelSource = 1;
  nh.getParam("/op_global_planner/velocitySource", bVelSource);
  if(bVelSource == 0)
    sub_robot_odom = nh.subscribe("/odom", 10, &GlobalPlanner::callbackGetRobotOdom, this);
  else if(bVelSource == 1)
    sub_current_velocity = nh.subscribe("/current_velocity", 10, &GlobalPlanner::callbackGetVehicleStatus, this);
  else if(bVelSource == 2)
    sub_can_info = nh.subscribe("/can_info", 10, &GlobalPlanner::callbackGetCANInfo, this);

  if(m_params.bEnableDynamicMapUpdate)
    sub_road_status_occupancy = nh.subscribe<>("/occupancy_road_status", 1, &GlobalPlanner::callbackGetRoadStatusOccupancyGrid, this);

  //Mapping Section
  sub_lanes = nh.subscribe("/vector_map_info/lane", 1, &GlobalPlanner::callbackGetVMLanes,  this);
  sub_points = nh.subscribe("/vector_map_info/point", 1, &GlobalPlanner::callbackGetVMPoints,  this);
  sub_dt_lanes = nh.subscribe("/vector_map_info/dtlane", 1, &GlobalPlanner::callbackGetVMdtLanes,  this);
  sub_intersect = nh.subscribe("/vector_map_info/cross_road", 1, &GlobalPlanner::callbackGetVMIntersections,  this);
  sup_area = nh.subscribe("/vector_map_info/area", 1, &GlobalPlanner::callbackGetVMAreas,  this);
  sub_lines = nh.subscribe("/vector_map_info/line", 1, &GlobalPlanner::callbackGetVMLines,  this);
  sub_stop_line = nh.subscribe("/vector_map_info/stop_line", 1, &GlobalPlanner::callbackGetVMStopLines,  this);
  sub_signals = nh.subscribe("/vector_map_info/signal", 1, &GlobalPlanner::callbackGetVMSignal,  this);
  sub_vectors = nh.subscribe("/vector_map_info/vector", 1, &GlobalPlanner::callbackGetVMVectors,  this);
  sub_curbs = nh.subscribe("/vector_map_info/curb", 1, &GlobalPlanner::callbackGetVMCurbs,  this);
  sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &GlobalPlanner::callbackGetVMRoadEdges,  this);
  sub_way_areas = nh.subscribe("/vector_map_info/way_area", 1, &GlobalPlanner::callbackGetVMWayAreas,  this);
  sub_cross_walk = nh.subscribe("/vector_map_info/cross_walk", 1, &GlobalPlanner::callbackGetVMCrossWalks,  this);
  sub_nodes = nh.subscribe("/vector_map_info/node", 1, &GlobalPlanner::callbackGetVMNodes,  this);

  if(use_static_goal == true){
    geometry_msgs::Quaternion orientation;
    orientation.x = goal_ori_x;
    orientation.y = goal_ori_y;
    orientation.z = goal_ori_z;
    orientation.w = goal_ori_w;
    PlannerHNS::WayPoint wp = PlannerHNS::WayPoint(goal_pose_x, goal_pose_y, goal_pose_z, tf::getYaw(orientation));
    //m_GoalsPos.push_back(wp);
    m_WayPoints.push_back(wp);
  }

}

GlobalPlanner::~GlobalPlanner()
{
  if(m_params.bEnableRvizInput)
    SaveSimulationData();
}

void GlobalPlanner::callbackGetRoadStatusOccupancyGrid(const nav_msgs::OccupancyGridConstPtr& msg)
{
//  std::cout << "Occupancy Grid Origin (" << msg->info.origin.position.x << ", " << msg->info.origin.position.x << ") , " << msg->header.frame_id << ", Res: " << msg->info.resolution <<  std::endl;

  m_GridMapIntType.clear();

  //std::cout << "Found Map Data: Zero " <<  std::endl;
  for(unsigned int i=0; i < msg->data.size(); i++)
  {
    if((int8_t)msg->data.at(i) == 0)
      m_GridMapIntType.push_back(0);
    else if((int8_t)msg->data.at(i) == 50)
      m_GridMapIntType.push_back(75);
    else if((int8_t)msg->data.at(i) == 100)
      m_GridMapIntType.push_back(255);
    else
      m_GridMapIntType.push_back(128);

      //std::cout << msg->data.at(i) << ",";
  }
  //std::cout << std::endl << "--------------------------------------------------------" << std::endl;

  //std::cout << "Found Map Data: Zero : " << m_GridMapIntType.size() <<  std::endl;
  PlannerHNS::WayPoint center(msg->info.origin.position.x, msg->info.origin.position.y, msg->info.origin.position.z, tf::getYaw(msg->info.origin.orientation));
  PlannerHNS::OccupancyToGridMap grid(msg->info.width,msg->info.height, msg->info.resolution, center);
  std::vector<PlannerHNS::WayPoint*> modified_nodes;
  timespec t;
  UtilityHNS::UtilityH::GetTickCount(t);
  PlannerHNS::MappingHelpers::UpdateMapWithOccupancyGrid(grid, m_GridMapIntType, m_Map, modified_nodes);
  m_ModifiedMapItemsTimes.push_back(std::make_pair(modified_nodes, t));

  visualization_msgs::MarkerArray map_marker_array;
  PlannerHNS::ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);

//  visualization_msgs::Marker mkr = PlannerHNS::ROSHelpers::CreateGenMarker(center.pos.x, center.pos.y, center.pos.z, 0, 0,0,1,0.5, 1000, "TestCenter", visualization_msgs::Marker::SPHERE);
//
//  map_marker_array.markers.push_back(mkr);

  pub_MapRviz.publish(map_marker_array);
}

void GlobalPlanner::ClearOldCostFromMap()
{
  for(int i=0; i < (int)m_ModifiedMapItemsTimes.size(); i++)
  {
    if(UtilityHNS::UtilityH::GetTimeDiffNow(m_ModifiedMapItemsTimes.at(i).second) > CLEAR_COSTS_TIME)
    {
      for(unsigned int j= 0 ; j < m_ModifiedMapItemsTimes.at(i).first.size(); j++)
      {
        for(unsigned int i_action=0; i_action < m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.size(); i_action++)
        {
          if(m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.at(i_action).first == PlannerHNS::FORWARD_ACTION)
          {
            m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.at(i_action).second = 0;
          }
        }
      }

      m_ModifiedMapItemsTimes.erase(m_ModifiedMapItemsTimes.begin()+i);
      i--;
    }
  }
}

void GlobalPlanner::callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  PlannerHNS::WayPoint wp = PlannerHNS::WayPoint(msg->pose.position.x+m_OriginPos.position.x, msg->pose.position.y+m_OriginPos.position.y, msg->pose.position.z+m_OriginPos.position.z, tf::getYaw(msg->pose.orientation));
  m_GoalsPos.push_back(wp);

  ROS_INFO("Received Goal Pose");
}

void GlobalPlanner::callbackGetStartPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  m_isCurrentPoseReceived = true;
  m_CurrentPose = PlannerHNS::WayPoint(msg->pose.pose.position.x+m_OriginPos.position.x, msg->pose.pose.position.y+m_OriginPos.position.y, msg->pose.pose.position.z+m_OriginPos.position.z, tf::getYaw(msg->pose.pose.orientation));
  ROS_INFO("Received Start pose");
}

void GlobalPlanner::callbackGetGlobalWaypoints(const geometry_msgs::PoseArray& msg)
{
  if(m_WayPoints.size() > 0)
    return;
  m_WayPoints.clear();
  if(m_isCurrentPoseReceived){
    m_WayPoints.push_back(m_CurrentPose);    

    for(auto it = msg.poses.begin(); it != msg.poses.end(); ++it){
      PlannerHNS::WayPoint wp = PlannerHNS::WayPoint((*it).position.x+m_OriginPos.position.x, (*it).position.y+m_OriginPos.position.y, (*it).position.z+m_OriginPos.position.z, tf::getYaw((*it).orientation));
      m_WayPoints.push_back(wp);
    }    
  }

  std::cout<<"[ Waypoints ]"<<std::endl;
  for(auto it = m_WayPoints.begin(); it != m_WayPoints.end(); ++it)
    std::cout<<(*it).pos.x<<" "<<(*it).pos.y<<std::endl;
  sleep(1);

  ROS_INFO("Received Waypoints");
}

void GlobalPlanner::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  m_isCurrentPoseReceived = true;
  m_CurrentPose = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
}

void GlobalPlanner::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
  m_VehicleState.speed = msg->twist.twist.linear.x;
  m_CurrentPose.v = m_VehicleState.speed;
  if(fabs(msg->twist.twist.linear.x) > 0.25)
    m_VehicleState.steer += atan(2.7 * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
}

void GlobalPlanner::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
  m_VehicleState.speed = msg->twist.linear.x;
  m_CurrentPose.v = m_VehicleState.speed;
  if(fabs(msg->twist.linear.x) > 0.25)
    m_VehicleState.steer = atan(2.7 * msg->twist.angular.z/msg->twist.linear.x);
  UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
}

void GlobalPlanner::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
  m_VehicleState.speed = msg->speed/3.6;
  m_CurrentPose.v = m_VehicleState.speed;
  m_VehicleState.steer = msg->angle * 0.45 / 660;
  UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
}

bool GenerateGlobalPlan(PlannerHNS::WayPoint& startPoint, PlannerHNS::WayPoint& goalPoint, std::vector<std::vector<PlannerHNS::WayPoint> >& generatedTotalPaths)
{
  std::vector<int> predefinedLanesIds;
  double ret = 0;

  int rot_cnt = 0;

  while(rot_cnt < 12){
    if(ret != 0) break;
    ret = m_PlannerH.PlanUsingDP(startPoint, goalPoint, MAX_GLOBAL_PLAN_DISTANCE, m_params.bEnableLaneChange, predefinedLanesIds, m_Map, generatedTotalPaths);  
    double degree = goalPoint.pos.a/M_PI*180;
    goalPoint.pos.a  = (degree+30)/180*M_PI;
    rot_cnt++;
  }

  if(ret == 0)
  {
    // std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString()
    //                     << ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
    return false;
  }


  if(generatedTotalPaths.size() > 0 && generatedTotalPaths.at(0).size()>0)
  {
    if(m_params.bEnableSmoothing)
    {
      for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
      {
        PlannerHNS::PlanningHelpers::FixPathDensity(generatedTotalPaths.at(i), m_params.pathDensity);
        PlannerHNS::PlanningHelpers::SmoothPath(generatedTotalPaths.at(i), 0.49, 0.35 , 0.01);
      }
    }

    for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
    {
      PlannerHNS::PlanningHelpers::CalcAngleAndCost(generatedTotalPaths.at(i));
      if(m_GlobalPathID > 10000)
        m_GlobalPathID = 1;

      for(unsigned int j=0; j < generatedTotalPaths.at(i).size(); j++)
        generatedTotalPaths.at(i).at(j).gid = m_GlobalPathID;

      m_GlobalPathID++;

      // std::cout << "New DP Path -> " << generatedTotalPaths.at(i).size() << std::endl;
    }
    return true;
  }
  else
  {
    // std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString() << ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
  }
  return false;
}

bool GlobalPlanner::GenerateWayPointSequences(){
  static int waypoint_id = 0;  
  bool areCandidatesGenerated = true;

  m_allWaypointCandidates.clear();
  m_WayPointSequences.clear();
  m_PairPaths.clear();
  m_PairIds.clear();
  m_PairPoints.clear();

  for(int i = 0; i < m_WayPoints.size(); i++){
    WpPtrIdVec waypoint_candidates;
    PlannerHNS::WayPoint wp = m_WayPoints[i];
    
    if(i==0){
      waypoint_candidates.push_back(WpPtrId(waypoint_id, &m_CurrentPose));
      waypoint_id++;
    }    

    WpPtrVec candidatwe_without_id;
    candidatwe_without_id = PlannerHNS::MappingHelpers::GetCloseWaypointsFromMap(wp, m_Map, true, m_WaypointCandidateNum);
    
    for(auto it = candidatwe_without_id.begin(); it != candidatwe_without_id.end(); ++it){
      PlannerHNS::WayPoint* candidate = *it;
      if(i==0){
        double angle_diff = ((m_CurrentPose.pos.a)/M_PI*180.0) - (((*it)->pos.a)/M_PI*180.0);
        if(fabs(angle_diff) > 150) continue;
      }

      
      waypoint_candidates.push_back(WpPtrId(waypoint_id, candidate));
      waypoint_id++;
    }

    m_allWaypointCandidates.push_back(waypoint_candidates);
    if(waypoint_candidates.size() == 0) areCandidatesGenerated = false;
  }

  if(areCandidatesGenerated){
    for(auto it = m_allWaypointCandidates.begin(); it != m_allWaypointCandidates.end(); ++it){
      WpPtrIdVec waypoint_candidates = *it;
      std::cout<<" >> Waypoint "<<it - m_allWaypointCandidates.begin()<<" Candidates "<<std::endl;
      for(int i = 0; i < waypoint_candidates.size(); ++i){
        std::cout<<"   ID: "<<waypoint_candidates[i].first<<" / "<<waypoint_candidates[i].second->pos.x<<" "<<waypoint_candidates[i].second->pos.y<<std::endl;
        // std::cout<<"   ID: "<<waypoint_candidates[i].first<<std::endl;
      }
    }
  }
  else{
    return false;
  }

  int num_of_sequence = m_allWaypointCandidates[0].size();
  for(auto it = m_allWaypointCandidates.begin()+1; it != m_allWaypointCandidates.end(); ++it){
    num_of_sequence *= (*it).size();
  }

  std::cout<<"num of sequence: "<<num_of_sequence<<std::endl;
  WpPtrIdVec entry;
  seq_DFS(num_of_sequence, 0, entry);

  return areCandidatesGenerated;
}

void GlobalPlanner::GeneratePairPath(){
  
  // Create Pair points and ids
  for(int depth = 0; depth < m_allWaypointCandidates.size()-1; depth++){
    
    for(int i = 0; i < m_allWaypointCandidates[depth].size(); i++){
    int start_point_id = m_allWaypointCandidates[depth][i].first;
    PlannerHNS::WayPoint start_point = *(m_allWaypointCandidates[depth][i].second);

      for(int j = 0; j < m_allWaypointCandidates[depth+1].size(); j++){
        int goal_point_id = m_allWaypointCandidates[depth+1][j].first;
        PlannerHNS::WayPoint goal_point = *(m_allWaypointCandidates[depth+1][j].second);

        m_PairIds.push_back(std::pair<int, int>(start_point_id, goal_point_id));
        m_PairPoints.push_back(std::pair<PlannerHNS::WayPoint, PlannerHNS::WayPoint>(start_point, goal_point));
      }
    }
  }

  // Multi Threading
  int start_idx = 0;            
  int remain_num = m_PairIds.size();
  int one_data_num = remain_num/m_ThreadNum+1;   

  std::cout<<" >> Total Pair: "<<m_PairIds.size()<<std::endl;

  for(int thread_idx = 0; thread_idx < m_ThreadNum-1; ++thread_idx){
    if(one_data_num <= 0 || remain_num < one_data_num)
      break;
    thread_vec.push_back(std::thread(threadMain, start_idx, start_idx+one_data_num, m_PairIds.size()));
    m_pThreadVec.push_back(thread_vec[thread_idx].native_handle());

    std::cout <<" >> Thread "<<thread_idx<<": Pair("<<start_idx<<") ~ Pair("<<start_idx+one_data_num-1<<")"<<std::endl;
    start_idx += one_data_num;
    remain_num -= one_data_num;
  }

  if(remain_num > 0){
    thread_vec.push_back(std::thread(threadMain, start_idx, start_idx+remain_num, m_PairIds.size()));
    std::cout <<" >> Last Thread: Pair("<<start_idx<<") ~ Pair("<<start_idx+remain_num-1<<")"<<std::endl;
  }


  for(auto t_it = thread_vec.begin(); t_it != thread_vec.end(); ++t_it)
    (*t_it).join();

  while(1){
    if(finished_thread==thread_vec.size()) break;
    sleep(1);
  }

}

void threadMain(int start_idx, int end_idx, int total_size){
  for(int i = start_idx; i < end_idx; i++){
    if(i >= total_size-1) break;
      int start_point_id = m_PairIds[i].first;
      int goal_point_id = m_PairIds[i].second;
      PlannerHNS::WayPoint start_point = m_PairPoints[i].first;
      PlannerHNS::WayPoint goal_point = m_PairPoints[i].second;

      std::vector<WpVec> local_path;
      bool isPathCreated = GenerateGlobalPlan(start_point, goal_point, local_path);
      
      PairPath entry;
      if(isPathCreated){
        entry = PairPath(start_point_id, goal_point_id, local_path);
        std::cout<< ">> Local Path " <<get<0>(entry)<<" -> "<< get<1>(entry) <<" / size: "<< (get<2>(entry)).at(0).size()<< std::endl;
      }
      else{
        entry = PairPath(start_point_id, goal_point_id, std::vector<WpVec>());
      }
      mtx.lock();
      m_PairPaths.push_back(entry);
      mtx.unlock();
  }

  finished_thread++;
}


void GlobalPlanner::seq_DFS(int num_of_sequence, int depth, WpPtrIdVec entry ){
  if(depth == m_allWaypointCandidates.size()){
    m_WayPointSequences.push_back(entry);
    return;
  }

  int iter = num_of_sequence;
  for(int i = 0; i < depth+1; i++){
    iter /= m_allWaypointCandidates[i].size();
  }

  for(int i = 0; i < m_allWaypointCandidates[depth].size(); i++){
    WpPtrIdVec data = entry;
    data.push_back(m_allWaypointCandidates[depth][i]);
    seq_DFS(num_of_sequence, depth+1, data);
  }

}

bool GenerateWaypointsGlobalPlan(std::vector<PlannerHNS::WayPoint>& wayPoints, std::vector<std::vector<PlannerHNS::WayPoint> >& generatedTotalPaths, int& fail_idx)
{ 
  if(wayPoints.size() == 0)
    return false;

  std::vector<int> predefinedLanesIds;
  double ret = 0;
  std::vector<std::vector<PlannerHNS::WayPoint> > temp_paths;
  std::vector<PlannerHNS::WayPoint> last_path;
  PlannerHNS::WayPoint startPoint;

  startPoint = wayPoints[0];

  generatedTotalPaths.clear();
  
  int rot_cnt = 0;

  for(auto it = wayPoints.begin(); it != wayPoints.end()-1; ++it){       
    PlannerHNS::WayPoint start_wp = *it;
    PlannerHNS::WayPoint end_wp = *(it+1);
    rot_cnt = 0;
    ret = 0;
    temp_paths.clear();   
    while(rot_cnt < 12){
      if(ret != 0){
        // PlannerHNS::WayPoint* updated_goal = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(end_wp, m_Map);
        // if(updated_goal != nullptr) 
        //   *(it+1) = (*updated_goal);
        break;
      }
      ret = m_PlannerH.PlanUsingDP(start_wp, end_wp, MAX_GLOBAL_PLAN_DISTANCE, m_params.bEnableLaneChange, predefinedLanesIds, m_Map, temp_paths);
      double degree = end_wp.pos.a/M_PI*180;
      end_wp.pos.a  = (degree+30)/180*M_PI;
      rot_cnt++;
    }

    if(ret != 0 && temp_paths.size() != 0){
      for(auto it = temp_paths.at(0).begin(); it != temp_paths.at(0).end(); ++it){
        PlannerHNS::WayPoint wp = *it; 
        last_path.push_back(wp);
      }
    }
    else{
      // std::cout << "Can't Generate Global Path for Start (" << start_wp.pos.ToString()
      //                   << ") and Goal (" << end_wp.pos.ToString() << ")" << std::endl;
      fail_idx = it - wayPoints.begin()+1;
      return false;
    }
  }
  
  generatedTotalPaths.push_back(last_path);

  if(ret == 0)
  {
    std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString()
                        << ") and Goal (" << wayPoints.back().pos.ToString() << ")" << std::endl;    
    return false;
  }

  if(generatedTotalPaths.size() > 0 && generatedTotalPaths.at(0).size()>0)
  {
    if(m_params.bEnableSmoothing)
    {
      for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
      {
        PlannerHNS::PlanningHelpers::FixPathDensity(generatedTotalPaths.at(i), m_params.pathDensity);
        PlannerHNS::PlanningHelpers::SmoothPath(generatedTotalPaths.at(i), 0.49, 0.35 , 0.01);
      }
    }

    for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
    {
      PlannerHNS::PlanningHelpers::CalcAngleAndCost(generatedTotalPaths.at(i));
      if(m_GlobalPathID > 10000)
        m_GlobalPathID = 1;

      for(unsigned int j=0; j < generatedTotalPaths.at(i).size(); j++)
        generatedTotalPaths.at(i).at(j).gid = m_GlobalPathID;

      m_GlobalPathID++;

      // std::cout << "New DP Path -> " << generatedTotalPaths.at(i).size() << std::endl;
    }
    return true;
  }
  else
  {
    // std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString() << ") and Goal (" << wayPoints.back().pos.ToString() << ")" << std::endl;
  }
  return false;
}

void GlobalPlanner::VisualizeAndSend(const std::vector<std::vector<PlannerHNS::WayPoint> > generatedTotalPaths)
{
  autoware_msgs::LaneArray lane_array;
  visualization_msgs::MarkerArray pathsToVisualize;

  for(int i=0; i< generatedTotalPaths.at(0).size(); i++){
    PlannerHNS::WayPoint w = generatedTotalPaths.at(0).at(i);

    // printf("pid : %d, laneId : %d, lpid : %d, rpid : %d, llid : %d, rlid : %d, origin_idx : %d, toId.size : %d, fromId.size : %d\n", 
    //   w.id, w.laneId, w.LeftPointId, w.RightPointId, w.LeftLnId, w.RightLnId, w.iOriginalIndex, w.toIds.size(), w.fromIds.size());

    // printf("pid : %d, laneId : %d, toId : [", w.id, w.laneId);

    // for(int p=0; p<w.pFronts.size(); p++){
    //   printf("%d ", w.pFronts.at(p)->id);
    // }

    // printf("]\n");

    // std::cout << "pid : " << generatedTotalPaths.at(0).at(i).id << ", laneId : " << generatedTotalPaths.at(0).at(i).laneId << ", start_idx : " << generatedTotalPaths.at(0).at(i).LeftLnId << ", end_idx : " << generatedTotalPaths.at(0).at(i).RightLnId << std::endl;
  }

  for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
  {
    autoware_msgs::Lane lane;
    PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(generatedTotalPaths.at(i), lane);
    lane_array.lanes.push_back(lane);
  }

  std_msgs::ColorRGBA total_color;
  total_color.r = 0;
  total_color.g = 0.7;
  total_color.b = 1.0;
  total_color.a = 0.9;
  PlannerHNS::ROSHelpers::createGlobalLaneArrayMarker(total_color, lane_array, pathsToVisualize);
  PlannerHNS::ROSHelpers::createGlobalLaneArrayOrientationMarker(lane_array, pathsToVisualize);
  PlannerHNS::ROSHelpers::createGlobalLaneArrayVelocityMarker(lane_array, pathsToVisualize);
  pub_PathsRviz.publish(pathsToVisualize);
  if((m_bFirstStart && m_params.bEnableHMI) || !m_params.bEnableHMI)
    pub_Paths.publish(lane_array);

  for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
  {
    std::ostringstream str_out;
    str_out << UtilityHNS::UtilityH::GetHomeDirectory();
    str_out << UtilityHNS::DataRW::LoggingMainfolderName;
    str_out << UtilityHNS::DataRW::GlobalPathLogFolderName;
    str_out << "GlobalPath_";
    str_out << i;
    str_out << "_";
    PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), generatedTotalPaths.at(i));
  }
}

void GlobalPlanner::VisualizeDestinations(std::vector<PlannerHNS::WayPoint>& destinations, const int& iSelected)
{
  visualization_msgs::MarkerArray goals_array;

  for(unsigned int i=0; i< destinations.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "HMI_Destinations";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 3.25;
    marker.scale.y = 3.25;
    marker.scale.z = 3.25;
    marker.color.a = 0.9;
    marker.id = i;
    if(i == iSelected)
    {
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
    }
    else
    {
      marker.color.r = 0.2;
      marker.color.g = 0.8;
      marker.color.b = 0.2;
    }
    marker.pose.position.x = destinations.at(i).pos.x;
    marker.pose.position.y = destinations.at(i).pos.y;
    marker.pose.position.z = destinations.at(i).pos.z;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(destinations.at(i).pos.a);

    std::ostringstream str_out;
    str_out << "G";
    marker.text = str_out.str();

    goals_array.markers.push_back(marker);
  }
  pub_GoalsListRviz.publish(goals_array);
}

void GlobalPlanner::SaveSimulationData()
{
  std::vector<std::string> simulationDataPoints;
  std::ostringstream startStr;
  startStr << m_CurrentPose.pos.x << "," << m_CurrentPose.pos.y << "," << m_CurrentPose.pos.z << "," << m_CurrentPose.pos.a << ","<< m_CurrentPose.cost << "," << 0 << ",";
  simulationDataPoints.push_back(startStr.str());

  for(unsigned int i=0; i < m_GoalsPos.size(); i++)
  {
    std::ostringstream goalStr;
    goalStr << m_GoalsPos.at(i).pos.x << "," << m_GoalsPos.at(i).pos.y << "," << m_GoalsPos.at(i).pos.z << "," << m_GoalsPos.at(i).pos.a << "," << 0 << "," << 0 << ",destination_" << i+1 << ",";
    simulationDataPoints.push_back(goalStr.str());
  }

  std::string header = "X,Y,Z,A,C,V,name,";

  std::ostringstream fileName;
  fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::SimulationFolderName;
  fileName << "EgoCar.csv";
  std::ofstream f(fileName.str().c_str());

  if(f.is_open())
  {
    if(header.size() > 0)
      f << header << "\r\n";
    for(unsigned int i = 0 ; i < simulationDataPoints.size(); i++)
      f << simulationDataPoints.at(i) << "\r\n";
  }

  f.close();
}

int GlobalPlanner::LoadSimulationData()
{
  std::ostringstream fileName;
  fileName << "EgoCar.csv";

  std::string simuDataFileName = UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::SimulationFolderName + fileName.str();
  UtilityHNS::SimulationFileReader sfr(simuDataFileName);
  UtilityHNS::SimulationFileReader::SimulationData data;

  int nData = sfr.ReadAllData(data);
  if(nData == 0)
    return 0;

  m_CurrentPose = PlannerHNS::WayPoint(data.startPoint.x, data.startPoint.y, data.startPoint.z, data.startPoint.a);
  m_GoalsPos.push_back(PlannerHNS::WayPoint(data.goalPoint.x, data.goalPoint.y, data.goalPoint.z, data.goalPoint.a));

  for(unsigned int i=0; i < data.simuCars.size(); i++)
  {
    m_GoalsPos.push_back(PlannerHNS::WayPoint(data.simuCars.at(i).x, data.simuCars.at(i).y, data.simuCars.at(i).z, data.simuCars.at(i).a));
  }

  return nData;
}



void GlobalPlanner::MainLoop()
{
  ros::Rate loop_rate(25);
  timespec animation_timer;
  UtilityHNS::UtilityH::GetTickCount(animation_timer);
  bool isCandidatesCreated = false;

  signal(SIGINT, signalHandler);
  signal(SIGQUIT, signalHandler);

  while (ros::ok())
  {
    ros::spinOnce();
    bool bMakeNewPlan = false;

    if(m_params.mapSource == PlannerHNS::MAP_KML_FILE && !m_bKmlMap)
    {
      m_bKmlMap = true;
      PlannerHNS::MappingHelpers::LoadKML(m_params.KmlMapPath, m_Map);
      visualization_msgs::MarkerArray map_marker_array;
      PlannerHNS::ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);
      pub_MapRviz.publish(map_marker_array);
    }
    else if (m_params.mapSource == PlannerHNS::MAP_FOLDER && !m_bKmlMap)
    {
      m_bKmlMap = true;
      PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(m_params.KmlMapPath, m_Map, true);
      visualization_msgs::MarkerArray map_marker_array;
      PlannerHNS::ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);

      pub_MapRviz.publish(map_marker_array);
    }
    else if (m_params.mapSource == PlannerHNS::MAP_AUTOWARE && !m_bKmlMap)
    {
      std::vector<UtilityHNS::AisanDataConnFileReader::DataConn> conn_data;;

      if(m_MapRaw.GetVersion()==2)
      {
        std::cout << "Map Version 2" << endl;
        m_bKmlMap = true;
        PlannerHNS::MappingHelpers::ConstructRoadNetworkFromROSMessageV2(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
            m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
            m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,  m_MapRaw.pSignals->m_data_list,
            m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
            m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,
            m_MapRaw.pLanes, m_MapRaw.pPoints, m_MapRaw.pNodes, m_MapRaw.pLines, PlannerHNS::GPSPoint(), m_Map, true, m_params.bEnableLaneChange, false);

        // Add Lane Info from yaml file
        XmlRpc::XmlRpcValue lane_info_xml;
        nh.getParam("/op_global_planner/lane_info_list", lane_info_xml);
        PlannerHNS::MappingHelpers::ConstructLaneInfo_RUBIS(m_Map, lane_info_xml);
      }
      else if(m_MapRaw.GetVersion()==1)
      {
        std::cout << "Map Version 1" << endl;
        m_bKmlMap = true;
        PlannerHNS::MappingHelpers::ConstructRoadNetworkFromROSMessage(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
            m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
            m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,  m_MapRaw.pSignals->m_data_list,
            m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
            m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,  PlannerHNS::GPSPoint(), m_Map, true, m_params.bEnableLaneChange, false);
      }

      if(m_bKmlMap)
      {
        visualization_msgs::MarkerArray map_marker_array;
        PlannerHNS::ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);
        pub_MapRviz.publish(map_marker_array);
      }
    }

    ClearOldCostFromMap();

    if(!m_EnableWaypoints){ // Use only one goal(waypoint)
      if(m_GoalsPos.size() > 0)
      {
        if(m_GeneratedTotalPaths.size() > 0 && m_GeneratedTotalPaths.at(0).size() > 3)
        {
          if(m_params.bEnableReplanning)
          {
            PlannerHNS::RelativeInfo info;
            bool ret = PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_GeneratedTotalPaths, m_CurrentPose, 0.75, info);
            if(ret == true && info.iGlobalPath >= 0 &&  info.iGlobalPath < m_GeneratedTotalPaths.size() && info.iFront > 0 && info.iFront < m_GeneratedTotalPaths.at(info.iGlobalPath).size())
            {
              double remaining_distance =    m_GeneratedTotalPaths.at(info.iGlobalPath).at(m_GeneratedTotalPaths.at(info.iGlobalPath).size()-1).cost - (m_GeneratedTotalPaths.at(info.iGlobalPath).at(info.iFront).cost + info.to_front_distance);
              if(remaining_distance <= REPLANNING_DISTANCE)
              {
                bMakeNewPlan = true;
                if(m_GoalsPos.size() > 0)
                  m_iCurrentGoalIndex = (m_iCurrentGoalIndex + 1) % m_GoalsPos.size();
                std::cout << "Current Goal Index = " << m_iCurrentGoalIndex << std::endl << std::endl;
              }
            }
          }
        }
        else
          bMakeNewPlan = true;

        if(bMakeNewPlan || (m_params.bEnableDynamicMapUpdate && UtilityHNS::UtilityH::GetTimeDiffNow(m_ReplnningTimer) > REPLANNING_TIME))
        {
          UtilityHNS::UtilityH::GetTickCount(m_ReplnningTimer);
          PlannerHNS::WayPoint goalPoint = m_GoalsPos.at(m_iCurrentGoalIndex);
          bool bNewPlan = GenerateGlobalPlan(m_CurrentPose, goalPoint, m_GeneratedTotalPaths);


          if(bNewPlan)
          {
            bMakeNewPlan = false;
            VisualizeAndSend(m_GeneratedTotalPaths);
          }
        }
        VisualizeDestinations(m_GoalsPos, m_iCurrentGoalIndex);
      }
    }
    else{ // Use multiple waypoint
      if(m_WayPoints.size() > 0)
      {
        if(m_GeneratedTotalPaths.size() > 0 && m_GeneratedTotalPaths.at(0).size() > 3)
        {
          if(m_params.bEnableReplanning)
          {
            PlannerHNS::RelativeInfo info;
            bool ret = PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_GeneratedTotalPaths, m_CurrentPose, 0.75, info);
            if(ret == true && info.iGlobalPath >= 0 &&  info.iGlobalPath < m_GeneratedTotalPaths.size() && info.iFront > 0 && info.iFront < m_GeneratedTotalPaths.at(info.iGlobalPath).size())
            {
              double remaining_distance =    m_GeneratedTotalPaths.at(info.iGlobalPath).at(m_GeneratedTotalPaths.at(info.iGlobalPath).size()-1).cost - (m_GeneratedTotalPaths.at(info.iGlobalPath).at(info.iFront).cost + info.to_front_distance);
              if(remaining_distance <= REPLANNING_DISTANCE)
              {
                bMakeNewPlan = true;
                if(m_GoalsPos.size() > 0)
                  m_iCurrentGoalIndex = (m_iCurrentGoalIndex + 1) % m_GoalsPos.size();
                std::cout << "Current Goal Index = " << m_iCurrentGoalIndex << std::endl << std::endl;
              }
            }
          }
        }
        else
          bMakeNewPlan = true;
        
        if(bMakeNewPlan || (m_params.bEnableDynamicMapUpdate && UtilityHNS::UtilityH::GetTimeDiffNow(m_ReplnningTimer) > REPLANNING_TIME))
        {
          UtilityHNS::UtilityH::GetTickCount(m_ReplnningTimer);

          if(m_ThreadWorking == true) continue; // avoid duplicated execution

          // Generate candidates
          if(!isCandidatesCreated){
            isCandidatesCreated = GenerateWayPointSequences();
          }

          if(DEBUG_FLAG) {
            std::cout<<"## Sequence size: "<<m_WayPointSequences.size()<<std::endl;
            for(int seq_id = 0; seq_id != m_WayPointSequences.size(); ++seq_id){
              std::cout<<"## Sequence id: "<<seq_id<<std::endl;
              WpPtrIdVec waypoints;
              waypoints = (m_WayPointSequences[seq_id]);
              for(auto pid = 0; pid < waypoints.size(); ++pid){
                PlannerHNS::WayPoint point = *(waypoints[pid].second);
                int id = waypoints[pid].first;
                // std:cout<<"ID:"<<id<<"("<<point.pos.x<<" "<<point.pos.y<<") -> ";
                std:cout<<id<<" -> ";
              }
              std::cout<<"(End)"<<endl;
            }
            
          }

          
          if(isCandidatesCreated){
            // Generate all combination of candidates                            
            m_PathCandidates.clear();

            // Generate Pair Path
            GeneratePairPath();           
            m_ThreadWorking = true;
            for(int seq_id = 0; seq_id != m_WayPointSequences.size(); ++seq_id){              
              WpPtrIdVec seq;
              seq = (m_WayPointSequences[seq_id]);
              bool isPlannable = true;
              std::vector<WpVec> candidate_path;
              candidate_path.push_back(WpVec());
              for(int seq_data_idx = 0; seq_data_idx < seq.size()-1; ++seq_data_idx){
                bool isLocalPathExist = false; 
                int start_id = seq[seq_data_idx].first;
                int goal_id = seq[seq_data_idx+1].first;
                std::vector<WpVec> local_path;
                isLocalPathExist = getLocalPathFromPairPaths(start_id, goal_id, local_path);                
                if(isLocalPathExist){
                  for(auto local_it = local_path.at(0).begin(); local_it != local_path.at(0).end(); local_it++){
                    candidate_path.at(0).push_back(*local_it);
                  }
                }
                else{
                  isPlannable = false;
                  break;
                }
              }
              if(!isPlannable) continue;
              else{
                std::cout<<"["<<(float)seq_id/(float)m_WayPointSequences.size()*100<<"] Seq: "<<seq_id<<" Success!("<<candidate_path.at(0).size()<<")"<<std::endl;
                if(m_GeneratedTotalPaths.empty())
                  m_GeneratedTotalPaths = candidate_path;              
                else if(m_GeneratedTotalPaths.at(0).size() > candidate_path.at(0).size())
                  m_GeneratedTotalPaths = candidate_path;
              }

            }
            
            bool bNewPlan = false;
            if( !(m_GeneratedTotalPaths.empty()) ){
              bNewPlan = true;
              std::cout<<" >> Selected Path Size: "<<m_GeneratedTotalPaths.at(0).size()<<std::endl;
            }
            else{
              ROS_ERROR("Path is not plannable!");
              exit(1);
            }
            thread_vec.clear();            
            m_ThreadWorking = false;

            /*
            int start_idx = 0;            
            int remain_num = m_WayPointSequences.size();
            int one_data_num = remain_num/m_ThreadNum+1;                        
            for(int thread_idx = 0; thread_idx < m_ThreadNum-1; ++thread_idx){
              if(one_data_num <= 0 || remain_num < one_data_num)
                break;
              thread_vec.push_back(std::thread(threadMain, start_idx, start_idx+one_data_num));
              m_pThreadVec.push_back(thread_vec[thread_idx].native_handle());

              std::cout <<" >> Thread "<<thread_idx<<": Seq("<<start_idx<<") ~ Seq("<<start_idx+one_data_num-1<<")"<<std::endl;
              start_idx += one_data_num;
              remain_num -= one_data_num;
            }
            if(remain_num > 0)
              thread_vec.push_back(std::thread(threadMain, start_idx, start_idx+remain_num));
            std::cout <<" >> Thread "<<m_ThreadNum-1<<": Seq("<<start_idx<<") ~ Seq("<<start_idx+remain_num-1<<")"<<std::endl;

            

            std::cout<<"============================================"<<std::endl;
            
            for(auto t_it = thread_vec.begin(); t_it != thread_vec.end(); ++t_it)
              (*t_it).join();

            while(1){
              if(finished_thread==thread_vec.size()) break;
              sleep(1);
            }

            std::cout<<"============================================"<<std::endl;            
            
            
            std::cout<<" ## Planning for all sequence is finished!" <<std::endl;
            // Select shortest path
            bool bNewPlan;            
            int min_path_size = std::numeric_limits<int>::max();
            int selected_path_id = -1;
            for(auto it = m_PathCandidates.begin(); it != m_PathCandidates.end(); ++it){
              std::vector<std::vector<PlannerHNS::WayPoint> > candidate_path = (*it).second;

              if(candidate_path.size()!=0 && candidate_path[0].size() < min_path_size){
                selected_path_id = (*it).first;
                min_path_size = candidate_path[0].size();
                m_GeneratedTotalPaths = candidate_path;
                bNewPlan = true;
              }
            }

            std::cout<<" ## Selected Path: "<<selected_path_id<<std::endl;
            */

            if(bNewPlan)
            {
              bMakeNewPlan = false;
              VisualizeAndSend(m_GeneratedTotalPaths);
              sub_waypoints.shutdown();
            }
          }
          
        }
        VisualizeDestinations(m_GoalsPos, m_iCurrentGoalIndex);
      }
    }

    loop_rate.sleep();
  }
}

 bool getLocalPathFromPairPaths(int start_id, int goal_id, std::vector<WpVec>& local_path){   
  for(auto it = m_PairPaths.begin(); it != m_PairPaths.end(); ++it){
    int path_start_id = get<0>(*it);
    int path_goal_id = get<1>(*it);

    std::vector<WpVec> local_path_data = get<2>(*it);
    
    if(path_start_id == start_id && path_goal_id == goal_id && local_path_data.size() != 0){  
      local_path = local_path_data;
      return true;
    }
  }
  return false;
}

void clearUnnecessarySequences(int current_seq_idx, int end_idx, int fail_idx, WpPtrIdVec& planned_waypoint_pointers){
  std::vector<int> fail_waypoints_vec;
  
  // Create vector of ids to failure sequence
  for(auto it = planned_waypoint_pointers.begin(); it != planned_waypoint_pointers.end(); ++it){
    fail_waypoints_vec.push_back((*it).first);
  }

  // Clear not necessary sequences
  for(int i = current_seq_idx+1; i < end_idx; ++i){  
    WpPtrIdVec target_sequence = m_WayPointSequences[i];
    if(target_sequence.size()==0){
      continue;
    }
    bool isCleared = true;
    for(int idx = 0; idx < fail_idx + 1; ++idx){
      if(target_sequence[idx].first != fail_waypoints_vec[idx]){
        isCleared = false;
        break;
      }
    }
    if(isCleared){
      m_WayPointSequences[i].clear();
      finished_seq_num++;
      // std::cout<< " >> Clear Sequence ("<<i<<")"<<std::endl;
    }
  }  
}

void signalHandler(int signum){
  std::cout<<"[ Kill all op_global_planner's threads! ]"<<std::endl;
  for(auto it = m_pThreadVec.begin(); it != m_pThreadVec.end(); ++it){
    pthread_cancel(*it);
  }
  exit(0);
}

//Mapping Section

void GlobalPlanner::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
  std::cout << "Received Lanes" << msg.data.size() << endl;
  if(m_MapRaw.pLanes == nullptr)
    m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
}

void GlobalPlanner::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
  std::cout << "Received Points" << msg.data.size() << endl;
  if(m_MapRaw.pPoints  == nullptr)
    m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
}

void GlobalPlanner::callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg)
{
  std::cout << "Received dtLanes" << msg.data.size() << endl;
  if(m_MapRaw.pCenterLines == nullptr)
    m_MapRaw.pCenterLines = new UtilityHNS::AisanCenterLinesFileReader(msg);
}

void GlobalPlanner::callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg)
{
  std::cout << "Received CrossRoads" << msg.data.size() << endl;
  if(m_MapRaw.pIntersections == nullptr)
    m_MapRaw.pIntersections = new UtilityHNS::AisanIntersectionFileReader(msg);
}

void GlobalPlanner::callbackGetVMAreas(const vector_map_msgs::AreaArray& msg)
{
  std::cout << "Received Areas" << msg.data.size() << endl;
  if(m_MapRaw.pAreas == nullptr)
    m_MapRaw.pAreas = new UtilityHNS::AisanAreasFileReader(msg);
}

void GlobalPlanner::callbackGetVMLines(const vector_map_msgs::LineArray& msg)
{
  std::cout << "Received Lines" << msg.data.size() << endl;
  if(m_MapRaw.pLines == nullptr)
    m_MapRaw.pLines = new UtilityHNS::AisanLinesFileReader(msg);
}

void GlobalPlanner::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
  std::cout << "Received StopLines" << msg.data.size() << endl;
  if(m_MapRaw.pStopLines == nullptr)
    m_MapRaw.pStopLines = new UtilityHNS::AisanStopLineFileReader(msg);
}

void GlobalPlanner::callbackGetVMSignal(const vector_map_msgs::SignalArray& msg)
{
  std::cout << "Received Signals" << msg.data.size() << endl;
  if(m_MapRaw.pSignals  == nullptr)
    m_MapRaw.pSignals = new UtilityHNS::AisanSignalFileReader(msg);
}

void GlobalPlanner::callbackGetVMVectors(const vector_map_msgs::VectorArray& msg)
{
  std::cout << "Received Vectors" << msg.data.size() << endl;
  if(m_MapRaw.pVectors  == nullptr)
    m_MapRaw.pVectors = new UtilityHNS::AisanVectorFileReader(msg);
}

void GlobalPlanner::callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg)
{
  std::cout << "Received Curbs" << msg.data.size() << endl;
  if(m_MapRaw.pCurbs == nullptr)
    m_MapRaw.pCurbs = new UtilityHNS::AisanCurbFileReader(msg);
}

void GlobalPlanner::callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg)
{
  std::cout << "Received Edges" << msg.data.size() << endl;
  if(m_MapRaw.pRoadedges  == nullptr)
    m_MapRaw.pRoadedges = new UtilityHNS::AisanRoadEdgeFileReader(msg);
}

void GlobalPlanner::callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg)
{
  std::cout << "Received Wayareas" << msg.data.size() << endl;
  if(m_MapRaw.pWayAreas  == nullptr)
    m_MapRaw.pWayAreas = new UtilityHNS::AisanWayareaFileReader(msg);
}

void GlobalPlanner::callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg)
{
  std::cout << "Received CrossWalks" << msg.data.size() << endl;
  if(m_MapRaw.pCrossWalks == nullptr)
    m_MapRaw.pCrossWalks = new UtilityHNS::AisanCrossWalkFileReader(msg);
}

void GlobalPlanner::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
  std::cout << "Received Nodes" << msg.data.size() << endl;
  if(m_MapRaw.pNodes == nullptr)
    m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
}

}

