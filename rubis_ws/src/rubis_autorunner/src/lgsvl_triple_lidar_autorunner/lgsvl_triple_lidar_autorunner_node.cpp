#include <lgsvl_triple_lidar_autorunner/lgsvl_triple_lidar_autorunner.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "lgsvl_triple_lidar_autorunner_node");
    ros::NodeHandle nh;

    LaneKeepingAutorunner lane_keeping_autorunner(nh);
    lane_keeping_autorunner.Run();

    return 0;
}