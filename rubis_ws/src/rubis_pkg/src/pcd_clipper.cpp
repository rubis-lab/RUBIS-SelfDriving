#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "ros/this_node.h"
#include <math.h>

static ros::Subscriber sub;
static ros::Publisher detection_pcd_pub, localize_pcd_pub;

// for rosparam
bool enableLocalize;
float tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw;
std::string input_topic;
std::string detection_point_output_topic;
std::string localization_point_output_topic;
std::string output_frame_id;
float localization_center_angle;
float localization_viewing_angle;
float detection_center_angle;
float detection_viewing_angle;

bool isInside(float y, float x, float center_angle, float viewing_angle){
    float angle = atan2(fabs(y), fabs(x)) * 57.295779; // deg = rad * 180 / PI
    float angle_from_left = center_angle + viewing_angle / 2;
    float angle_from_right = center_angle - viewing_angle / 2;

    if(x > 0 && y > 0){ // quadrant 1
        if(angle_from_right <= 0 && 90 + angle_from_right < angle) return true;
        if(angle_from_left - 270 > angle && angle_from_right - 270 < angle) return true;
        return false;
    }
    else if(x < 0 && y > 0){ // quadrant 2
        if(angle_from_left >= 360 && 450 - angle_from_left < angle) return true;
        if(90 - angle_from_right > angle && 90 - angle_from_left < angle) return true;
        return false;
    }
    else if(x < 0 && y < 0){ // quadrant 3
        if(angle_from_left >= 450 && angle_from_left - 450 > angle) return true;
        if(angle_from_right - 90 < angle && angle_from_left - 90 > angle) return true;
        return false;
    }
    else if(x > 0 && y < 0){ // quadrant 4
        if(angle_from_right <= -90 && -(90 + angle_from_right) > angle) return true;
        if(270 - angle_from_left < angle && 270 - angle_from_right > angle) return true;
        return false;
    }
    return false;
}


void points_cb(const sensor_msgs::PointCloud2& msg){
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr detection_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr localization_points(new pcl::PointCloud<pcl::PointXYZI>);

    sensor_msgs::PointCloud2 localization_out;
    sensor_msgs::PointCloud2 detection_out;

    pcl::fromROSMsg(msg, *pcl_points);

    for(int i=0; i<(*pcl_points).size(); i++){
        if(enableLocalize && isInside(pcl_points->points[i].x, pcl_points->points[i].y, localization_center_angle, localization_viewing_angle)){
            localization_points->points.push_back(pcl_points->points[i]);
        }

        if(isInside(pcl_points->points[i].x, pcl_points->points[i].y, detection_center_angle, detection_viewing_angle)){
            detection_points->points.push_back(pcl_points->points[i]);
        }
    }

    if(enableLocalize){
        pcl::toROSMsg(*localization_points, localization_out);
        localization_out.header = msg.header;
        localization_out.header.frame_id = output_frame_id;
        localize_pcd_pub.publish(localization_out);
    }

    pcl::toROSMsg(*detection_points, detection_out);
    detection_out.header = msg.header;
    detection_out.header.frame_id = output_frame_id;
    detection_pcd_pub.publish(detection_out);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pcd_clipper");
    ros::NodeHandle nh;    

    // for topic name
    std::string node_name = ros::this_node::getName();
    std::string enableLocalize_name = node_name + "/enableLocalize";
    std::string tf_x_name = node_name + "/tf_x";
    std::string tf_y_name = node_name + "/tf_y";
    std::string tf_z_name = node_name + "/tf_z";
    std::string tf_roll_name = node_name + "/tf_roll";
    std::string tf_pitch_name = node_name + "/tf_pitch";
    std::string tf_yaw_name = node_name + "/tf_yaw";

    std::string input_topic_name = node_name + "/input_topic";
    std::string detection_pcd_topic_name = node_name + "/detection_point_output_topic";
    std::string localization_pcd_topic_name = node_name + "/localization_point_output_topic";

    std::string output_frame_id_topic_name = node_name + "/output_frame_id";

    std::string localization_center_angle_topic_name = node_name + "/localization_center_angle";
    std::string localization_viewing_angle_topic_name = node_name + "/localization_viewing_angle";
    std::string detection_center_angle_topic_name = node_name + "/detection_center_angle";
    std::string detection_viewing_angle_topic_name = node_name + "/detection_viewing_angle";

    nh.param<bool>(enableLocalize_name, enableLocalize, true);
    nh.param<float>(tf_x_name, tf_x, 0);
    nh.param<float>(tf_y_name, tf_y, 0);
    nh.param<float>(tf_z_name, tf_z, 0);
    nh.param<float>(tf_roll_name, tf_roll, 0);
    nh.param<float>(tf_pitch_name, tf_pitch, 0);
    nh.param<float>(tf_yaw_name, tf_yaw, 0);
    nh.param<std::string>(input_topic_name, input_topic, "/points_raw_origin");
    nh.param<std::string>(detection_pcd_topic_name, detection_point_output_topic, "/detection_points_raw");
    nh.param<std::string>(localization_pcd_topic_name, localization_point_output_topic, "/localization_points_raw");

    nh.param<std::string>(output_frame_id_topic_name, output_frame_id, "velodyne");

    nh.param<float>(localization_center_angle_topic_name, localization_center_angle, 0);
    nh.param<float>(localization_viewing_angle_topic_name, localization_viewing_angle, 360);
    nh.param<float>(detection_center_angle_topic_name, detection_center_angle, 0);
    nh.param<float>(detection_viewing_angle_topic_name, detection_viewing_angle, 360);

    sub = nh.subscribe(input_topic, 1, points_cb);

    if(enableLocalize){
        localize_pcd_pub = nh.advertise<sensor_msgs::PointCloud2>(localization_point_output_topic, 1);
    }

    detection_pcd_pub = nh.advertise<sensor_msgs::PointCloud2>(detection_point_output_topic, 1); 

    while(ros::ok())
        ros::spin();
    
    return 0;
}