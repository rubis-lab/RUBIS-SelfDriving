#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <autoware_msgs/Signals.h>
#include <autoware_msgs/ExtractedPosition.h>

int main(int argc, char* argv[]){
    // Initialize
    ros::init(argc, argv, "fake_traffic_signal_generator");
    ros::NodeHandle nh;
    double traffic_light_rate;

    nh.param<double>("/fake_traffic_signal_generator/traffic_light_rate", traffic_light_rate, 1.0);

    ros::Rate rate(traffic_light_rate);

    ros::Publisher traffic_signal_pub;
    ros::Publisher stop_line_rviz_pub;

    traffic_signal_pub = nh.advertise<autoware_msgs::Signals>("/roi_signal", 10);
    stop_line_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>("/stop_line_marker", 10);

    // Add Traffic Signal Info from yaml file
    XmlRpc::XmlRpcValue traffic_light_list;
    nh.getParam("/traffic_light_list", traffic_light_list);

    // Add Traffic Signal Info from yaml file
    XmlRpc::XmlRpcValue stop_line_list;
    nh.getParam("/stop_line_list", stop_line_list);

    visualization_msgs::MarkerArray stop_line_marker_array;

    for(int i=0; i<stop_line_list.size(); i++){
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();

        int id = stop_line_list[i]["id"];
        marker.id = id;
        marker.ns = std::to_string(id);

        marker.pose.position.x = stop_line_list[i]["pose"]["x"];
        marker.pose.position.y = stop_line_list[i]["pose"]["y"];
        marker.pose.position.z = stop_line_list[i]["pose"]["z"];
        marker.pose.orientation.w = 1;

        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        visualization_msgs::Marker text_marker;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.header.frame_id = "world";
        text_marker.ns = "text";
        text_marker.header.stamp = ros::Time::now();
        text_marker.id = -id;
        text_marker.text = "StopLine " + std::to_string(id);
        text_marker.pose = marker.pose;
        text_marker.scale.z = 2;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        stop_line_marker_array.markers.push_back(marker);
        stop_line_marker_array.markers.push_back(text_marker);
    }

    // Make signal msg
    bool isGreen = true;
    autoware_msgs::Signals roi_signal;
    for(int i=0; i<stop_line_list.size(); i++){
        autoware_msgs::ExtractedPosition sig;
        sig.signalId = stop_line_list[i]["tl_id"];
        roi_signal.Signals.push_back(sig);
    }

    while(ros::ok()){
        for(int i=0; i<stop_line_list.size(); i++){
            if(isGreen){
                stop_line_marker_array.markers.at(2*i).color.r = 0.0f;
                stop_line_marker_array.markers.at(2*i).color.g = 1.0f;
                roi_signal.Signals.at(i).type = 1; // Green
            }
            else{
                stop_line_marker_array.markers.at(2*i).color.r = 1.0f;
                stop_line_marker_array.markers.at(2*i).color.g = 0.0f;
                roi_signal.Signals.at(i).type = 2; // Red
            }
        }
        isGreen = !isGreen;

        traffic_signal_pub.publish(roi_signal);
        stop_line_rviz_pub.publish(stop_line_marker_array);
        rate.sleep();
    }

    return 0;
}