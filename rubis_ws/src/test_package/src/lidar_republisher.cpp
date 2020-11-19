#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>



static ros::Subscriber sub;
static ros::Publisher pub;

void points_cb(const sensor_msgs::PointCloud2& msg){
    sensor_msgs::PointCloud2 out;
    out = msg;
    out.header.stamp = ros::Time::now();
    pub.publish(out);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_republisher");
    ros::NodeHandle nh;    
    std::string input_topic;
    nh.param<std::string>("/lidar_republisher/input_topic", input_topic, "/points_raw_origin");

    sub = nh.subscribe(input_topic, 1, points_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/points_raw", 1);    

    while(ros::ok())
        ros::spin();
    
    return 0;
}