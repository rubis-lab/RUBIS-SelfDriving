#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>



int main(int argc, char* argv[]){
    ros::init(argc, argv, "data_type_test");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    ros::Publisher bb_pub;

    bb_pub = nh.advertise<jsk_recognition_msgs::BoundingBox>("bb_msg",10);

    jsk_recognition_msgs::BoundingBox bb_msg;


    int seq = 0;
    while(ros::ok())
    {
        std_msgs::Header bb_header;
        bb_header.seq = seq;
        bb_header.stamp = ros::Time::now();
        bb_header.frame_id = "/";
        bb_msg.header = bb_header;


        bb_pub.publish(bb_msg) ;
        seq++;
        rate.sleep();
    }

}