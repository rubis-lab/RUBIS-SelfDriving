#include <ros/ros.h>
#include <ros/time.h>
#include <hellocm_msgs/GPS_Out.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>

static ros::Subscriber sub;
static ros::Publisher pub;
static int seq;

void cb(const hellocm_msgs::GPS_Out& msg){
    sensor_msgs::NavSatStatus status;    
    status.status = -1;
    status.service = 1;

    sensor_msgs::NavSatFix out;
    out.header.stamp = ros::Time::now();
    out.header.seq = seq;
    out.header.frame_id = "gps";
    out.status = status;

    out.latitude = msg.latitude;
    out.longitude = msg.longitude;
    out.altitude = msg.altitude;
    out.position_covariance_type = 0;

    pub.publish(out);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "gnss_cm_republisher");
    ros::NodeHandle nh;    
    std::string input_topic;
    nh.param<std::string>("/gnss_cm_republisher/input_topic", input_topic, "/gps_out");

    sub = nh.subscribe(input_topic, 1, cb);
    pub = nh.advertise<sensor_msgs::NavSatFix>("/gps_nav", 1);    

    while(ros::ok())
        ros::spin();
    
    return 0;
}