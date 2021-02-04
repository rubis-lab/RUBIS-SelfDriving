#include "gnss_localizer_cm.h"
void GPSCallback(const hellocm_msgs::GPS_Out& msg){
    current_time_ = msg.header.stamp.sec + msg.header.stamp.nsec / 1e9;
    cur_pose_.header = msg.header;
    cur_pose_.header.stamp = ros::Time::now();
    cur_pose_.header.frame_id = MAP_FRAME_;
    cur_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_);
    LLH2UTM(msg.latitude, msg.longitude, msg.altitude);
    pose_pub.publish(cur_pose_);
    nav_msgs::Odometry gps_odom_msg;
    gps_odom_msg.header = cur_pose_.header;
    gps_odom_msg.header.frame_id = "odom";
    gps_odom_msg.child_frame_id = "gps";
    gps_odom_msg.pose.pose = cur_pose_.pose;
<<<<<<< HEAD
    gps_odom_msg.pose.covariance = {0.958504205, 0, 0, 0, 0, 0,
                                    0, 0.586713415, 0, 0, 0, 0,
                                    0, 0, 0.000000000001, 0, 0, 0,
                                    0, 0, 0, 0.001, 0, 0,
                                    0, 0, 0, 0, 0.001, 0,
                                    0, 0, 0, 0, 0, 0.001};
    
=======
    gps_odom_msg.pose.covariance = {1, 0, 0, 0, 0, 0,
                                    0, 1, 0, 0, 0, 0,
                                    0, 0, 1, 0, 0, 0,
                                    0, 0, 0, 1, 0, 0,
                                    0, 0, 0, 0, 1, 0,
                                    0, 0, 0, 0, 0, 1};
>>>>>>> 0e4284ba7acf15e96e0ebf0e8bf74595ec182b62
    odom_pub.publish(gps_odom_msg);
    cur_pose_data_.x = cur_pose_.pose.position.x;
    cur_pose_data_.y = cur_pose_.pose.position.y;
    cur_pose_data_.z = cur_pose_.pose.position.z;
    cur_pose_data_.roll = roll_;
    cur_pose_data_.pitch = pitch_;
    cur_pose_data_.yaw = yaw_;
    publishVelocity();
    if(publish_tf_)
        publishTF();
    prev_pose_data_.x = cur_pose_data_.x;
    prev_pose_data_.y = cur_pose_data_.y;
    prev_pose_data_.z = cur_pose_data_.z;
    prev_pose_data_.roll = cur_pose_data_.roll;
    prev_pose_data_.pitch = cur_pose_data_.pitch;
    prev_pose_data_.yaw = cur_pose_data_.yaw;
}
void IMUCallback(const sensor_msgs::Imu& msg){
    roll_ = msg.orientation.x;
    pitch_ = msg.orientation.y;
    yaw_ = msg.orientation.z - 1.648;
}
void LLH2UTM(double Lat, double Long, double H){
    double a = WGS84_A;
    double eccSquared = UTM_E2;
    double k0 = UTM_K0;
    double LongOrigin;
    double eccPrimeSquared;
    double N, T, C, A, M;
    // Make sure the longitude is between -180.00 .. 179.9
    // (JOQ: this is broken for Long < -180, do a real normalize)
    double LongTemp = (Long+180)-int((Long+180)/360)*360-180;
    double LatRad = angles::from_degrees(Lat);
    double LongRad = angles::from_degrees(LongTemp);
    double LongOriginRad;
    cur_pose_.pose.position.z = H;
    // Fix Zone number with Korea
    int zone = 52;
    char band = 'S';
    // +3 puts origin in middle of zone
    LongOrigin = (zone - 1)*6 - 180 + 3;
    LongOriginRad = angles::from_degrees(LongOrigin);
    eccPrimeSquared = (eccSquared)/(1-eccSquared);
    N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
    T = tan(LatRad)*tan(LatRad);
    C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
    A = cos(LatRad)*(LongRad-LongOriginRad);
    M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64
        - 5*eccSquared*eccSquared*eccSquared/256) * LatRad
        - (3*eccSquared/8 + 3*eccSquared*eccSquared/32
        + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
        + (15*eccSquared*eccSquared/256
        + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
        - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));
    cur_pose_.pose.position.y = (double)
    (k0*N*(A+(1-T+C)*A*A*A/6
        + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
    + 500000.0);
    cur_pose_.pose.position.x = (double)
    (k0*(M+N*tan(LatRad)
        *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
        + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));
    // scale down x and y
    cur_pose_.pose.position.x -= 4161000;
    cur_pose_.pose.position.y -= 313000;
    cur_pose_.pose.position.y *= -1;
}
void publishTF()
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(cur_pose_.pose.position.x, cur_pose_.pose.position.y, cur_pose_.pose.position.z));
  tf::Quaternion quaternion;
  quaternion.setRPY(roll_, pitch_, yaw_);
  transform.setRotation(quaternion);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), MAP_FRAME_, "base_link"));
}
void publishVelocity(){
    static int zero_cnt = 0;
    double diff_x, diff_y, diff_z, diff_yaw, diff, current_velocity, angular_velocity;
    diff_x = abs(cur_pose_data_.x - prev_pose_data_.x);
    diff_y = abs(cur_pose_data_.y - prev_pose_data_.y);
    diff_z = abs(cur_pose_data_.z - prev_pose_data_.z);
    diff_yaw = cur_pose_data_.yaw - prev_pose_data_.yaw;
    diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
    if(diff_x == 0 && diff_y == 0 && diff_z == 0){
        if(zero_cnt<=5){
            zero_cnt++;
            return;
        }
    }
    else{
        zero_cnt=0;
    }
    const double diff_time = current_time_ - prev_time_;
    current_velocity = (diff_time > 0) ? (diff / diff_time) : 0;
    angular_velocity = (diff_time > 0) ? (diff_yaw / diff_time) : 0;
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.stamp = ros::Time::now();
    twist_msg.header.frame_id = "base_link";
    twist_msg.twist.linear.x = current_velocity;
    twist_msg.twist.linear.y = 0.0;
    twist_msg.twist.linear.z = 0.0;
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = angular_velocity;
    vel_pub.publish(twist_msg);
    prev_time_ = current_time_;
}
int main(int argc, char** argv){
    ros::init(argc, argv, "gnss_cm_localizer");
    ros::NodeHandle nh;
    gps_sub = nh.subscribe("/gps_out", 100, GPSCallback);
    imu_sub = nh.subscribe("/imu_out", 100, IMUCallback);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/gnss_pose", 10);
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>("gnss_vel", 10);    
    odom_pub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 10);
    nh.param<bool>("/gnss_localizer_cm/publish_tf", publish_tf_, true);
    while(ros::ok())
        ros::spin();
    return 0;
}