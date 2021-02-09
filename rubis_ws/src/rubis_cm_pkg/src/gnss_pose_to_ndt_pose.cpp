#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>

static ros::Subscriber sub;
static ros::Publisher pub;


void cb(geometry_msgs::PoseStamped msg){
  double T[4][4] = 
    {{-0.821456, -0.593423, -0.006448, 3606301.475406},
    {-0.596954, 0.803991, -0.096993, 2231713.639404},
    {0.049875, 0.018177, -0.047063, -213252.081285},
    {0.000000, 0.000000, 0.000000, 1.000000}};

  double input[4] = {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 1};

  geometry_msgs::PoseStamped out;
  out.header = msg.header;
  out.pose.position.x = T[0][0]*input[0] + T[0][1]*input[1] + T[0][2]*input[2] + T[0][3]*input[3];
  out.pose.position.y = T[1][0]*input[0] + T[1][1]*input[1] + T[1][2]*input[2] + T[1][3]*input[3];
  out.pose.position.z = T[2][0]*input[0] + T[2][1]*input[1] + T[2][2]*input[2] + T[2][3]*input[3];

  pub.publish(out);
}




int main(int argc, char* argv[]){
    ros::init(argc, argv, "gnss_pose_to_ndt_pose");
    ros::NodeHandle nh;
    ros::Rate r(10);

    sub = nh.subscribe("/gnss_pose", 1, cb);
    pub = nh.advertise<geometry_msgs::PoseStamped>("/gnss_ndt_pose", 1);  



    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}