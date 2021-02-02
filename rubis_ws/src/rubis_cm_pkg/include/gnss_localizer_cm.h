#include <ros/ros.h>
#include <ros/time.h>
#include <hellocm_msgs/GPS_Out.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <angles/angles.h>
#include <tf/tf.h>

#define WGS84_A		6378137.0		// major axis
#define WGS84_B		6356752.31424518	// minor axis
#define WGS84_F		0.0033528107		// ellipsoid flattening
#define WGS84_E		0.0818191908		// first eccentricity
#define WGS84_EP	0.0820944379		// second eccentricity

// UTM Parameters
#define UTM_K0		0.9996			// scale factor
#define UTM_FE		500000.0		// false easting
#define UTM_FN_N	0.0           // false northing, northern hemisphere
#define UTM_FN_S	10000000.0    // false northing, southern hemisphere
#define UTM_E2		(WGS84_E*WGS84_E)	// e^2
#define UTM_E4		(UTM_E2*UTM_E2)		// e^4
#define UTM_E6		(UTM_E4*UTM_E2)		// e^6
#define UTM_EP2		(UTM_E2/(1-UTM_E2))	// e'^2

#define MAP_FRAME_ "map"

struct Pose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

ros::Subscriber gps_sub, imu_sub;
ros::Publisher pose_pub, vel_pub;

double roll_, pitch_, yaw_;

double current_time_, prev_time_;
geometry_msgs::PoseStamped cur_pose_;
Pose cur_pose_data_, prev_pose_data_;

void LLH2UTM(double Lat, double Long, double H);
void publishVelocity();

void GPSCallback(const hellocm_msgs::GPS_Out& msg);
void IMUCallback(const sensor_msgs::Imu& msg);