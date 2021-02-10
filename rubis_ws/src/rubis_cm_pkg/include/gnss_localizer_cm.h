#include <ros/ros.h>
#include <ros/time.h>
#include <hellocm_msgs/GPS_Out.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <fstream>
#include <iostream>


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

#define MIN(x,y) (x <= y ? x : y)
#define MAX(x,y) (x >= y ? x : y)

#define MAP_FRAME_ "map"
#define GPS_FRAME_ "gps"

#define DEBUG_FLAG false

using namespace std;



typedef struct _Point{
    double x;
    double y;
    double z;
}Point;

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

bool publish_tf_;

static int max_zone = -1;
static std::vector< std::vector<double> > zone_points;
static std::vector< std::vector<double> > zone_offset;

void LLH2UTM(double Lat, double Long, double H, geometry_msgs::PoseStamped& pose);
void init_zone_points(std::string filename);
void print_zone_points(std::vector< std::vector<double> >& zone_points);
void init_zone_offset(std::string filename);
void print_zone_offset(std::vector< std::vector<double> >& zone_points);
void add_offset(Point& p);


void publishVelocity();

void GPSCallback(const hellocm_msgs::GPS_Out& msg);
void IMUCallback(const sensor_msgs::Imu& msg);
void publishTF();

inline int PointInsidePolygon(const std::vector<Point>& polygon,const Point& p)
{
    int counter = 0;
      int i;
      double xinters;
      Point p1,p2;
      int N = polygon.size();
      if(N <=0 ) return -1;

      p1 = polygon.at(0);
      for (i=1;i<=N;i++)
      {
        p2 = polygon.at(i % N);

        if (p.y > MIN(p1.y,p2.y))
        {
          if (p.y <= MAX(p1.y,p2.y))
          {
            if (p.x <= MAX(p1.x,p2.x))
            {
              if (p1.y != p2.y)
              {
                xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
                if (p1.x == p2.x || p.x <= xinters)
                  counter++;
              }
            }
          }
        }
        p1 = p2;
      }

      if (counter % 2 == 0)
        return 0;
      else
        return 1;
}
