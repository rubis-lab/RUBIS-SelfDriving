#include <ros/ros.h>
#include <tf/tf.h>

#include <sstream>
#include <fstream>
#include <iostream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <angles/angles.h>
#include <geographic_msgs/GeoPoint.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <autoware_config_msgs/ConfigNDT.h>

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

#define POSE_MODE 0
#define GPS_MODE 1

#define DEBUG_FLAG false


using namespace std;

int mode = -1;

typedef struct _Point{
    double x;
    double y;
    double z;
}Point;

static int max_zone = -1;
ros::Publisher pub;
ros::Publisher ndt_config_pub;
ros::Subscriber sub;
static std::vector< std::vector<double> > zone_points;
static std::vector< std::vector<double> > zone_offset;
static double roll_, pitch_, yaw_;

void LLH2UTM(double Lat, double Long, double H, geometry_msgs::Pose& pose);
void create_gps_waypoints(std::string waypoint_filename, geometry_msgs::PoseArray& out);
void create_pose_waypoints(std::string waypoint_filename, geometry_msgs::PoseArray& out);
void init_zone_points(std::string filename);
void print_zone_points(std::vector< std::vector<double> >& zone_points);
void init_zone_offset(std::string filename);
void print_zone_offset(std::vector< std::vector<double> >& zone_points);
void add_offset(Point& p);
void imu_cb(const sensor_msgs::Imu& msg);
void test();

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

void LLH2UTM(double Lat, double Long, double H, geometry_msgs::Pose& pose){
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
    pose.position.z = H;
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
    pose.position.y = (double)
    (k0*N*(A+(1-T+C)*A*A*A/6
        + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
    + 500000.0);
    pose.position.x = (double)
    (k0*(M+N*tan(LatRad)
        *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
        + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));
    
    double TM[4][4] = 
    {{-0.821456, -0.593423, -0.006448, 3606301.475406},
    {-0.596954, 0.803991, -0.096993, 2231713.639404},
    {0.049875, 0.018177, -0.047063, -213252.081285},
    {0.000000, 0.000000, 0.000000, 1.000000}};

    double input[4] = {pose.position.x, pose.position.y, pose.position.z, 1};
    pose.position.x = TM[0][0]*input[0] + TM[0][1]*input[1] + TM[0][2]*input[2] + TM[0][3]*input[3];
    pose.position.y = TM[1][0]*input[0] + TM[1][1]*input[1] + TM[1][2]*input[2] + TM[1][3]*input[3];
    pose.position.z = TM[2][0]*input[0] + TM[2][1]*input[1] + TM[2][2]*input[2] + TM[2][3]*input[3];    

    // Add offset
    Point p;
    p.x = pose.position.x;
    p.y = pose.position.y;
    p.z = pose.position.z;
    add_offset(p);
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = p.z;
}

void init_zone_points(std::string filename){
    ifstream inputFile(filename);
    int l=0;

    vector< vector<std::string> > data;

    while(inputFile){
        
        string s;
        if (!getline(inputFile, s)) break;
        l++;
        if (s[0] != '#') {
            
            istringstream ss(s);
            vector<string> record;

            while (ss) {
                string line;
                if (!getline(ss, line, ','))
                    break;
                try {
                    record.push_back(line);
                }
                catch (const std::invalid_argument e) {
                    cout << "NaN found in file " << filename << " line " << l
                         << endl;
                    e.what();
                }
            }
            data.push_back(record);
        }
        else{
            l--;
        }
    }
    max_zone = l;

    for(auto it = data.begin(); it != data.end(); ++it){
        std::vector<string> data_line = *it;
        std::vector<double> zone_line;
        zone_line.push_back(stof(data_line[1])); // lt x
        zone_line.push_back(stof(data_line[2])); // lt y
        zone_line.push_back(stof(data_line[3])); // rt x
        zone_line.push_back(stof(data_line[4])); // rt y
        zone_line.push_back(stof(data_line[5])); // rb x
        zone_line.push_back(stof(data_line[6])); // rb y
        zone_line.push_back(stof(data_line[7])); // lb x
        zone_line.push_back(stof(data_line[8])); // lb y
        zone_points.push_back(zone_line);
    }

}

void print_zone_points(std::vector< std::vector<double> >& zone_points){
    std::cout<<"[Print Zone Points!]"<<std::endl;
    for(auto it = zone_points.begin(); it != zone_points.end(); ++it){
        for(int i = 0; i <8; ++i){
            std::cout<<(*it)[i]<<" ";
        }
        std::cout<<std::endl;
    }
}

void init_zone_offset(std::string filename){
    ifstream inputFile(filename);
    int l=0;

    vector< vector<std::string> > data;

    while(inputFile){
        l++;
        string s;
        if (!getline(inputFile, s)) break;
        if (s[0] != '#') {
            istringstream ss(s);
            vector<string> record;

            while (ss) {
                string line;
                if (!getline(ss, line, ','))
                    break;
                try {
                    record.push_back(line);
                }
                catch (const std::invalid_argument e) {
                    cout << "NaN found in file " << filename << " line " << l
                         << endl;
                    e.what();
                }
            }
            data.push_back(record);
        }
    }

    for(auto it = data.begin(); it != data.end(); ++it){
        std::vector<string> data_line = *it;
        std::vector<double> offset_line;
        offset_line.push_back(stof(data_line[1])); // offset x
        offset_line.push_back(stof(data_line[2])); // offset y
        offset_line.push_back(stof(data_line[3])); // offset z
        zone_offset.push_back(offset_line);
    }

}

void print_zone_offset(std::vector< std::vector<double> >& zone_offset){
    std::cout<<"[Print Zone Offset!]"<<std::endl;
    for(auto it = zone_offset.begin(); it != zone_offset.end(); ++it){
        for(int i = 0; i <3; ++i){
            std::cout<<(*it)[i]<<" ";
        }
        std::cout<<std::endl;
    }
}

void add_offset(Point& p){
    int isZoneFound = 0;
    int target_zone = -1;
    for(int i = 0; i < max_zone; i++){
        
        std::vector<double> line = zone_points[i];
        std::vector<Point> polygon;
        Point pol_point1 = {line[0], line[1], 0.0};
        Point pol_point2 = {line[2], line[3], 0.0};
        Point pol_point3 = {line[4], line[5], 0.0};
        Point pol_point4 = {line[6], line[7], 0.0};
        polygon.push_back(pol_point1);
        polygon.push_back(pol_point2);
        polygon.push_back(pol_point3);
        polygon.push_back(pol_point4);

        isZoneFound = PointInsidePolygon(polygon, p);

        if(isZoneFound){
            target_zone = i;
            break;
        }
    }

    if(DEBUG_FLAG) std::cout<<"Target Zone: "<<target_zone<<std::endl;
    if(isZoneFound){
        p.x += zone_offset[target_zone][0];
        p.y += zone_offset[target_zone][1];
        p.z += zone_offset[target_zone][2];
    }

}

void create_gps_waypoints(std::string waypoint_filename, geometry_msgs::PoseArray& out){
    static int seq;
    ifstream inputFile(waypoint_filename);
    int l=0;

    vector< vector<string> > data;

    while(inputFile){
        l++;
        string s;
        if (!getline(inputFile, s))
            break;
        if (s[0] != '#') {
            istringstream ss(s);
            vector<string> record;

            while (ss) {
                string line;
                if (!getline(ss, line, ','))
                    break;
                try {
                    record.push_back(line);
                }
                catch (const std::invalid_argument e) {
                    cout << "NaN found in file " << waypoint_filename << " line " << l
                         << endl;
                    e.what();
                }
            }
            data.push_back(record);
        }
    }

    out.header.stamp = ros::Time::now();
    out.header.seq = seq;
    out.header.frame_id = "map";

    static int ndt_pub_cnt = 0;
    
    if(DEBUG_FLAG) std::cout<<"================================================="<<std::endl;
    for(auto it = data.begin(); it != data.end(); ++it){
        vector<string> line = *it;
        geometry_msgs::Pose pose;
        std::string category = line[0];
        double lat = stof(line[2]);
        double lon = stof(line[3]);
        double h = stof(line[1]);
        if(DEBUG_FLAG) std::cout<<"GPS waypoint: "<<category<<" "<<lat<<" "<<lon<<" "<<h<<std::endl;
        LLH2UTM(lat, lon, h, pose);

        if(category=="Start" && ndt_pub_cnt < 30){
            autoware_config_msgs::ConfigNDT config_msg;
            config_msg.init_pos_gnss = 0;
            config_msg.x = pose.position.x;
            config_msg.y = pose.position.y;
            config_msg.z = pose.position.z;
            config_msg.roll = roll_;
            config_msg.pitch = pitch_;
            config_msg.yaw = yaw_;
            config_msg.use_predict_pose = 1;
            config_msg.error_threshold = 1.0;
            config_msg.resolution = 10.0;
            config_msg.step_size = 0.5;
            config_msg.trans_epsilon = 0.01;
            config_msg.max_iterations = 100;

            ndt_config_pub.publish(config_msg);
            ndt_pub_cnt++;
        }        
        else if(category!="Start"){
            out.poses.push_back(pose);
        }        
    }
    pub.publish(out);
}

void create_pose_waypoints(std::string waypoint_filename, geometry_msgs::PoseArray& out){
    static int seq;
    ifstream inputFile(waypoint_filename);
    int l=0;

    vector< vector<string> > data;

    while(inputFile){
        l++;
        string s;
        if (!getline(inputFile, s))
            break;
        if (s[0] != '#') {
            istringstream ss(s);
            vector<string> record;

            while (ss) {
                string line;
                if (!getline(ss, line, ','))
                    break;
                try {
                    record.push_back(line);
                }
                catch (const std::invalid_argument e) {
                    cout << "NaN found in file " << waypoint_filename << " line " << l
                         << endl;
                    e.what();
                }
            }
            data.push_back(record);
        }
    }
    
    out.header.stamp = ros::Time::now();
    out.header.seq = seq;
    out.header.frame_id = "map";

    static int ndt_pub_cnt = 0;

    if(DEBUG_FLAG) std::cout<<"================================================="<<std::endl;
    for(auto it = data.begin(); it != data.end(); ++it){
        vector<string> line = *it;
        std::string category = line[0];
        geometry_msgs::Pose pose;
        pose.position.x = stof(line[1]);
        pose.position.y = stof(line[2]);
        pose.position.z = stof(line[3]);

        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0.6691;
        pose.orientation.w = 0.7431;

        if(DEBUG_FLAG) std::cout<<"Pose waypoint: "<<category<<" "<<pose.position.x<<" "<<pose.position.y<<" "<<std::endl;
        if(category == "Start" && ndt_pub_cnt < 30){
            autoware_config_msgs::ConfigNDT config_msg;
            config_msg.init_pos_gnss = 0;
            config_msg.x = pose.position.x;
            config_msg.y = pose.position.y;
            config_msg.z = pose.position.z;
            config_msg.roll = roll_;
            config_msg.pitch = pitch_;
            config_msg.yaw = yaw_;
            config_msg.use_predict_pose = 1;
            config_msg.error_threshold = 1.0;
            config_msg.resolution = 10.0;
            config_msg.step_size = 0.5;
            config_msg.trans_epsilon = 0.01;
            config_msg.max_iterations = 100;

            ndt_config_pub.publish(config_msg);
            ndt_pub_cnt++;

        }
        else if (category!="Start"){
            out.poses.push_back(pose);
        }
    }
    pub.publish(out);
}


void test(){
    Point z1 = {1920.635,   -990.108,   -0.276};
    Point z2 = {1934.372,	-670.724,	-5.613};
    Point z3 = {1944.662,	-350.788,	-10.75};
    Point z4 = {1955.783,	-54.138,	-15.609};
    Point z5 = {1963.3,	    146.549,	-18.949};
    Point z6 = {1973.047,	405.937,	-23.696};
    Point z7 = {1982.455,	700.686,	-27.982};
    Point z8 = {1694.887,	725.675,	-13.846};
    Point z9 = {1660.789,	456.817,	-8.263};
    Point z10 = {1632.052,	177.283,	-2.567};
    Point z11 = {1612.564,	-20.432,	1.128};
    Point z12 = {1605.283,	-320.892,	5.86};
    Point z13 = {1631.438,	-644.327,	9.209};
    Point z14 = {1652.356,	-920.857,	11.93};
    Point z15 = {2272.361,	-55.788,	-31.722};
    Point z16 = {2288.847,	137.241,	-35.229};
    Point z17 = {2252.821,	351.01,	    -37.476};
    
    std::vector<Point> test_data;
    test_data.push_back(z1);
    test_data.push_back(z2);
    test_data.push_back(z3);
    test_data.push_back(z4);
    test_data.push_back(z5);
    test_data.push_back(z6);
    test_data.push_back(z7);
    test_data.push_back(z8);
    test_data.push_back(z9);
    test_data.push_back(z10);
    test_data.push_back(z11);
    test_data.push_back(z12);
    test_data.push_back(z13);
    test_data.push_back(z14);
    test_data.push_back(z15);
    test_data.push_back(z16);
    test_data.push_back(z17);
    
    for(int i = 0; i < 17; i++){
        std::cout<<"[Zone "<<i<<"]"<<std::endl;
        std::cout<<"Before - "<<test_data[i].x<<" "<<test_data[i].y<<" "<<test_data[i].z<<std::endl;
        add_offset(test_data[i]);
        std::cout<<"After - "<<test_data[i].x<<" "<<test_data[i].y<<" "<<test_data[i].z<<std::endl;
    }
    std::cout<<"================================"<<std::endl;

}

void imu_cb(const sensor_msgs::Imu& msg){
    roll_ = msg.orientation.x;
    pitch_ = msg.orientation.y;
    yaw_ = msg.orientation.z+2.181;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "waypoint_generator");
    ros::NodeHandle nh;
    std::string waypoint_filename;
    std::string zone_points_filename;
    std::string zone_offset_filename;
    ros::Rate r(10);
    

    pub = nh.advertise<geometry_msgs::PoseArray>("/global_waypoints", 1);
    ndt_config_pub = nh.advertise<autoware_config_msgs::ConfigNDT>("/config/ndt",1);

    sub = nh.subscribe("/imu_out", 100, imu_cb);

    nh.param<std::string>("/waypoint_generator/waypoint_filename", waypoint_filename, "none");
    if(waypoint_filename == "none"){
        ROS_ERROR("Can't find waypoint file %s!", waypoint_filename.c_str());
    }

    nh.param<std::string>("/waypoint_generator/zone_points_filename", zone_points_filename, "none");
    if(zone_points_filename == "none"){
        ROS_ERROR("Can't find zone points file %s!", zone_points_filename.c_str());
    }

    nh.param<std::string>("/waypoint_generator/zone_offset_filename", zone_offset_filename, "none");
    if(zone_offset_filename == "none"){
        ROS_ERROR("Can't find zone points file %s!", zone_offset_filename.c_str());
    }
    
    nh.param<int>("/waypoint_generator/mode", mode, -1);
    if(mode != GPS_MODE && mode != POSE_MODE){
        ROS_ERROR("Mode parameter shuld be POSE(0) or GPS(1)");
    }

    
    init_zone_points(zone_points_filename);
        init_zone_offset(zone_offset_filename);
    if(DEBUG_FLAG){
        print_zone_points(zone_points);
        std::cout<<"================================"<<std::endl;  
        std::cout<<endl;
        print_zone_offset(zone_offset);
        std::cout<<"================================"<<std::endl;    
    }
    

    // if(DEBUG_FLAG) test();

    while(ros::ok()){
        geometry_msgs::PoseArray out;
        if(mode == GPS_MODE)
            create_gps_waypoints(waypoint_filename, out);
        else if(mode == POSE_MODE)
            create_pose_waypoints(waypoint_filename, out);
        ros::spinOnce();
        r.sleep();
        
    }

    

    return 0;
}
