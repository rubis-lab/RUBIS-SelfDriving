#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseArray.h>
#include <angles/angles.h>
#include <geographic_msgs/GeoPoint.h>

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
#define GPS_FRAME_ "gps"


using namespace std;

ros::Publisher pub;

void LLH2UTM(double Lat, double Long, double H, geometry_msgs::Pose& pose);
void create_waypoints(std::string filename, geometry_msgs::PoseArray& out)

;

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

}

void create_waypoints(std::string filename, geometry_msgs::PoseArray& out){
    static int seq;
    ifstream inputFile(filename);
    int l=0;

    vector< vector<string> > data;

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

    out.header.stamp = ros::Time::now();
    out.header.seq = seq;
    out.header.frame_id = "map";

    std::cout<<"========================="<<std::endl;
    for(auto it = data.begin(); it != data.end(); ++it){
        vector<string> line = *it;
        geometry_msgs::Pose pose;
        double lat = stof(line[2]);
        double lon = stof(line[3]);
        double h = stof(line[1]);
        std::cout<<lat<<" "<<lon<<" "<<h<<std::endl;
        LLH2UTM(lat, lon, h, pose);
        out.poses.push_back(pose);
    }
    std::cout<<out.poses.size()<<std::endl;
    pub.publish(out);
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "waypoint_generator");
    ros::NodeHandle nh;
    std::string filename;
    ros::Rate r(10);

    pub = nh.advertise<geometry_msgs::PoseArray>("/global_waypoints", 1);

    nh.param<std::string>("/waypoint_generator/filename", filename, "none");
    if(filename == "none"){
        ROS_ERROR("Can't find open file %s!", filename.c_str());
    }

    while(ros::ok()){
        geometry_msgs::PoseArray out;
        create_waypoints(filename, out);
        r.sleep();
    }

    

    return 0;
}