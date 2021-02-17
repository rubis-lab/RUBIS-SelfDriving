#include "gnss_localizer_cm.h"

void GPSCallback(const hellocm_msgs::GPS_Out& msg){
    current_time_ = msg.header.stamp.sec + msg.header.stamp.nsec / 1e9;
    cur_pose_.header = msg.header;
    cur_pose_.header.stamp = ros::Time::now();
    cur_pose_.header.frame_id = MAP_FRAME_;
    cur_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_);
    LLH2UTM(msg.latitude, msg.longitude, msg.altitude, cur_pose_);
    pose_pub.publish(cur_pose_);

    publishVelocity();
    
    publishTF();
    prev_pose_data_.x = cur_pose_data_.x;
    prev_pose_data_.y = cur_pose_data_.y;
    prev_pose_data_.z = cur_pose_data_.z;
    prev_pose_data_.roll = cur_pose_data_.roll;
    prev_pose_data_.pitch = cur_pose_data_.pitch;
    prev_pose_data_.yaw = cur_pose_data_.yaw ;
}
void IMUCallback(const sensor_msgs::Imu& msg){
    roll_ = msg.orientation.x;
    pitch_ = msg.orientation.y;
    yaw_ = msg.orientation.z+2.181;
    //yaw_ *= -1;
}

void LLH2UTM(double Lat, double Long, double H, geometry_msgs::PoseStamped& pose){
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
    pose.pose.position.z = H;
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
    pose.pose.position.y = (double)
    (k0*N*(A+(1-T+C)*A*A*A/6
        + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
    + 500000.0);
    pose.pose.position.x = (double)
    (k0*(M+N*tan(LatRad)
        *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
        + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));
    
    double TM[4][4] = 
    {{-0.821456, -0.593423, -0.006448, 3606301.475406},
    {-0.596954, 0.803991, -0.096993, 2231713.639404},
    {0.049875, 0.018177, -0.047063, -213252.081285},
    {0.000000, 0.000000, 0.000000, 1.000000}};

    double input[4] = {pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 1};
    pose.pose.position.x = TM[0][0]*input[0] + TM[0][1]*input[1] + TM[0][2]*input[2] + TM[0][3]*input[3];
    pose.pose.position.y = TM[1][0]*input[0] + TM[1][1]*input[1] + TM[1][2]*input[2] + TM[1][3]*input[3];
    pose.pose.position.z = TM[2][0]*input[0] + TM[2][1]*input[1] + TM[2][2]*input[2] + TM[2][3]*input[3];    

    // Add offset
    Point p;
    p.x = pose.pose.position.x;
    p.y = pose.pose.position.y;
    p.z = pose.pose.position.z;
    add_offset(p);
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.position.z = p.z;
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

void publishTF()
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(cur_pose_.pose.position.x-0.9, cur_pose_.pose.position.y, cur_pose_.pose.position.z-1.5));
  tf::Quaternion quaternion;
  quaternion.setRPY(roll_, pitch_, yaw_);
  transform.setRotation(quaternion);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), MAP_FRAME_, "gps"));
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
    twist_msg.header.frame_id = "gps";
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
    std::string zone_points_filename;
    std::string zone_offset_filename;

    ros::init(argc, argv, "gnss_cm_localizer");
    ros::NodeHandle nh;
    gps_sub = nh.subscribe("/gps_out_noise", 100, GPSCallback);
    imu_sub = nh.subscribe("/imu_out", 100, IMUCallback);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/gnss_pose", 10);
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>("gnss_vel", 10);    

    nh.param<bool>("/gnss_localizer_cm/publish_tf", publish_tf_, true);
  
    nh.param<std::string>("/gnss_localizer_cm/zone_points_filename", zone_points_filename, "none");
    if(zone_points_filename == "none"){
        ROS_ERROR("Can't find zone points file %s!", zone_points_filename.c_str());
    }

    nh.param<std::string>("/gnss_localizer_cm/zone_offset_filename", zone_offset_filename, "none");
    if(zone_offset_filename == "none"){
        ROS_ERROR("Can't find zone points file %s!", zone_offset_filename.c_str());
    }

    std::cout<<"================================"<<std::endl;
    init_zone_points(zone_points_filename);
    print_zone_points(zone_points);

    std::cout<<endl;

    init_zone_offset(zone_offset_filename);
    print_zone_offset(zone_offset);
    std::cout<<"================================"<<std::endl;

    while(ros::ok())
        ros::spin();

    return 0;
}