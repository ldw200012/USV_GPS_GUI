#include "lla2utm_converter.h"
using namespace std;

bool location_initialized = 0;
// Origin Point
double bx = 0.0;//-594929.9431329881;//east
double by = 0.0;//-4139043.529676078;//north
double bz = 0.0;//unit in m

ULConverter::ULConverter(string hemi, int zone, double at, double fla, double k0):tm_(at,fla,k0),zone_(zone), hemi_(hemi){};

void ULConverter::RegiHandle(ros::NodeHandle &n)
{
  //gps_pub_ = n.advertise<sensor_msgs::NavSatFix>("/gps",10);
  xyz_pub_ = n.advertise<geometry_msgs::PoseStamped>("/Heron_UTMPose",10);
  path_pub_ = n.advertise<nav_msgs::Path>("/Heron_PathTrack",10);
};

void ULConverter::GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msgs)
{
  ROS_INFO("Get data lagitude from GPS: [%f]", msgs->latitude);
  ROS_INFO("Get data longitude from GPS: [%f]", msgs->longitude);
  ROS_INFO("Get data altitude from GPS: [%f]", msgs->altitude);

  if(hemi_ == "North")
    LLAConvert2UTM(NorthH, zone_, msgs->latitude, msgs->longitude, msgs->altitude);
  else if(hemi_ == "South")
    LLAConvert2UTM(SouthH, zone_, msgs->latitude, msgs->longitude, msgs->altitude);
  else{
    ROS_ERROR("only North or South Hemisphere. Are you on Mars?");
    ros::shutdown();
  }

  // ROS_INFO("Range UTM Zone: [%d]", ((zone_*6)-180));

  if(msgs->latitude == 0.0 && msgs->longitude == 0.0 && msgs->altitude == 0.0){
    ROS_INFO("The GPS data is not valid | Case 0: Zero Values from GPS");
    utm_.clear();
  }else if(msgs->longitude > ((zone_*6)-180) || msgs->longitude < (((zone_*6)-180)-6)){ // Check UTM Zone 52
    ROS_INFO("The GPS data is not valid | Case 1: Measurement out of UTM Zone");
    utm_.clear();
  }else{
    // Local Position :: PoseStamped
    geometry_msgs::PoseStamped local_pose;

    local_pose.header = msgs->header;    
    local_pose.pose.position.x = utm_[0] + bx;
    local_pose.pose.position.y = utm_[1] + by;
    local_pose.pose.position.z = utm_[2] + bz;

    ROS_INFO("Convert to x: [%f]", local_pose.pose.position.x);
    ROS_INFO("Convert to y: [%f]", local_pose.pose.position.y);
    ROS_INFO("Convert to z: [%f]", local_pose.pose.position.z);

    xyz_pub_.publish(local_pose);

    // Path accumulate :: Path
    nav_msgs::Path local_path;
    local_path.header = msgs->header;
    path_.push_back(local_pose);
    local_path.poses = path_;
    path_pub_.publish(local_path);

    // Transform :: TransformBroadcaster
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z));
    transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "m8t_base", "/Heron_TF"));

    utm_.clear();
  }
}

void ULConverter::LLAConvert2UTM(Hemi hemi, int zone, double latitude, double longitude, double height){
  double east,north;
 
  int lon0  = zone*6-183;

  tm_.Forward(lon0, latitude, longitude, east, north);

  east += kE0_;
  north += (hemi==NorthH) ? kNN_ : kNS_;

  utm_.push_back(east); utm_.push_back(north); utm_.push_back(height);
}

std::vector<double> ULConverter::get_lla()
{
  return lla_;
}
