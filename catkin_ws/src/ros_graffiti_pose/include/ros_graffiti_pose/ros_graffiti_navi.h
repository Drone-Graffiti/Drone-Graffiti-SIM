#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/RCOut.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

namespace pt = boost::property_tree;

// CALLBACKS

// ardupilot position callback
void ardu_local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &pos);
// command callback
void command_callback(const std_msgs::String::ConstPtr &cmd);
// rc callback
void rcout_callback(const mavros_msgs::RCOutConstPtr &rcout);

// PUBLISHERS
void set_desired_position_smooth();
void publish_sprayer_state();

// SERVICE CLIENTS
ros::ServiceClient cmd_long_client;
ros::ServiceClient land_client;

// NAV
void set_waypoint_smooth(uint32_t index);

void parse_svg(std::string filename);
void create_path_from_pathes_array();
void create_smooth_path();
void handle_sprayer_servo();
bool user_allows_to_paint = false;
bool allow_paint();
bool moving_to_position = true;
ros::Time painting_start_time;

// VIZ
void publish_wps_visual_smooth();

// desired position publisher
ros::Publisher desired_position_publisher;
// sprayer state publisher
ros::Publisher sprayer_state_publisher;
// waypoints publisher
ros::Publisher waypoints_publisher;
// current wpnum publisher
ros::Publisher wpnum_publisher;

// ardupilot position subscriber
ros::Subscriber ardupilot_local_position_subscriber;
// command subscriber
ros::Subscriber command_subscriber;

ros::Subscriber rcout_subscriber;

// parameters
double distance_to_wall = 1;
std::string svg_file_name;
double max_painting_distance_from_wall;
double max_painting_deviation;
int sprayer_servo_id;
float sprayer_servo_paint_pwm;
float sprayer_servo_stop_pwm;
float wall_left_border;
float wall_right_border;
float wall_upper_border;
float fps = 4;
float speed = 0.05; 

struct WP {
    double x; // hor, pozyx
    double y; // distance_from_wall;
    double z; // alt 
    double speed; // in cm\sec // TODO
    double paint; //0-1
};

std::vector<WP> wps;
std::vector<WP> wps_long;

uint8_t paint_state = 0;
uint32_t current_wp = 0;
uint32_t path_length = 0;

mavros_msgs::RCOut current_rcout;

std::vector<std::string> pathes;

// pose we get from ardupilot.
geometry_msgs::PoseStamped ardupilot_pose;

// MATH
double distance_between_points(double x1, double y1, double x2, double y2);
 
bool is_obtuse_angle(double opposite_line, double a, double b);
 
double distance_to_segment(double ax, double ay, double bx, double by, double x, double y);

float svg_h_offset = 0;
float svg_v_offset = 0;

float svg_h_scale = 1;
float svg_v_scale = 1;

