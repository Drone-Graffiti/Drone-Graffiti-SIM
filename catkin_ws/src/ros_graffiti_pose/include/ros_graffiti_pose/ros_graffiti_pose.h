#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/LaserScan.h"
#include "teraranger_array/RangeArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"

#include <list>
#include <vector>

#include <boost/algorithm/string.hpp>

#include <limits>

#include <iostream>
#include <fstream>

#define LEFT_LASER_RANGEFINDER 0
#define RIGHT_LASER_RANGEFINDER 1

// half of angle between horizontal laser sensors
#define SENSORS_HALF_ANGLE 0.5235987

//#define ALT_RANGEFINDER_DIST_FROM_GROUND 0.6f
//#define ALT_RANGEFINDER_DIST_FROM_GROUND 0.6f


// CALLBACKS

// Teraranger array callbac
void teraranger_callback(const teraranger_array::RangeArray::ConstPtr &scan);

// altitude ranger callback
void alt_range_callback(const sensor_msgs::LaserScan::ConstPtr &scan);

// pozyx callback
void pozyx_callback(const sensor_msgs::LaserScan::ConstPtr &scan);

// ardupilot position callback
void ardu_local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &pos);

// gazebo velocities callback 
void gazebo_twist_callback(const geometry_msgs::TwistStamped::ConstPtr &twist);

// gazebo position callback 
void gazebo_position_callback(const geometry_msgs::PoseStamped::ConstPtr &pose);

//optical flow callback
void of_callback(const geometry_msgs::Vector3Stamped::ConstPtr &vec);


// PUBLISHERS
void send_position_to_fake_gps();
void send_position_to_extnav();
void send_velocity_to_extnav();
void send_gazebo_velocity_to_extnav();
void send_pozyx_visualization();
void send_raw_data_logs();

// MATH
void calculate_pose();
bool calculate_x(double &x, double left_laser_range, double right_laser_range);
float calculate_y(std::vector<float> ranges, std::vector<float> places, float &result);

// LOG
void log_pozyx();

// ERR
void handle_errors();

// CVS DATA
void save_position_data_to_csv();


// fake gps position publisher
ros::Publisher fake_gps_position_publisher;

// graffiti (custom) position publisher
ros::Publisher extnav_position_publisher_ned;
ros::Publisher extnav_position_publisher_enu;
ros::Publisher extnav_velocity_publisher_ned;
ros::Publisher pozyx_visual_publisher;
ros::Publisher navi_cmd_publisher;
ros::Publisher log_publisher;

// Rangers array subscriber, for left and right lazer mesurments.
ros::Subscriber laser_array_subscriber;
// altitude ranger subscriber
ros::Subscriber altitude_subscriber;
// pozyx subscriber
ros::Subscriber pozyx_subscriber;
// ardupilot position subscriber
ros::Subscriber ardupilot_local_position_subscriber;
// gazebo velocities subscriber (for testing purposes)
ros::Subscriber gazebo_twist_subscriber;
// gazebo position subscriber (for testing purposes)
ros::Subscriber gazebo_position_subscriber;
// optical flow subscriber
ros::Subscriber of_subscriber;

// laser ranges
double horizontal_laser_ranges[2];
double altitude_range;

// pozyx last message
sensor_msgs::LaserScan pozyx_current_scan;

// parameters
bool use_fakegps = false;
bool use_extnav = false;
int left_laser_index = 1;
int right_laser_index = 2;
double altimeter_pitch_angle;
bool use_velocity = false;
bool send_gazebo_velocity = false;
std::vector<float> anchor_positions;
float pozyx_distance_from_wall;
float wall_left_border;
float wall_right_border;
float wall_upper_border;
bool save_position_data = false;
bool turn_on_raw_logging = false;

// pose we calculated here base on all of our hardware
geometry_msgs::PoseStamped calculated_pose_ned;
geometry_msgs::PoseStamped calculated_pose_enu;

// pose we get from ardupilot.
geometry_msgs::PoseStamped ardupilot_pose;
double ardu_roll, ardu_pitch, ardu_yaw;

// twist & pose from gazebo
geometry_msgs::TwistStamped gazebo_twist;
geometry_msgs::PoseStamped gazebo_pose;

geometry_msgs::Vector3 pos_sum;
geometry_msgs::Vector3 pos_counter;

// Velocity calculation. Position derivative.
geometry_msgs::Vector3 velocity_ned;
geometry_msgs::Point prev_point_ned;
geometry_msgs::Vector3 vel_sum;
geometry_msgs::Vector3 vel_counter;

teraranger_array::RangeArray current_tera_scan;

sensor_msgs::LaserScan current_alt_scan;

geometry_msgs::Vector3Stamped current_of;

ros::Time lastTerarangerUpdate;
ros::Time lastAltitudeUpdate;
ros::Time lastPozyxUpdate;

std::list<float> median_y_list;
std::list<double> median_y_velocity_list;

std_msgs::Float32MultiArray raw_log;

uint16_t pozyx_err_count = 0;
uint16_t tera_err_count = 0;
uint16_t borders_err_count = 0;

bool ardupilot_first_message_recieved = false;

std::ofstream csv_file;
