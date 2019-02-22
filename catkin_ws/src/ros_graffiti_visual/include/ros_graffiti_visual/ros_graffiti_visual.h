#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/LaserScan.h"
#include "teraranger_array/RangeArray.h"
#include "geometry_msgs/PoseStamped.h"

#include <iomanip>
#include <sstream>



void publish_wall();
void publish_tera();
void publish_pozyx();
void publish_alt();

ros::Publisher wall_visual_publisher;
ros::Publisher text_publisher;


// Rangers array subscriber, for left and right lazer mesurments.
ros::Subscriber laser_array_subscriber;
// altitude ranger subscriber
ros::Subscriber altitude_subscriber;
// pozyx subscriber
ros::Subscriber pozyx_subscriber;
// ardupilot position subscriber
ros::Subscriber ardupilot_local_position_subscriber;

sensor_msgs::LaserScan altitude;
teraranger_array::RangeArray tera;
sensor_msgs::LaserScan pozyx;
geometry_msgs::PoseStamped ardu;

// CALLBACKS

// Teraranger array callbac
void teraranger_callback(const teraranger_array::RangeArray::ConstPtr &scan);

// altitude ranger callback
void alt_range_callback(const sensor_msgs::LaserScan::ConstPtr &scan);

// pozyx callback
void pozyx_callback(const sensor_msgs::LaserScan::ConstPtr &scan);

// ardupilot position callback
void ardu_local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &pos);

std::string float2str(float v);