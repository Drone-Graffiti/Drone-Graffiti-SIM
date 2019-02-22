#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "teraranger_array/RangeArray.h"
#include <queue>


#define SENSOR_DELAY 0.00015

// CALLBACKS
// left ranger callback
void r1_range_callback(const sensor_msgs::LaserScan::ConstPtr &scan);

// right ranger callback
void r2_range_callback(const sensor_msgs::LaserScan::ConstPtr &scan);

void add_to_buffer();

bool r1_new = true;
bool r2_new = true;

// left ranger subscriber
ros::Subscriber r1_subscriber;
// right ranger subscriber
ros::Subscriber r2_subscriber;

ros::Publisher tera_publisher;

std::queue<teraranger_array::RangeArray> ranges_queue;

sensor_msgs::Range r1_range, r2_range;