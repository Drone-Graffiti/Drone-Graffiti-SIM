#include "ros/ros.h"
#include "mavros_msgs/CommandHome.h"
#include "mavros_msgs/StreamRate.h"
#include "geographic_msgs/GeoPointStamped.h"


// ekf set rate
#define EKF_SET_RATE 0.25

#define TELEMETRY_RATE 40

//#define ALT_RANGEFINDER_DIST_FROM_GROUND 0.6f
//#define ALT_RANGEFINDER_DIST_FROM_GROUND 0.6f

void set_ekf_origin();

void set_stream_rate();

// ekf origin publisher
ros::Publisher ekf_origin_publisher;

ros::ServiceClient set_home_position_client;
ros::ServiceClient set_stream_rate_client;

double extnav_latitude = 0;
double extnav_longitude = 0;
double extnav_altitude = 0;


