#include "PozyxI2c.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "ros/publisher.h"
#include <boost/algorithm/string.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "pozyx_1d_pub");

    ros::NodeHandle n("~");

    ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("/paint/pozyx/laser_scan", 1);

    // Parameters
    std::string i2c_bus_param;
    std::string names_list;
    int rate;

    n.param<std::string>("i2c_bus", i2c_bus_param, "/dev/i2c-5");
    n.param<std::string>("names_list", names_list, "0x00");
    n.param<int>("loop_rate", rate, 100);

    ros::Rate loop_rate(rate);

    std::vector<std::string> results;
    boost::algorithm::split(results, names_list, [](char c){return c == ';';});

    ROS_INFO("Pozyx anchor names list: %s", boost::algorithm::join(results, ",").c_str());

    std::vector<int> parsed_ids;

    for (auto id : results) {
        int parsed_id;
        sscanf(id.c_str(), "%x", &parsed_id);
        parsed_ids.push_back(parsed_id);
    }

    // Initilize Pozyx connection
    PozyxI2c pozyx;

    if (!pozyx.initBus(i2c_bus_param.c_str())) {
        ROS_ERROR("Failed to initialize I2C bus : %s", i2c_bus_param.c_str());
        return -1;
    } else {
        ROS_INFO("I2C bus initialized: %s", i2c_bus_param.c_str());
    }

    int res = 0;
    int error_count = 0;
    device_range_t range;
    sensor_msgs::LaserScan msg;
    msg.ranges = std::vector<float>(parsed_ids.size());

    while (ros::ok()) {
        for (int i = 0 ; i < results.size() ; i++) {
            res = pozyx.doRanging(parsed_ids[i], &range);
            if (res == POZYX_SUCCESS) {
                error_count = 0;
                msg.ranges[i] = range.distance / 1000.0;
            } else if (res == POZYX_TIMEOUT) {
                error_count++;
                msg.ranges[i] = -1;
            } else if (res == POZYX_FAILURE) {
                error_count++;
                msg.ranges[i] = -2;
                ROS_ERROR("Pozyx Error DoRanging! Result=%s", results[i].c_str());
            }
            if (error_count > 10) {
                ROS_ERROR("Pozyx disconnected.");
                return -1;
            }
            msg.header.stamp = ros::Time::now();
            pub.publish(msg);
            loop_rate.sleep();
        }
    }
}
