#include "ros_gazebo_teraranger/ros_gazebo_teraranger.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_teraranger");

    ros::NodeHandle n("~");

    tera_publisher = n.advertise<teraranger_array::RangeArray>("/ranges", 100);
    
    r1_subscriber = n.subscribe("/paint/laser/R1", 100, r1_range_callback);
    r2_subscriber = n.subscribe("/paint/laser/R2", 100, r2_range_callback);
    
    ros::Rate loop_rate(100);

    ROS_INFO("GAZEBO TERARANGER NODE STARTED WITH DELAY=%f", SENSOR_DELAY);

    while (ros::ok()) {
        
        bool has_to_send = true;
        while (has_to_send) {
            if (ranges_queue.size() == 0) {
                has_to_send = false;
                continue;
            }

            teraranger_array::RangeArray r = ranges_queue.front();

           // ROS_INFO(">>>> %f ", (ros::Time::now() - r.header.stamp).toSec())


            // too early to send. wait. 
            if ((ros::Time::now() - r.header.stamp).toSec() < SENSOR_DELAY) {
                has_to_send = false;
            }

            if (has_to_send) {
                ranges_queue.pop();
                r.header.stamp = ros::Time::now();
                tera_publisher.publish(r);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


void r1_range_callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    r1_range.range = scan->ranges[0];
    r1_new = true;
    add_to_buffer();
}

void r2_range_callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    r2_range.range = scan->ranges[0];
    r2_new = true;
    add_to_buffer();
}

void add_to_buffer() {
    if (r1_new && r2_new) {
        teraranger_array::RangeArray r;
        r.ranges.push_back(r1_range);
        r.ranges.push_back(r2_range);
        r.header.stamp = ros::Time::now();

        ranges_queue.push(r);
        r1_new = false;
        r2_new = false;
    }
}