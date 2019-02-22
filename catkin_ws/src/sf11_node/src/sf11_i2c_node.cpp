/*
* Copyright (c) 2016 Carnegie Mellon University, Guilherme Pereira <gpereira@ufmg.br>
            (c) 2018 SPH Engineering, Mike Charikov (mcharikov@ugcs.com)
*
* For License information please see the LICENSE file in the root directory.
*
*/

// S11 Laser rangefinder node- Mike Charikov, (c) 2018

#include <ros/ros.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>    
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sensor_msgs/LaserScan.h>


#define RANGE_ADDRESS 0x00

int busfd;
bool exit_i2c_;

void SigintHandlerI2c(int sig)
{
    exit_i2c_=true;
    close(busfd);
    ros::Duration(0.5).sleep();
    ros::shutdown();
}

static int read_register(int busfd, __uint16_t reg, unsigned char *buf, int bufsize)
{
    unsigned char reg_buf[2];
    int ret;

    reg_buf[0] = (reg >> 0) & 0xFF;
    reg_buf[1] = (reg >> 8) & 0xFF;

    ret = write(busfd, reg_buf, 2);
    if (ret < 0) {
        ROS_ERROR("Failed to write [0x%02x 0x%02x] (reg: %04x).\n", reg_buf[0], reg_buf[1], reg);
        return ret;
    }

    return read(busfd, buf, bufsize);
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "sf11_i2c_node");
    ros::NodeHandle n("~");

    signal(SIGINT, SigintHandlerI2c);

    ros::Rate loop_rate(50);

    std::string topic_name_param;
    std::string i2c_bus_param;
    std::string i2c_addres_param;
    int i2c_address_int;

    n.param<std::string>("topic", topic_name_param, "sf11_out");
    n.param<std::string>("i2c_bus", i2c_bus_param, "/dev/i2c-1");
    n.param<std::string>("i2c_address", i2c_addres_param, "0x66");

    sscanf(i2c_addres_param.c_str(), "%x", &i2c_address_int);

    ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>(topic_name_param,1);

    if ((busfd = open(i2c_bus_param.c_str(), O_RDWR)) < 0) {
        /* ERROR HANDLING: you can check errno to see what went wrong */
        ROS_ERROR("Failed to open the i2c bus");
        return 1;
    }

    if (ioctl(busfd, I2C_SLAVE, i2c_address_int) < 0) {
        ROS_ERROR("Failed to acquire bus access and/or talk to slave.");
        return 1;
    }


    //  set_interface_attribs (fdes, B115200, 0);  // set speed to 115200 bps, 8n1 (no parity)
    // set_blocking (fdes, 1);                    // set blocking

    uint8_t buf[2];
    ros::Time last_time=ros::Time::now();
    exit_i2c_=false;

    float dist;
    // Main loop
    while(ros::ok() && !exit_i2c_) {

        if (read_register(busfd, RANGE_ADDRESS, buf, 2) < 0) {
            ROS_ERROR("Failed to read data" );
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        dist = ((buf[0] << 8) + buf[1]) / 100.0;

        ros::Time now = ros::Time::now();
        ros::Duration duration=now-last_time;
        last_time=now;

        sensor_msgs::LaserScan data;
        data.header.frame_id = "sf11";
        data.header.stamp = ros::Time::now();
        data.scan_time =  duration.toSec();
        data.range_max=100.0;
        data.ranges.push_back(dist);
        data.intensities.push_back((double)1);
        laser_pub.publish(data);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
