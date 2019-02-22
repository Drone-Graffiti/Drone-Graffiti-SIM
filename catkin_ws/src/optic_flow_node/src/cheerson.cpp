#include "cheerson.h"
#include <geometry_msgs/Vector3Stamped.h>

using namespace optic_flow;

Cheerson::~Cheerson() {}

Cheerson::Cheerson() {
    ros::NodeHandle n("~");

    ros::Rate loop_rate(50);
    std::string port_name_param;

    n.param<std::string>("cheerson/portname", port_name_param, "/dev/ttyUSB0");
    n.param<std::float_t>("cheerson/flowScaler_x", flowScaler_x, 0.0f);
    n.param<std::float_t>("cheerson/flowScaler_y", flowScaler_y, 0.0f);


    // Serial Port init
    serial_port.setPort(port_name_param);
    serial_port.setBaudrate(19200);
    serial_port.setParity(serial::parity_none);
    serial_port.setStopbits(serial::stopbits_one);
    serial_port.setBytesize(serial::eightbits);
    serial::Timeout to = serial::Timeout::simpleTimeout(SERIAL_TIMEOUT_MS);
    serial_port.setTimeout(to);
    serial_port.open();
    if(!serial_port.isOpen())
    {
      ROS_ERROR("Could not open : %s ", port_name_param.c_str());
      ros::shutdown();
      return;
    }
    of_publisher = n.advertise<geometry_msgs::Vector3Stamped>("/paint/of_out", 1);
}



void Cheerson::spin() {
    ros::Time current_time, last_time;
    last_time = ros::Time::now();

    while(ros::ok()) {
        int32_t x_sum = 0;
        int32_t y_sum = 0;
        uint16_t qual_sum = 0;
        uint16_t count = 0;
        uint8_t buffer[10];

        int16_t nbytes = serial_port.read(buffer, 10);

        current_time = ros::Time::now();

        for (uint8_t i = 0; i < nbytes; i++) {
            uint8_t c = (uint8_t)buffer[i];
            // if buffer is empty and this byte is header, add to buffer
            if (buf_len == 0) {
                if (c == CXOF_HEADER) {
                   buf[buf_len++] = c;
                }
            } else {
                // add character to buffer
                buf[buf_len++] = c;

                // if buffer has 9 items try to decode it
                if (buf_len >= CXOF_FRAME_LENGTH) {
                    // check last character matches footer
                    if (buf[buf_len-1] != CXOF_FOOTER) {
                        buf_len = 0;
                        continue;
                    }

                    // decode package
                    int16_t x_raw = (int16_t)((uint16_t)buf[3] << 8) | buf[2];
                    int16_t y_raw = (int16_t)((uint16_t)buf[5] << 8) | buf[4];

                    // add to sum of all readings from sensor this iteration
                    count++;
                    x_sum += x_raw;
                    y_sum += y_raw;
                    qual_sum += buf[7];

                    // clear buffer
                    buf_len = 0;
                }
            }
        }

        double dt = (current_time - last_time).toSec();
        last_time = current_time;

        // sanity check dt
        if (dt > 0 ) {
            geometry_msgs::Vector3Stamped msg;

            msg.header.stamp = current_time;
            msg.header.frame_id = "cheerson";

            // calculate flow values
            float flowScaleFactorX = 1.0f + 0.001f * flowScaler_x;
            float flowScaleFactorY = 1.0f + 0.001f * flowScaler_y;

            // copy flow rates to state structure
            msg.vector.x = ((float)x_sum / count) * flowScaleFactorX;
            msg.vector.y = ((float)y_sum / count) * flowScaleFactorY;

          //  msg.vector.x *= CXOF_PIXEL_SCALING / dt;
          //  msg.vector.y *= CXOF_PIXEL_SCALING / dt;

            of_publisher.publish(msg);
        }

        ros::spinOnce();
    }
    serial_port.close();
}


int main(int argc, char **agv) {
  ros::init(argc, agv, "optic_flow_node");
  optic_flow::Cheerson nn;
  nn.spin();
  return 0;
}
