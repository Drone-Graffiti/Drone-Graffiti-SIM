#pragma once

#include <serial/serial.h>
#include <ros/ros.h>

#define SERIAL_SPEED 19200
#define SERIAL_TIMEOUT_MS 1000

#define CXOF_HEADER         (uint8_t)0xFE
#define CXOF_FOOTER         (uint8_t)0xAA
#define CXOF_FRAME_LENGTH               9
#define CXOF_PIXEL_SCALING      (1.76e-3)

namespace optic_flow {

class Cheerson {
public:
    Cheerson();
    virtual ~Cheerson();

    void spin();
private:
    ros::Publisher of_publisher;
    serial::Serial serial_port;
    uint8_t buf[10];                    // buff of characters received from flow sensor
    uint8_t buf_len;                    // number of characters in buffer
    float flowScaler_x;
    float flowScaler_y;
};

}
