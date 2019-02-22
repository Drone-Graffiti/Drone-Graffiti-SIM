#ifndef POZYXI2C_H
#define POZYXI2C_H

#include <inttypes.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>


#define BUFFER_LENGTH 512

extern "C" {
  #include "Pozyx_definitions.h"
}


/**
* The device range type stores all the attributes linked to a range measurement
*/
typedef struct __attribute__((packed))_device_range {
    /** The timestamp in ms of the range measurement. */
    uint32_t timestamp;
    /** The distance in mm. */
    uint32_t distance;
    /** The received signal strength in dBm. */
    int16_t RSS;
} device_range_t;


class PozyxI2c
{
public:
    PozyxI2c();
    virtual ~PozyxI2c();

    bool initBus(const char* bus);

    int doRanging(uint16_t destination, device_range_t *range);

    int doRemoteRanging(uint16_t device_from, uint16_t device_to, device_range_t* range);

    int regFunction(uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size);

    int remoteRegFunction(uint16_t destination, uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size);

    int getDeviceRangeInfo(uint16_t device_id, device_range_t *device_range, uint16_t device_id_from = 0);

    bool waitForFlag_safe(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt = nullptr);

    bool waitForFlag(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt);

    int i2cWriteRead(uint8_t* write_data, int write_len, uint8_t* read_data, int read_len);

    int i2cRead(uint8_t reg, unsigned char *buf, int bufsize);

    int remoteI2cRead(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size);

    int readRXBufferData(uint8_t* pData, int size);

private :
    int busfd;               // file desciptor for I2C bus.

    int _mode;               // the mode of operation, can be MODE_INTERRUPT or MODE_POLLING
    int _interrupt;          // variable to indicate that an interrupt has occured

};

#endif // POZYXI2C_H
