#include "PozyxI2c.h"
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <assert.h>
#include <cstring>
#include <chrono>
#include <unistd.h>


PozyxI2c::PozyxI2c()
{

}

PozyxI2c::~PozyxI2c() {
    if (busfd > 0)
        close(busfd);
}

bool PozyxI2c::initBus(const char* bus) {

    if ((busfd = open(bus, O_RDWR)) < 0) {
        return false;
    } else {
        return true;
    }
}

int PozyxI2c::i2cWriteRead(uint8_t* write_data, int write_len, uint8_t* read_data, int read_len)
{
    uint8_t outbuf[write_len], inbuf[read_len];

    memcpy(outbuf, write_data, write_len);

    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

    msgs[0].addr = POZYX_I2C_ADDRESS;
    msgs[0].flags = 0;
    msgs[0].len = write_len;
    msgs[0].buf = outbuf;

    msgs[1].addr = POZYX_I2C_ADDRESS;
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].len = read_len;
    msgs[1].buf = inbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;

    inbuf[0] = 0;

    if (ioctl(busfd, I2C_RDWR, &msgset) < 0) {
        perror("ioctl(I2C_RDWR) in i2c_write");
        return POZYX_FAILURE;
    }

    memcpy(read_data, inbuf, read_len);

    return POZYX_SUCCESS;  // return : no error
}

int PozyxI2c::i2cRead(uint8_t reg, unsigned char *buf, int bufsize)
{
    uint8_t outbuf[1], inbuf[bufsize];
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

    msgs[0].addr = POZYX_I2C_ADDRESS;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = outbuf;

    msgs[1].addr = POZYX_I2C_ADDRESS;
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].len = bufsize;
    msgs[1].buf = inbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;

    outbuf[0] = reg;

    inbuf[0] = 0;

    *buf = 0;
    if (ioctl(busfd, I2C_RDWR, &msgset) < 0) {
        perror("ioctl(I2C_RDWR) in i2c_read");
        return POZYX_FAILURE;
    }

    memcpy(buf, inbuf, bufsize);

    return bufsize;
}


bool PozyxI2c::waitForFlag(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt)
{
    using namespace std::chrono;

    auto start = std::chrono::system_clock::now();
    int status;

    // stay in this loop until the event interrupt flag is set or until the the timer runs out
    while (duration_cast<milliseconds>(std::chrono::system_clock::now() - start).count() < timeout_ms)
    {
        // in polling mode, we insert a small delay such that we don't swamp the i2c bus
        if( _mode == MODE_POLLING ){
            usleep(1000);
        }

        if( (_interrupt == 1) || (_mode == MODE_POLLING))
        {
            _interrupt = 0;

            // Read out the interrupt status register. After reading from this register, pozyx automatically clears the interrupt flags.
            uint8_t interrupt_status = 0;
            status = i2cRead(POZYX_INT_STATUS, &interrupt_status, 1);
            if((interrupt_status & interrupt_flag) && status == POZYX_SUCCESS)
            {
                // one of the interrupts we were waiting for arrived!
                if(interrupt != NULL)
                    *interrupt = interrupt_status;
                return true;
            }
        }
    }
    // too bad, pozyx didn't respond
    // 1) pozyx can select from two pins to generate interrupts, make sure the correct pin is connected with the attachInterrupt() function.
    // 2) make sure the interrupt we are waiting for is enabled in the POZYX_INT_MASK register)
    return false;
}

bool PozyxI2c::waitForFlag_safe(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt) {
    int tmp = _mode;
    _mode = MODE_POLLING;
    bool result = waitForFlag(interrupt_flag, timeout_ms, interrupt);
    _mode = tmp;
    return result;
}


int PozyxI2c::regFunction(uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size) {
    if(!IS_FUNCTIONCALL(reg_address))
        return POZYX_FAILURE;

    uint8_t status;

    // this feels a bit clumsy with all these memcpy's
    uint8_t write_data[param_size+1];
    write_data[0] = reg_address;
    memcpy(write_data+1, params, param_size);
    uint8_t read_data[size+1];

    // first write some data with i2c and then read some data
    status = i2cWriteRead(write_data, param_size + 1, read_data, size+1);
    if(status == POZYX_FAILURE)
        return status;

    memcpy(pData, read_data+1, size);

    // the first byte that a function returns is always it's success indicator, so we simply pass this through
    return read_data[0];
}


int PozyxI2c::doRanging(uint16_t destination, device_range_t *range) {
    memset((uint8_t*)range, 0, sizeof(device_range_t));

    // trigger the ranging
    uint8_t int_status = 0;
    i2cRead(POZYX_INT_STATUS, &int_status, 1);
    int status = regFunction(POZYX_DO_RANGING, (uint8_t *) &destination, 2, NULL, 0);
    if (status == POZYX_SUCCESS )
    {
        // wait for the result
        if(waitForFlag_safe(POZYX_INT_STATUS_FUNC | POZYX_INT_STATUS_ERR, POZYX_DELAY_INTERRUPT, &int_status))
        {
            if((int_status & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR)
            {
                return POZYX_FAILURE;
            }else{
                // read out the ranging results
                return getDeviceRangeInfo(destination, range);
            }
        } else {
            return POZYX_TIMEOUT;
        }
    } else {
        return POZYX_FAILURE;
    }
}

int PozyxI2c::doRemoteRanging(uint16_t device_from, uint16_t device_to, device_range_t* range) {
    assert(device_from != 0);
    assert(device_to != 0);
    assert(range != NULL);

    int status;
    range->timestamp = 0;
    range->distance = 0;
    range->RSS = 0;

    // trigger remote ranging between the two devices
    status = remoteRegFunction(device_from, POZYX_DO_RANGING, (uint8_t *)&device_to, 2, NULL, 0);
    if (status == POZYX_SUCCESS)
    {
        // the remote device (device_from) will respond with the ranging result, wait for that to happen
        if(waitForFlag_safe(POZYX_INT_STATUS_RX_DATA , POZYX_DELAY_INTERRUPT))
        {
            // read out the ranging results from the received message
            //delay(5);
            return getDeviceRangeInfo(device_to, range, device_from);

        }else{
            return POZYX_TIMEOUT;
        }
    }else{
        return status;
    }
}


int PozyxI2c::remoteRegFunction(uint16_t destination, uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size)
{
    // some checks
    if(!IS_FUNCTIONCALL(reg_address))      return POZYX_FAILURE;        // the register is not a function

    int status = 0;

    // first prepare the packet to send
    uint8_t tmp_data[param_size+2];
    tmp_data[0] = 0;
    tmp_data[1] = reg_address;                // the first byte is the function register address we want to call.
    memcpy(tmp_data+2, params, param_size);   // the remaining bytes are the parameter bytes for the function.
    status = regFunction(POZYX_TX_DATA, tmp_data, param_size+2, NULL, 0);

    // stop if POZYX_TX_DATA returned an error.
    if(status == POZYX_FAILURE)
    {
        return status;
    }

    // send the packet
    uint8_t tx_params[3];
    tx_params[0] = (uint8_t)destination;
    tx_params[1] = (uint8_t)(destination>>8);
    tx_params[2] = 0x08;    // flag to indicate a register function call
    uint8_t int_status = 0;
    i2cRead(POZYX_INT_STATUS, &int_status, 1);      // first clear out the interrupt status register by reading from it
    status = regFunction(POZYX_TX_SEND, tx_params, 3, NULL, 0);

    // stop if POZYX_TX_SEND returned an error.
    if(status == POZYX_FAILURE){
        return status;
    }

    // wait up to x ms to receive a response
    if(waitForFlag_safe(POZYX_INT_STATUS_FUNC | POZYX_INT_STATUS_ERR, 1000, &int_status))
    {
        if((int_status & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR)
        {
            return POZYX_FAILURE;
        }else
        {
            // we received a response, now get some information about the response
            uint8_t rx_info[3];
            i2cRead(POZYX_RX_NETWORK_ID, rx_info, 3);
            uint16_t remote_network_id = rx_info[0] + ((uint16_t)rx_info[1]<<8);
            uint8_t data_len = rx_info[2];

            if( remote_network_id == destination && data_len == size+1)
            {
                uint8_t return_data[size+1];

                status = readRXBufferData(return_data, size+1);

                if(status == POZYX_FAILURE){
                    // debug information
                    return status;
                }

                memcpy(pData, return_data+1, size);

                return return_data[0];
            }else{
                return POZYX_FAILURE;
            }
        }

    }else{
        // timeout
        return POZYX_TIMEOUT;
    }
}

int PozyxI2c::getDeviceRangeInfo(uint16_t device_id, device_range_t *device_range, uint16_t device_id_from) {
    assert(device_id != 0);
    assert(device_range != NULL);

    return regFunction(POZYX_DEVICE_GETRANGEINFO, (uint8_t *) &device_id, 2, (uint8_t *) device_range, sizeof(device_range_t));
}

int PozyxI2c::remoteI2cRead(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size) {
    // some checks
    if(!IS_REG_READABLE(reg_address))      return POZYX_FAILURE;        // the register is not readable
    if(size > MAX_BUF_SIZE)                return POZYX_FAILURE;        // trying to read too much data
    if(destination == 0)                   return POZYX_FAILURE;        // remote read not allowed in broadcast mode

    int status = 0;

    // first prepare the packet to send
    uint8_t tmp_data[3];
    tmp_data[0] = 0;                  // the offset in the TX buffer
    tmp_data[1] = reg_address;        // the first byte is the register address we want to start reading from
    tmp_data[2] = size;               // the number of bytes to read starting from the register address
    status = regFunction(POZYX_TX_DATA, (uint8_t *)&tmp_data, 3, NULL, 0);

    // stop if POZYX_TX_DATA returned an error.
    if(status == POZYX_FAILURE)
        return status;

    // send the packet
    uint8_t params[3];
    params[0] = (uint8_t)destination;
    params[1] = (uint8_t)(destination>>8);
    params[2] = 0x02;    // flag to indicate a register read

    uint8_t int_status = 0;
    i2cRead(POZYX_INT_STATUS, &int_status, 1);      // first clear out the interrupt status register by reading from it
    status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);

    // stop if POZYX_TX_SEND returned an error.
    if(status == POZYX_FAILURE)
        return status;

    // wait up to x ms to receive a response
    if(waitForFlag_safe(POZYX_INT_STATUS_FUNC | POZYX_INT_STATUS_ERR, 1000, &int_status))
    {
        if((int_status & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR)
        {
            // An error occured during positioning.
            // Please read out the register POZYX_ERRORCODE to obtain more information about the error
            return POZYX_FAILURE;
        }else{
            // we received a response, now get some information about the response
            uint8_t rx_info[3]= {0,0,0};
            i2cRead(POZYX_RX_NETWORK_ID, rx_info, 3);
            uint16_t remote_network_id = rx_info[0] + ((uint16_t)rx_info[1]<<8);
            uint8_t data_len = rx_info[2];

            if( remote_network_id == destination && data_len == size)
            {
                status = readRXBufferData(pData, size);
                return status;
            } else {
                return POZYX_FAILURE;
            }
        }

    } else {
        // timeout
        return POZYX_TIMEOUT;
    }
}


int PozyxI2c::readRXBufferData(uint8_t* pData, int size) {
    if (size > MAX_BUF_SIZE){
        return POZYX_FAILURE;
    }

    int status;
    int i;
    uint8_t params[2];
    int max_bytes = BUFFER_LENGTH-1;
    int n_runs = ceil((float)size / max_bytes);

    // read out the received data.
    for(i=0; i<n_runs; i++)
    {
        params[0] = i*max_bytes;      // the offset
        if(i+1 != n_runs){
            params[1] = max_bytes;      // the number of bytes to read
        }else{
            params[1] = size - i*max_bytes;      // the number of bytes to read
        }
        status = regFunction(POZYX_RX_DATA, params, 2, pData+params[0], params[1]);
    }

    return status;
}
