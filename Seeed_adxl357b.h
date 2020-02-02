/*

    Copyright (c) 2019 Seeed Technology Co., Ltd.
    Website    : www.seeed.cc
    Author     : downey
    Create Time: Jan 2018
    Change Log :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#ifndef __SEEED_ADXL357B_H
#define __SEEED_ADXL357B_H


#include <Wire.h>
#include <Arduino.h>

#define DEFAULT_DEV_I2C_ADDRESS   0x1d



#define DEV_MEMS_REG_ADDR         0x01
#define DEV_ID_REG_ADDR           0x02
#define DEV_VERSION_ID_REG_ADDR   0x03
#define STATUS_REG_ADDR           0x04
#define FIFO_ENTRY_REG_ADDR       0x06

#define ACTION_ENABLE_REG_ADDR    0x24
#define SET_THRESHOLD_REG_ADDR    0x25
#define GET_ACTIVE_COUNT_REG_ADDR 0x26
#define SET_INT_PIN_MAP_REG_ADDR  0x2a

#define X_DATA_REG_ADDR           0x08
#define FIFO_DATA_REG_ADDR        0x11

#define POWER_CTR_REG_ADDR        0x2d

#define TEMPERATURE_REG_ADDR      0x06
#define FILTER_REG_ADDR           0x28

#define SET_RANGE_REG_ADDR        0x2c
#define RESET_REG_ADDR            0x2f

typedef enum {
    X_DIR,
    Y_DIR,
    Z_DIR,
} AxisPos;


typedef enum {
    TEN_G = 1,
    TWENTY_G = 2,
    FOURTY_G = 3,
} Adxl_Range;

class Adxl357b {
  public:
    Adxl357b() {};
    ~Adxl357b() {};
    /** begin(),i2c init & set defaule I2C address.
        @param set i2c_address
     **/
    int32_t begin(uint8_t dev_addr = DEFAULT_DEV_I2C_ADDRESS);

    /** set action enable.
        @param enable_x enable x axis action.When the x axis result above threshold,trigger event.
        @param enable_y enable y axis action.When the y axis result above threshold,trigger event.
        @param enable_z enable z axis action.When the z axis result above threshold,trigger event.
     **/
    int32_t setActEnable(bool enable_x, bool enable_y, bool enable_z);


    int32_t readDeviceID(uint8_t& ID);
    int32_t readDeviceVersion(uint8_t& ver);
    /** Read x/y/z axis data from register.

     **/
    int32_t readXYZAxisResultData(int32_t& x, int32_t& y, int32_t& z);
    /** Config ADXL357 mode.
        bit2 - DRDY_OFF ,
        bit1 - TEMP-OFF ,
        bit0 - running mode,0 for measurement mode,1 for standby mode.
     **/
    int32_t setPowerCtr(uint8_t val);

    int32_t readTemperature(float& T);

    /** Get status register data.
        bit4 - NVM-BUSY
        bit3 - Activity
        bit2 - FIFO_OVR
        bit1 - FIFO_FULL
        bit0 - DATA_RDY
     **/
    int32_t getAdxl357Status(uint8_t& byte);
    /** Check whether the  status register's first bit is 1.
     **/
    bool checkDataReady(void);

    int32_t adxlReset(void);

    int32_t setFilter(void);
    /** Set ADXL357's full scale range.
        ±10g,±20g,±40g
     **/
    int32_t setAdxlRange(Adxl_Range range);
    /** Read x/y/z axis data from FIFO.

     **/
    int32_t readXYZAxisResultDataFromFIFO(int32_t& x, int32_t& y, int32_t& z);

    int32_t getFifoEntry(uint8_t& entry);

    /** Set threshold,when measured result over than threshold,trigger event,if corresponding INT function has enabled,trigger interruct event.
        @param acc_g The threshold value,unit is g.
        @param factory Acceleration value corresponding to one acceleration result value,the result value is read from register.

     **/
    int32_t setActThreshold(float acc_g, float factory);
    /** Set config INT pin.

     **/
    int32_t setIntPinMap(uint8_t val);
    /** Event count.event count incresed when axis result value over than threshold.

     **/
    int32_t getActiveCnt(void);
  private:
    uint32_t _x_data;
    uint32_t _y_data;
    uint32_t _z_data;
    uint8_t  _dev_addr;


    int32_t i2cWriteByte(uint8_t reg, uint8_t data);
    int32_t i2cWriteU16(uint8_t reg, uint16_t data);
    int32_t i2cWriteU32(uint8_t reg, uint32_t data);
    int32_t i2cWriteBytes(uint8_t reg, uint8_t* data, uint32_t write_len);

    int32_t i2cReadBytes(uint8_t reg, uint8_t* data, uint32_t read_len);
    int32_t i2cReadU16(uint8_t reg, uint16_t& value);
    int32_t i2cReadByte(uint8_t reg, uint8_t& byte);
    int32_t i2cReadU32(uint8_t reg, uint32_t& value);

};



#endif