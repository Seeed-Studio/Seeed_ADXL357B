#include "Seeed_adxl357b.h"





/**begin(),i2c init & set defaule I2C address.
 * @param set i2c_address
**/
int32_t Adxl357b::begin(uint8_t dev_addr)
{
	uint8_t ID = 0;
	Wire.begin();
	_dev_addr = dev_addr;
	if(readDeviceID(ID) || (ID != 0xed))
		return -1;
	adxlReset();
	delay(200);
	return 0;
}


int32_t Adxl357b::setAdxlRange(Adxl_Range range)
{
	uint8_t orig = 0;
	i2cReadByte(SET_RANGE_REG_ADDR,orig);
	// Serial.print("read reg = ");
	// Serial.println(orig);
	orig |= range;
	// Serial.println(orig);
	return i2cWriteByte(SET_RANGE_REG_ADDR,orig);
}


int32_t Adxl357b::getActiveCnt(void)
{
	uint8_t cnt = 0;
	if(i2cReadByte(GET_ACTIVE_COUNT_REG_ADDR,cnt))
	{
		return -1;
	}
	return cnt;
}

int32_t Adxl357b::adxlReset(void)
{
	return i2cWriteByte(RESET_REG_ADDR,0x52); 
}


int32_t Adxl357b::setActThreshold(float acc_g,float factory)
{
	int16_t thres = 0;
	thres = (int16_t)(acc_g/factory);
	thres >>= 3;
	// thres = 13300;
	// thres >>= 3;
	Serial.println(thres,HEX);

	return i2cWriteU16(SET_THRESHOLD_REG_ADDR,(uint16_t)thres);
}

/**set action enable.
 * @param enable_x enable x axis action.When the x axis result above threshold,trigger event.
 * @param enable_y enable y axis action.When the y axis result above threshold,trigger event.
 * @param enable_z enable z axis action.When the z axis result above threshold,trigger event.
 **/
int32_t Adxl357b::setActEnable(bool enable_x,bool enable_y,bool enable_z)
{
	uint8_t val = 0;
	val = val | (enable_z << 2) | (enable_y << 1) | enable_x;
	return i2cWriteByte(ACTION_ENABLE_REG_ADDR,val);
}

int32_t Adxl357b::setIntPinMap(uint8_t val)
{
	return i2cWriteByte(SET_INT_PIN_MAP_REG_ADDR,val);
}

/**Config ADXL357 mode.
 * bit2 - DRDY_OFF ,
 * bit1 - TEMP-OFF ,
 * bit0 - running mode,0 for measurement mode,1 for standby mode.
 **/
int32_t Adxl357b::setPowerCtr(uint8_t val)
{
	return i2cWriteByte(POWER_CTR_REG_ADDR,val);
}


int32_t Adxl357b::readTemperature(float &T)
{
	int32_t ret = 0;
	int16_t temperature = 0;
	uint16_t val = 0;
	if(i2cReadU16(TEMPERATURE_REG_ADDR,val))
	{
		return -1;
	}
	
	temperature = val;
	T = 25 + (temperature - 1852)/(-9.05);
	return 0;
}

int32_t Adxl357b::setFilter(void)
{
	/*011 - 15.545*10^-3*ODR, 0011 - 500HZ&125HZ*/
	return i2cWriteByte(FILTER_REG_ADDR,0x33);
}


int32_t Adxl357b::getFifoEntry(uint8_t &entry)
{
	return i2cReadByte(FIFO_ENTRY_REG_ADDR,entry);
}


int32_t Adxl357b::readXYZAxisResultDataFromFIFO(int32_t &x,int32_t &y,int32_t &z)
{
	uint8_t data[9] = {0};
	x = y = z =0;

	if(i2cReadBytes(FIFO_DATA_REG_ADDR,data,9))
	{
		return -1;
	}
	x = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);
	y = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4);
	z = ((uint32_t)data[6] << 12) | ((uint32_t)data[7] << 4) | ((uint32_t)data[8] >> 4);

	if(x & 0x80000){
		x = (x & 0x7ffff) - 0x80000;
	}
	if(y & 0x80000){
		y = (y & 0x7ffff) - 0x80000;
	}
	if(z & 0x80000){
		z = (z & 0x7ffff) - 0x80000;
	}

	// Serial.println("....");
	// Serial.println(x);
	// Serial.println(y);
	// Serial.println(z);
	// Serial.println("....");
	return 0;
}



int32_t Adxl357b::readXYZAxisResultData(int32_t &x,int32_t &y,int32_t &z)
{
	uint8_t data[9] = {0};
	x = y = z =0;

	if(i2cReadBytes(FIFO_DATA_REG_ADDR,data,9))
	{
		return -1;
	}
	x = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);
	y = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4);
	z = ((uint32_t)data[6] << 12) | ((uint32_t)data[7] << 4) | ((uint32_t)data[8] >> 4);
	// Serial.println("....");
	// for(int i=0;i<9;i++)
	// {
	// 	Serial.print(data[i],HEX);
	// 	Serial.print(".");
	// }
	if(x & 0x80000){
		x = (x & 0x7ffff) - 0x80000;
	}
	if(y & 0x80000){
		y = (y & 0x7ffff) - 0x80000;
	}
	if(z & 0x80000){
		z = (z & 0x7ffff) - 0x80000;
	}

	// Serial.println("....");
	// Serial.println(x);
	// Serial.println(y);
	// Serial.println(z);
	// Serial.println("....");
	return 0;
}



int32_t Adxl357b::readDeviceID(uint8_t &ID)
{
	return i2cReadByte(DEV_ID_REG_ADDR,ID);
}

int32_t Adxl357b::readDeviceVersion(uint8_t &ver)
{
	return i2cReadByte(DEV_VERSION_ID_REG_ADDR,ver);
}



int32_t Adxl357b::getAdxl357Status(uint8_t &byte)
{
	return i2cReadByte(STATUS_REG_ADDR,byte);
}

bool Adxl357b::checkDataReady(void)
{
	uint8_t stat = 0;
	getAdxl357Status(stat);
	// Serial.println(",,,");
	// Serial.println(stat,HEX);
	// Serial.println(",,,");
	return stat & 0x01;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/

int32_t Adxl357b::i2cWriteBytes(uint8_t reg,uint8_t *data,uint32_t write_len)
{
	Wire.beginTransmission(_dev_addr);
	Wire.write(reg);
	for(int i=0;i<write_len;i++)
	{
		Wire.write(data[i]);
	}
	return Wire.endTransmission();
}


int32_t Adxl357b::i2cWriteByte(uint8_t reg,uint8_t data)
{
	Wire.beginTransmission(_dev_addr);
    Wire.write(reg);
    Wire.write(data);
    return Wire.endTransmission();
}


int32_t Adxl357b::i2cWriteU16(uint8_t reg,uint16_t data)
{
	uint8_t temp[2] ={0};
	temp [0] = (uint8_t)(data >> 8);
	temp [1] = (uint8_t)data;
	return i2cWriteBytes(reg,temp,2);
}


int32_t Adxl357b::i2cWriteU32(uint8_t reg,uint32_t data)
{
	uint8_t temp[4] ={0};
	temp [0] = (uint8_t)(data >> 24);
	temp [1] = (uint8_t)(data >> 16);
	temp [2] = (uint8_t)(data >> 8);
	temp [3] = (uint8_t)(data);
	return i2cWriteBytes(reg,temp,4);
}



int32_t Adxl357b::i2cReadByte(uint8_t reg,uint8_t &byte)
{
	uint32_t time_out_count=0;
    Wire.beginTransmission(_dev_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
	Wire.requestFrom(_dev_addr,(uint8_t)1);
	while(1!=Wire.available())
	{
        time_out_count++;
        if(time_out_count>10)  return -1;
        delay(1);
    }
	byte = Wire.read();
	return 0;
}


int32_t Adxl357b::i2cReadU16(uint8_t reg,uint16_t &value)
{
	uint32_t time_out_count=0;
	value = 0;
    Wire.beginTransmission(_dev_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
	Wire.requestFrom(_dev_addr,(uint8_t)2);
	while(2!=Wire.available())
	{
        time_out_count++;
        if(time_out_count>10)  return -1;
        delay(1);
    }
	value |= Wire.read();
	value <<= 8;
	value |= Wire.read();
	return 0;
}


int32_t Adxl357b::i2cReadU32(uint8_t reg,uint32_t &value)
{
	uint32_t time_out_count=0;
	value = 0;
    Wire.beginTransmission(_dev_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
	Wire.requestFrom(_dev_addr,(uint8_t)4);
	while(4!=Wire.available())
	{
        time_out_count++;
        if(time_out_count>10)  return -1;
        delay(1);
    }
	value |= Wire.read();
	value <<= 24;
	
	value |= Wire.read();
	value <<= 16;
	
	value |= Wire.read();
	value <<= 8;
	
	value |= Wire.read();
	return 0;
}



int32_t Adxl357b::i2cReadBytes(uint8_t reg,uint8_t *data,uint32_t read_len)
{
    uint32_t time_out_count=0;
    Wire.beginTransmission(_dev_addr);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_dev_addr,read_len);
    while(read_len!=Wire.available())
    {
        time_out_count++;
        if(time_out_count>10)  return -1;
        delay(1);
    }
	for(int i=0;i<read_len;i++)
	{
		data[i] = Wire.read();
	}
    return 0;
	
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
