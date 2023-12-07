#include <MLX90640_I2C_Driver.h>
#include "myiic.h"
//#include "i2c.h"
int MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
	I2C_Start();                                 //起始信号
	I2C_SendByte(slaveAddr<<1&I2C_WR);         //发送设备地址
	uint8_t ack;
	
	ack=I2C_BufferRead_16bit(slaveAddr<<1,startAddress,nMemAddressRead,data);
	if(ack==0)	return -1;
	return 0;
	
//	uint8_t* bp = (uint8_t*) data;
//	ack = HAL_I2C_Mem_Read(&hi2c1,slaveAddr<<1,startAddress,I2C_MEMADD_SIZE_16BIT,bp,nMemAddressRead*2,HAL_MAX_DELAY);
//	if(ack!= HAL_OK)	return -1;
//	for(int i=0;i<nMemAddressRead*2;i+=2)
//	{
//		uint8_t tmpbytelsb = bp[i+1];
//		bp[i+1]=bp[i];
//		bp[i]=tmpbytelsb;
//	}
//	return 0;
}

int MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data)
{
	int ack = 0;
	uint8_t cmd[2];
	uint16_t dataCheck;
	
	cmd[0] = data >> 8;
	cmd[1] = data & 0x00FF;
	ack=I2C_BufferWrite(slaveAddr<<1,writeAddress,sizeof(cmd),cmd);
	if(ack!= 1)	return -1;
	
//	ack = HAL_I2C_Mem_Write(&hi2c1,slaveAddr<<1,writeAddress,I2C_MEMADD_SIZE_16BIT,cmd,sizeof(cmd),HAL_MAX_DELAY);
//	if(ack!= HAL_OK)	return -1;
	
//	MLX90640_I2CRead(slaveAddr,writeAddress,1,&dataCheck);
//	if(dataCheck != data)	return -2;
	
	return 0;
}