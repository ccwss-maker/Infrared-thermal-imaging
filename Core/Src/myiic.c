#include "myiic.h"

void I2C_Delay();
void I2C_Init()
{
	I2C_Stop();
}

void I2C_Start(void)
{
	I2C_SDA_HIGH();	
	I2C_SCL_HIGH();
	I2C_Delay();
	I2C_SDA_LOW();
	I2C_Delay();
	I2C_SCL_LOW();		//一般低电平结束表示一个时钟周期
	I2C_Delay();
}

void I2C_Stop(void)
{
	I2C_SDA_LOW();
	I2C_SCL_HIGH();
	I2C_Delay();
	I2C_SDA_HIGH();
}

void I2C_Ack(void)
{
	I2C_SDA_LOW();
	I2C_SCL_HIGH();
	I2C_Delay();
	I2C_SCL_LOW();
	I2C_Delay();
	I2C_SDA_HIGH();	 //前面提到的，应答之后需要释放SDA总线
}

void I2C_NAck(void)
{
	I2C_SDA_HIGH();
	I2C_SCL_HIGH();
	I2C_Delay();
	I2C_SCL_LOW();
	I2C_Delay();
}
uint8_t I2C_WaitAck(void)
{
	uint8_t r;
	I2C_SDA_HIGH();		//等待应答，主机需要释放SDA总线，由从机产生应答	
	I2C_SCL_HIGH();
	I2C_Delay();
	if(I2C_SDA_READ())
	{
		r=1;
	}
	else
	{
		r=0;
	}
	I2C_SCL_LOW();
	I2C_Delay();
	return r;
}

void I2C_Delay()
{
	for(volatile uint8_t i=0;i < 3;i++);
//	TIM5->CNT=0;
//	while(TIM5->CNT<=1);
}

uint8_t I2C_ReadByte(void)
{
	uint8_t i=0;
	uint8_t value=0;
	for(i=0;i<8;i++)
	{
		value<<=1;
		I2C_SCL_HIGH();
		I2C_Delay();
		if(I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_LOW();
		I2C_Delay();
	}
	return value;
}
void I2C_SendByte(uint8_t data)
{
	uint8_t i = 0;
	for(i=0;i<8;i++)
	{
		if(data&0x80)
		{
			I2C_SDA_HIGH();
		}
		else
		{
			I2C_SDA_LOW();
		}
		I2C_Delay();
		I2C_SCL_HIGH();
		I2C_Delay();
		I2C_SCL_LOW();
		if(i==7)
		{
			I2C_SDA_HIGH();		//数据发送完，释放SDA总线
		}
		data<<=1;
		I2C_Delay();
	}
}

uint8_t I2C_CheckDevice(uint8_t addr)
{
	uint8_t Ack ;
	I2C_Start();
	I2C_SendByte(addr<<1);
	Ack	=	I2C_WaitAck();
	I2C_Stop();
	return Ack;
}

uint8_t I2C_BufferRead_16bit(uint8_t slaveAddr,uint16_t startAddress,uint16_t nMemAddressRead,uint16_t *data)
{
	if(I2C_WaitAck())                            //等待应答
	{
		I2C_Stop();
		return 0;
	}
	I2C_SendByte(startAddress >> 8);                  //发送要读取的内存地址
	if(I2C_WaitAck())                                 //等待应答
	{
		I2C_Stop();
		return 0;
	}
	I2C_SendByte(startAddress & 0x00FF);               //发送要读取的内存地址
	if(I2C_WaitAck())                                  //等待应答
	{
		I2C_Stop();
		return 0;
	}
	I2C_Start();                           //起始信号
	I2C_SendByte(slaveAddr|I2C_RD);        //设备地址
	if(I2C_WaitAck())
	{
		I2C_Stop();
		return 0;
	}
	for(uint16_t i=0;i<nMemAddressRead*2;i+=2)            //读取数据
	{
		uint8_t* bp = (uint8_t*) data;
		bp[i+1] = I2C_ReadByte();
		I2C_Ack();
		bp[i] = I2C_ReadByte();
		
		if(i!=(nMemAddressRead*2-2))        //最后一个字节要发送非应答信号
		{
			I2C_Ack();
		}
		else
		{
			I2C_NAck();
		}
	}
	I2C_Stop();
	return 1;
}

uint8_t I2C_BufferWrite(uint8_t slaveAddr, uint16_t startAddress,uint16_t nMemAddressRead, uint8_t *data)
{
	uint16_t i;
	I2C_Start();
	I2C_SendByte(slaveAddr&I2C_WR);
	if(I2C_WaitAck())
	{
		I2C_Stop();
		return 0;
	}
	I2C_SendByte(startAddress >> 8);                  //发送要读取的内存地址
	if(I2C_WaitAck())                                 //等待应答
	{
		I2C_Stop();
		return 0;
	}
	I2C_SendByte(startAddress & 0x00FF);               //发送要读取的内存地址
	if(I2C_WaitAck())                                  //等待应答
	{
		I2C_Stop();
		return 0;
	}
	while(nMemAddressRead--)
	{
	I2C_SendByte(*data);
	if(I2C_WaitAck())
	{
		I2C_Stop();
		return 0;
	}
	data++;
	}
	I2C_Stop();
	
	return 1;
}