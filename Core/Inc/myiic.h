#ifndef MYIIC_H
#define MYIIC_H

#include "main.h"

#define SDA_PORT GPIOB          
#define SCL_PORT GPIOB

#define SDA_PIN GPIO_PIN_7
#define SCL_PIN GPIO_PIN_6

#define I2C_SDA_HIGH()      SDA_PORT->BSRR=SDA_PIN
#define I2C_SDA_LOW()       SDA_PORT->BSRR=(uint32_t)SDA_PIN<<16U
#define I2C_SCL_HIGH()      SCL_PORT->BSRR=SCL_PIN
#define I2C_SCL_LOW()       SCL_PORT->BSRR=(uint32_t)SCL_PIN<<16U
#define I2C_SDA_READ()  		SDA_PORT->IDR & SDA_PIN

/**
 * ACK 回应状态
 **/
#define ACK_OK 0      
#define ACK_ERROR 1
#define I2C_WR				0xFE
#define I2C_RD				0x01

void I2C_Init();
void I2C_Start();                       //IIC起始信号
void I2C_Stop();                        //IIC停止信号
uint8_t I2C_WaitAck();                  //IIC主机等待从机响应ACK
void I2C_Ack();                         //IIC主机向从机发送ACK
void I2C_NAck();                        //IIC主机向从机发送NOACK
void I2C_SendByte(uint8_t data);        //IIC主机发送
uint8_t I2C_ReadByte();                 //IIC从机接收
uint8_t I2C_CheckDevice(uint8_t addr);
uint8_t I2C_BufferRead_16bit(uint8_t slaveAddr,uint16_t startAddress,uint16_t nMemAddressRead,uint16_t *data);
uint8_t I2C_BufferWrite(uint8_t slaveAddr, uint16_t startAddress,uint16_t nMemAddressRead, uint8_t *data);
#endif

