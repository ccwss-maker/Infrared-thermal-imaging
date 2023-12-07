#ifndef __MLX90640_SWI2C_Driver_H
#define __MLX90640_SWI2C_Driver_H 

#include <main.h>
//#include <i2c.h>
int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,uint16_t nMemAddressRead, uint16_t *data);
int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data);
#endif