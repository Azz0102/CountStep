#ifndef __I2C1_H
#define __I2C1_H
#include <stm32f10x.h>

void I2C1_Init(void);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_EV6_1(void);
void I2C1_EV7_1(void);
void I2C1_Address_Send(uint8_t addr);
void I2C1_Read_Started(uint8_t addr, uint8_t *buff, uint8_t size);
void I2C1_Read(uint8_t addr, uint8_t reg, uint8_t *buff, uint8_t size);
void I2C1_Write_Started(uint8_t data);
void I2C1_Write_To_Reg(uint8_t addr, uint8_t reg, uint8_t data);
void I2C1_Write(uint8_t addr, uint8_t data);

#endif
