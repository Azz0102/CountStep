#include "MPU6050.h"
#include "I2C1.h"
#include <math.h>

int16_t MPU6050_accX;
int16_t MPU6050_accY;
int16_t MPU6050_accZ;

int16_t MPU6050_read_accX(void) {
		uint8_t data[2];
		I2C1_Read(MPU6050_ADDR, ACCEL_XOUT_H, data, 2);
		return (int16_t)(data[0] << 8 | data[1]);
		 
}

int16_t MPU6050_read_accY(void) {
		uint8_t data[2];
		I2C1_Read(MPU6050_ADDR, ACCEL_YOUT_H, data, 2);
		return (int16_t)(data[0] << 8 | data[1]);
}	

int16_t MPU6050_read_accZ(void) {
		uint8_t data[2];
		I2C1_Read(MPU6050_ADDR, ACCEL_ZOUT_H, data, 2);
		return (int16_t)(data[0] << 8 | data[1]);
}

void MPU_Read(uint8_t addr, uint8_t reg, uint8_t *buff, uint8_t size) {
		I2C1_Read(addr, reg, buff, size);
}

void MPU_Write(uint8_t addr, uint8_t reg, uint8_t data){
		I2C1_Write_To_Reg(addr, reg, data);
}
 
void MPU6050_init(void) {

	uint8_t check;
	uint8_t Data;
	MPU_Read(MPU6050_ADDR, WHO_AM_I, &check, 1);
	if (check == 0x68)  // 0x68 will be returned by the sensor if everything goes well
	{
		Data = 0;
		MPU_Write(MPU6050_ADDR, PWR_MGMT_1, Data);

		// Set DATA_RATE by writing SMPLRT_DIV register
		Data = 79;
		MPU_Write(MPU6050_ADDR, SMPLRT_DIV, Data);

		Data = 0x00;
		MPU_Write(MPU6050_ADDR, ACCEL_CONFIG, Data);
		MPU_Write(MPU6050_ADDR, GYRO_CONFIG, Data);
	}
}
