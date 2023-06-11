#include <stm32f10x.h>
#include "I2C1.h"

void I2C1_Init(void) {
		// enable clock and gpio clock
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	
		// configure B6 and B7 to alternative open drain function, maximum output rate is 50MHz
		GPIOB->CRL |= (GPIO_CRL_CNF6 | GPIO_CRL_MODE6);
		GPIOB->CRL |= (GPIO_CRL_CNF7 | GPIO_CRL_MODE7);
	
		// reset I2C
		I2C1->CR1 |= I2C_CR1_SWRST;
		I2C1->CR1 &= ~I2C_CR1_SWRST;
		
		// set I2C frequency = 100k
		I2C1->CR2 |= I2C_CR2_FREQ_3; 	// I2C T_PCLK1 = 1/8e6
		I2C1->CCR |= (40u<<0); 				// CRR[11:0] = 40 --> T_high = T_low = CCR * T_PCLK1 = 1/2e6
		I2C1->TRISE |= (9u<<0);				// Maximum rise time
		
		// enable I2C
		I2C1->CR1 |= I2C_CR1_PE;	
}

void I2C1_Start(void) {
		I2C1->CR1 |= I2C_CR1_ACK;				// enable ack
		I2C1->CR1 |= I2C_CR1_START;			// send start conditon
		while(!(I2C1->SR1 & I2C_SR1_SB));
}

void I2C1_Stop(void) {
		I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C1_EV6_1(void) {
		I2C1->CR1 &= ~I2C_CR1_ACK;  								// clear the ACK bit 
		uint8_t temp = I2C1->SR1 | I2C1->SR2;  		// read SR1 and SR2 to clear the ADDR bit.... EV6 condition
		I2C1->CR1 |= I2C_CR1_STOP;  								// Stop I2C		
}

void I2C1_EV7_1(void) {
		I2C1->CR1 &= ~I2C_CR1_ACK;  							
		I2C1->CR1 |= I2C_CR1_STOP; 
}

void I2C1_Address_Send(uint8_t addr) {
		I2C1->DR = addr;  											// send the address
		while (!(I2C1->SR1 & I2C_SR1_ADDR));  	// wait for ADDR bit to set
		uint8_t temp = I2C1->SR1 | I2C1->SR2;  	// read SR1 and SR2 to clear the ADDR bit
}	

void I2C1_Read_Started(uint8_t addr, uint8_t *buff, uint8_t size) {
		I2C1_Address_Send(addr);
		int i = 0;
		if(size == 1) {
				I2C1_EV6_1();
				while (!(I2C1->SR1 & I2C_SR1_RXNE));
				buff[0] = I2C1->DR;
		} else {
				for(i = 0; i < size - 2; i++) {
						while (!(I2C1->SR1 & I2C_SR1_RXNE));
						buff[i] = I2C1->DR;              //this act ~ EV7
						I2C1->CR1 |= I2C_CR1_ACK;
				}
				while (!(I2C1->SR1 & I2C_SR1_RXNE));
				buff[i++] = I2C1->DR; 
				I2C1_EV7_1();  								
				while (!(I2C1->SR1 & I2C_SR1_RXNE));
				buff[i] = I2C1->DR;
		}
}

void I2C1_Write_Started(uint8_t data) {
		while (!(I2C1->SR1 & I2C_SR1_TXE));  // wait for TXE bit to set
		I2C1->DR = data;
		while (!(I2C1->SR1 & I2C_SR1_BTF));  // wait for BTF bit to set
}

void I2C1_Read(uint8_t addr, uint8_t reg, uint8_t *buff, uint8_t size) {
		I2C1_Start();
		I2C1_Address_Send(addr);
		I2C1_Write_Started(reg);
		I2C1_Start();  // repeated start
		I2C1_Read_Started(addr + 0x01, buff, size);
		I2C1_Stop();
}

void I2C1_Write_To_Reg(uint8_t addr, uint8_t reg, uint8_t data) {
		I2C1_Start();
		I2C1_Address_Send(addr);
		I2C1_Write_Started(reg);
		I2C1_Write_Started(data);
		I2C1_Stop();
}

void I2C1_Write(uint8_t addr, uint8_t data) {
		I2C1_Start();
		I2C1_Address_Send(addr);
		I2C1_Write_Started(data);
		I2C1_Stop();
}
