/*
 * I2C.c
 *
 *  Created on: Aug 28, 2023
 *      Author: dprim
 */
#include "stm32l4xx.h"

#define DISPLAY_LENGTH 8 //same as display width
#define FIVEMSDELAY 5000
#define PTR_CMD 0x00
#define I2C_CONTROL 0xE0 //7 bit address

void I2C_init(void)
{
  // Enable I2C peripheral clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
  // Enable GPIOB clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  // Configure GPIO pins PB8 and PB9
  GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
  GPIOB->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);
  GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9;
  GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9);
  GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL8_Pos) | (4 << GPIO_AFRH_AFSEL9_Pos);

  // Configure I2C peripheral
  I2C1->CR1 &= ~I2C_CR1_PE;
  I2C1->CR1 |= I2C_CR1_TXIE | I2C_CR1_RXIE;
  // Configure timing factor
  I2C1->TIMINGR = 4;
  // Enable I2C peripheral
  I2C1->CR1 |= I2C_CR1_PE;
}

void I2C_write(uint8_t data){ //sends address and 1 data byte

	I2C1->CR2 |= (1<<I2C_CR2_NBYTES_Pos); //amount of bytes being written here

	I2C1->CR2 |= (I2C_CONTROL);	//Control byte, matrix address, 7 bit, 1110 000 0

	I2C1->CR2 |= (I2C_CR2_AUTOEND);
	I2C1->CR2 &= ~(I2C_CR2_RD_WRN);

	I2C1->CR2 |= (I2C_CR2_START);			//Write mem address bytes

	while(!(I2C1->ISR & I2C_ISR_TXE));		//Write data bytes
	I2C1->TXDR =  (data);
	for(int i = 0; i < FIVEMSDELAY; i++); //5ms delay
}

void I2C_writeFrame(uint16_t arr[DISPLAY_LENGTH]){ //sends address, command, and 16 LED info
	I2C1->CR2 |= (17<<I2C_CR2_NBYTES_Pos); //amount of bytes being written here

	I2C1->CR2 |= (I2C_CONTROL);	//Control byte, matrix address, 7 bit, 1110 000 0

	I2C1->CR2 |= (I2C_CR2_AUTOEND);
	I2C1->CR2 &= ~(I2C_CR2_RD_WRN);

	I2C1->CR2 |= (I2C_CR2_START);			//Write mem address bytes

	while(!(I2C1->ISR & I2C_ISR_TXE));		//Write command
	I2C1->TXDR =  PTR_CMD;	//start pointer at 0x00
	for(int i = 0; i < FIVEMSDELAY; i++); //5ms delay

	for (int i = 0; i < DISPLAY_LENGTH; i++){
		while(!(I2C1->ISR & I2C_ISR_TXE));
		I2C1->TXDR =  arr[i] & 0xFF; //write lower 8 bits
		while(!(I2C1->ISR & I2C_ISR_TXE));
		I2C1->TXDR =  arr[i] >> 8; //write upper 8 bits
		for(int i = 0; i < FIVEMSDELAY; i++); //5ms delay
	}

}


