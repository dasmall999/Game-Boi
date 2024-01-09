/*
 * buttons.c
 *
 *  Created on: Aug 28, 2023
 *      Author: dprim
 */
#include "main.h"
#include <stdint.h>


void button_init(void){
	  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	  // PA8 Green read, PA9 Red read, PA10 reset read

	  GPIOA->MODER   &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10); //clear MODE8
	  GPIOA->MODER   |= (0 << GPIO_MODER_MODE8_Pos); //set MODES = 00, input
	  GPIOA->MODER   |= (0 << GPIO_MODER_MODE9_Pos); //set MODES = 00, input
	  GPIOA->MODER   |= (0 << GPIO_MODER_MODE10_Pos); //set MODES = 00, input
	  GPIOA->PUPDR   |= (GPIO_PUPDR_PUPD8_1 | GPIO_PUPDR_PUPD9_1 | GPIO_PUPDR_PUPD10_1); //pull down


}

uint8_t read_green(void){
	if ((GPIOA->IDR & GPIO_IDR_ID8_Msk) != 0){ //check if the 8th position is filled
		return 1; //true, high signal being sent
	}
	else {
		return 0; //false, low signal being sent
	}
}

uint8_t read_red(void){
	if ((GPIOA->IDR & GPIO_IDR_ID9_Msk) != 0){
		return 1; //true, high signal being sent
	}
	else {
		return 0; //false, low signal being sent
	}
}

uint8_t read_reset(void){
	if ((GPIOA->IDR & GPIO_IDR_ID10_Msk) != 0){
		return 1; //true, high signal being sent
	}
	else {
		return 0; //false, low signal being sent
	}
}

