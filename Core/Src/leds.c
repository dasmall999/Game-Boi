/*
 * leds.c
 *
 *  Created on: Aug 28, 2023
 *      Author: dprim
 */
#include "main.h"
#include <stdint.h>

#define WAIT_TIME 100000
#define NUM_FLASH 10

void led_init(void){
	  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	  // Configure PA7 for GPIO output to control LED RED

	  GPIOA->MODER   &= ~(GPIO_MODER_MODE7);        //clear MODE7
	  GPIOA->MODER   |= (1<< GPIO_MODER_MODE7_Pos); //set MODES = 01
	  GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT7);         //push pull output
	  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED7);    // low speed
	  GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD7);       //no pull up or pull down registers
	  // PA6 for BLUE
	  GPIOA->MODER   &= ~(GPIO_MODER_MODE6);        //clear MODE6
	  GPIOA->MODER   |= (1<< GPIO_MODER_MODE6_Pos); //set MODES = 01
	  GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT6);         //push pull output
	  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6);    // low speed
	  GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD6);       //no pull up or pull down registers
	  //PA11 for green button led
	  GPIOA->MODER   &= ~(GPIO_MODER_MODE11);        //clear MODE11
	  GPIOA->MODER   |= (1<< GPIO_MODER_MODE11_Pos); //set MODES = 01
	  GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT11);         //push pull output
	  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED11);    // low speed
	  GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD11);       //no pull up or pull down registers
	  //PA12 for red button led
	  GPIOA->MODER   &= ~(GPIO_MODER_MODE12);        //clear MODE12
	  GPIOA->MODER   |= (1<< GPIO_MODER_MODE12_Pos); //set MODES = 01
	  GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT12);         //push pull output
	  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED12);    // low speed
	  GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD12);       //no pull up or pull down registers

	  GPIOA->ODR &= ~(GPIO_ODR_OD6); // turn off blue
	  GPIOA->ODR |= (GPIO_ODR_OD7); //turn on red to start, red = p1

	  GPIOA->ODR |= (GPIO_ODR_OD11); // turn on both buttons
	  GPIOA->ODR |= (GPIO_ODR_OD12);
}

void displayPlayerLED(uint8_t player){
	if (player == 1){
		//blue off, red on
	    GPIOA->ODR &= ~(GPIO_ODR_OD6); // turn off
	    GPIOA->ODR |= (GPIO_ODR_OD7); //turn on
	}
	else{
	    GPIOA->ODR |= (GPIO_ODR_OD6); // turn on
	    GPIOA->ODR &= ~(GPIO_ODR_OD7); //turn off
	}
}

void displayWinner(uint8_t player){
	if (player == 1){
		//flash red
		for(int i=0; i< NUM_FLASH; i++){ //flash 10 times
			GPIOA->ODR &= ~(GPIO_ODR_OD7); // turn off
		    for(int i=0; i<WAIT_TIME; i++);
			GPIOA->ODR |= (GPIO_ODR_OD7); //turn on
			for(int i=0; i<WAIT_TIME; i++);
		}

	}
	else{
		//flash blue
		for(int i=0; i< NUM_FLASH; i++){
			GPIOA->ODR &= ~(GPIO_ODR_OD6); // turn off
		    for(int i=0; i<WAIT_TIME; i++);
			GPIOA->ODR |= (GPIO_ODR_OD6); //turn on
			for(int i=0; i<WAIT_TIME; i++);
		}
	}
}



