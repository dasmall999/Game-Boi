/*
 * adc.c
 *
 *  Created on: Aug 28, 2023
 *      Author: dprim
 */

#include "main.h"

#define MAX_VOLTAGE 3.3

#define ADC_COEFF 1 // 2.5 clks
//#define ADC_COEFF 1.01 // 47.5 clks
//#define ADC_COEFF 1 // 640.5 clks

uint16_t ADC1_data; //x
uint8_t ADC1_flag;

uint16_t ADC2_data; //y
uint8_t ADC2_flag;

float ADC_volt_conv(uint16_t adc_val)
{
	float adc_adjusted = (ADC_COEFF * MAX_VOLTAGE * adc_val) / (0xFFF);
	return adc_adjusted;
}
uint16_t ADC1_block_read()
{
	// start a conversion
	ADC1->CR |= (ADC_CR_ADSTART);

	// wait for interrupt
	while (ADC1_flag == 0);

	// Reset flag, return data
	ADC1_flag = 0;
	return ADC1_data;
}

uint16_t ADC2_block_read()
{
	// start a conversion
	ADC2->CR |= (ADC_CR_ADSTART);

	// wait for interrupt
	while (ADC2_flag == 0);

	// Reset flag, return data
	ADC2_flag = 0;
	return ADC2_data;
}

void ADC1_init()
{

	// Turn on clock
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

	// Clock Divider Common for ADCs, set clk divider to 1
	ADC123_COMMON->CCR = ADC_CCR_CKMODE_0;

	// Power up the ADC (turn off powerdown)
	ADC1->CR &= ~(ADC_CR_DEEPPWD);

	// Turn on voltage regulator
	ADC1->CR |= (ADC_CR_ADVREGEN);

	// Wait 20 us for Voltage Regulator
	for (uint32_t i = 0; i < 4000; i++);

	// Calibrate the ADC, Disable ADC and Non-Differential
	ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF);

	// Start Calibration
	ADC1->CR |= (ADC_CR_ADCAL);

	// Wait for Calibration to finish
	while (ADC1->CR & ADC_CR_ADCAL);

	// Configure Single Ended Mode on CH10
	ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_10);

	// Turn it on
	ADC1->ISR |= (ADC_ISR_ADRDY); // write 1 to clear rdy
	ADC1->CR |= (ADC_CR_ADEN); // enable ADC
	while (!(ADC1->ISR & ADC_ISR_ADRDY)); // wait for ADC ready flag
	ADC1->ISR |= (ADC_ISR_ADRDY); // write 1 to clear rdy again

	// Configure Sequence to 1 conversion on channel 10
	ADC1->SQR1 = (10 << ADC_SQR1_SQ1_Pos);

	// Configure CH10 Sampling TIme
	ADC1->SMPR1 = 0; // 2.5
	// ADC1->SMPR1 = 0x4 << ADC_SMPR1_SMP5_Pos; // 47.5
	// ADC1->SMPR1 = 0x7 << ADC_SMPR1_SMP5_Pos; // 640.5

	// ADC Configuration - 12-bit, soft trigger, right align, 1 conv, no overrun
	ADC1->CFGR = 0;

	// Enable Interrupt on end of conversion
	ADC1->IER |= ADC_IER_EOCIE;

	// Clear EOC flag
	ADC1->ISR |= ADC_ISR_EOC;
	NVIC->ISER[0] = (1 << (ADC1_2_IRQn));
	__enable_irq(); // Globally enable interrupts

	// Configure GPIO pin for CH10
	GPIOA->MODER |= (GPIO_MODER_MODE5); // set PA5 to analog mode
	GPIOA->ASCR |= (GPIO_ASCR_ASC5); //set to adc input, 1

	// start a conversion
	ADC1->CR |= (ADC_CR_ADSTART);
	return;
}

void ADC2_init()
{

	// Turn on clock
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

	// Clock Divider Common for ADCs, set clk divider to 1
	ADC123_COMMON->CCR = ADC_CCR_CKMODE_0;

	// Power up the ADC (turn off powerdown)
	ADC2->CR &= ~(ADC_CR_DEEPPWD);

	// Turn on voltage regulator
	ADC2->CR |= (ADC_CR_ADVREGEN);

	// Wait 20 us for Voltage Regulator
	for (uint32_t i = 0; i < 4000; i++);

	// Calibrate the ADC, Disable ADC and Non-Differential
	ADC2->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF);

	// Start Calibration
	ADC2->CR |= (ADC_CR_ADCAL);

	// Wait for Calibration to finish
	while (ADC2->CR & ADC_CR_ADCAL);

	// Configure Single Ended Mode on CH5
	ADC2->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);

	// Turn it on
	ADC2->ISR |= (ADC_ISR_ADRDY); // write 1 to clear rdy
	ADC2->CR |= (ADC_CR_ADEN); // enable ADC
	while (!(ADC2->ISR & ADC_ISR_ADRDY)); // wait for ADC ready flag
	ADC2->ISR |= (ADC_ISR_ADRDY); // write 1 to clear rdy again

	// Configure Sequence to 1 conversion on channel 5
	ADC2->SQR1 = (5 << ADC_SQR1_SQ1_Pos);

	// Configure CH5 Sampling TIme
	ADC2->SMPR1 = 0; // 2.5
	// ADC1->SMPR1 = 0x4 << ADC_SMPR1_SMP5_Pos; // 47.5
	// ADC1->SMPR1 = 0x7 << ADC_SMPR1_SMP5_Pos; // 640.5

	// ADC Configuration - 12-bit, soft trigger, right align, 1 conv, no overrun
	ADC2->CFGR = 0;

	// Enable Interrupt on end of conversion
	ADC2->IER |= ADC_IER_EOCIE;

	// Clear EOC flag
	ADC2->ISR |= ADC_ISR_EOC;
	NVIC->ISER[0] = (1 << (ADC1_2_IRQn));
	__enable_irq(); // Globally enable interrupts

	// Configure GPIO pin for CH5
	GPIOA->MODER |= (GPIO_MODER_MODE0); // set PA0 to analog mode
	GPIOA->ASCR |= (GPIO_ASCR_ASC0); //set to adc input

	// start a conversion
	ADC2->CR |= (ADC_CR_ADSTART);
	return;
}

void ADC1_2_IRQHandler (void)
{
	// Clear EOC flag
	ADC1->ISR |= ADC_ISR_EOC;

	// Read Data and Set Flag
	ADC1_data = ADC1 -> DR; //store x voltage
	ADC1_flag = 1;

	ADC2->ISR |= ADC_ISR_EOC;

	ADC2_data = ADC2 -> DR; //store y voltage
	ADC2_flag = 1;
}


