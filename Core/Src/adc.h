/*
 * adc.h
 *
 *  Created on: Aug 28, 2023
 *      Author: dprim
 */

#ifndef SRC_ADC_H_
#define SRC_ADC_H_

float ADC_volt_conv(uint16_t adc_val);
uint16_t ADC1_block_read();
uint16_t ADC2_block_read();
void ADC1_init();
void ADC2_init();
void ADC1_2_IRQHandler (void);

#endif /* SRC_ADC_H_ */
