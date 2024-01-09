/*
 * led.h
 *
 *  Created on: Aug 28, 2023
 *      Author: dprim
 */

#ifndef SRC_LED_H_
#define SRC_LED_H_

void led_init(void);
void displayPlayerLED(uint8_t player);
void displayWinner(uint8_t player);

#endif /* SRC_LED_H_ */
