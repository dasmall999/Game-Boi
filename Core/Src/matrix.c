/*
 * matrix.c
 *
 *  Created on: Aug 28, 2023
 *      Author: dprim
 */
#include "I2C.h"

#define DISPLAY_LENGTH 8 //same as display width
#define ORANGE_MASK 257 // 1 0000 0001, displays 2 places at once
#define ONEMSDELAY 1000
#define OSC_CMD 0x20
#define BRIGHT_CMD 0xE0
#define DISP_CMD 0x80

void matrix_init(void){
	for(int i = 0; i< ONEMSDELAY; i++); //1ms delay
	I2C_write(OSC_CMD | 1); //0010 xxx1, turn on oscillator
	I2C_write(BRIGHT_CMD | 4); // 1110 bbbb, set brightness level to 4
	I2C_write(DISP_CMD | 1); // 1000 xbbd, turn on display
	I2C_write(DISP_CMD | 0 << 1 | 1); // 1000 xbbd, no blinking

}

void clear(uint16_t arr[DISPLAY_LENGTH]){
	for(int i = 0; i < DISPLAY_LENGTH; i++){
		arr[i] = 0;
	}
}

void clear2D(uint8_t arr[DISPLAY_LENGTH][DISPLAY_LENGTH]){
	for(int i = 0; i < DISPLAY_LENGTH; i++){
		for(int j= 0; j<DISPLAY_LENGTH; j++){
			arr[i][j] = 0;  //clear at passed in address
		}
	}
}

//(0,0) origin at upper left corner

void writeGreen(uint8_t x, uint8_t y, uint16_t arr[DISPLAY_LENGTH]){
	arr[(DISPLAY_LENGTH-1) - x] |= (1 << y);
}

void writeRed(uint8_t x, uint8_t y, uint16_t arr[DISPLAY_LENGTH]){
	arr[(DISPLAY_LENGTH-1) - x] |= (1 << (y + DISPLAY_LENGTH));
}

void writeOrange(uint8_t x, uint8_t y, uint16_t arr[DISPLAY_LENGTH]){
	arr[(DISPLAY_LENGTH-1) - x] |= (ORANGE_MASK << y);
}

void clearGreen(uint8_t x, uint8_t y, uint16_t arr[DISPLAY_LENGTH]){
	arr[(DISPLAY_LENGTH-1) - x] &= ~(1 << y);
}

void clearRed(uint8_t x, uint8_t y, uint16_t arr[DISPLAY_LENGTH]){
	arr[(DISPLAY_LENGTH-1) - x] &= ~(1 << (y + DISPLAY_LENGTH));
}

void clearOrange(uint8_t x, uint8_t y, uint16_t arr[DISPLAY_LENGTH]){
	arr[(DISPLAY_LENGTH-1) - x] &= ~(ORANGE_MASK << y);
}

