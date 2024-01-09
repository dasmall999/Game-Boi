/*
 * matrix.h
 *
 *  Created on: Aug 28, 2023
 *      Author: dprim
 */

#ifndef SRC_MATRIX_H_
#define SRC_MATRIX_H_

void matrix_init(void);

void clear(uint16_t arr[8]);
void clear2D(uint8_t arr[8][8]);
void writeGreen(uint8_t x, uint8_t y, uint16_t arr[8]);
void writeRed(uint8_t x, uint8_t y, uint16_t arr[8]);
void writeOrange(uint8_t x, uint8_t y, uint16_t arr[8]);
void clearGreen(uint8_t x, uint8_t y, uint16_t arr[8]);
void clearRed(uint8_t x, uint8_t y, uint16_t arr[8]);
void clearOrange(uint8_t x, uint8_t y, uint16_t arr[8]);

#endif /* SRC_MATRIX_H_ */
