/*
 * I2C.h
 *
 *  Created on: Aug 28, 2023
 *      Author: dprim
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#include <stdint.h>

void I2C_init(void);
void I2C_write(uint8_t data);
void I2C_writeFrame(uint16_t arr[]);

#endif /* SRC_I2C_H_ */
