/*
 * joystick.h
 *
 *  Created on: Aug 28, 2023
 *      Author: dprim
 */

#ifndef SRC_JOYSTICK_H_
#define SRC_JOYSTICK_H_

uint32_t avgOfArr(uint16_t arr[]);
float getX(void);
float getY(void);
void joystick_setup(void);
uint8_t joystick_moved(float init_x, float init_y);
float joystick_x_calib(void);
float joystick_y_calib(void);
uint8_t joystick_read(float init_x, float init_y);

#endif /* SRC_JOYSTICK_H_ */
