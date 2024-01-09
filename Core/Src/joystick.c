/*
 * joystick.c
 *
 *  Created on: Aug 28, 2023
 *      Author: dprim
 */
#include "main.h"
#include <stdint.h>
#include "adc.h"

#define NUM_SAMPLES 20
#define DEADZONEFACTOR 0.400 //
#define DELTAX (DEADZONEFACTOR * init_x)
#define DELTAY (DEADZONEFACTOR * init_y)

typedef enum {

	  CENTER,//0
	  LEFT,
	  UPLEFT,
	  UP,
	  UPRIGHT,
	  RIGHT,
	  DOWNRIGHT,
	  DOWN,
	  DOWNLEFT //8


} direction_var_type; //create variable type

uint16_t x_voltage; //filled by ADC1
uint16_t y_voltage; //filled by ADC2


uint32_t avgOfArr(uint16_t arr[]){
	uint32_t sum;
	sum = 0;
	for(int i = 0; i < NUM_SAMPLES; i++){
		sum += arr[i];
	}
	return (sum / NUM_SAMPLES);
}

float getX(void){
	  uint16_t adc_arr1[NUM_SAMPLES] = {0}; //x
	  for (int i = 0; i < NUM_SAMPLES; i++)
	  {
		  adc_arr1[i] = ADC1_block_read(); //get x
	  }
	  uint16_t avgx = avgOfArr(adc_arr1); //avg x
	  return ADC_volt_conv(avgx);
}

float getY(void){
	  uint16_t adc_arr2[NUM_SAMPLES] = {0}; //y
	  for (int i = 0; i < NUM_SAMPLES; i++)
	  {
		  adc_arr2[i] = ADC2_block_read(); //get y
	  }
	  uint16_t avgy = avgOfArr(adc_arr2); //avg y
	  return ADC_volt_conv(avgy);
}

void joystick_setup(void){
	  ADC1_init(); //start the ADCs, joystick is analog
	  ADC2_init();
}

uint8_t joystick_moved(float init_x, float init_y){
	//return T if more than a threshold value from inits, check both vars
	uint8_t ret = 0;
	if ((x_voltage >= (init_x + DELTAX)) || (x_voltage <= (init_x - DELTAX))){
		ret = 1; //return true
	}
	else if ((y_voltage >= (init_y + DELTAY)) || (y_voltage <= (init_y - DELTAY))){
		ret = 1; //return true
	}
	else{
		ret = 0;
	}
	return ret;
}

/*float * joystick_calibration(void){
	//return array of centered x,y voltages at power on
	float r[2]; //[x,y]
	x_voltage = getX(); //store
	y_voltage = getY();
	r[0] = x_voltage;
	r[1] = y_voltage;
	return r;
}*/

float joystick_x_calib(void){
	x_voltage = getX(); //store
	return x_voltage;

}

float joystick_y_calib(void){
	y_voltage = getY(); //store
	return y_voltage;
}

uint8_t joystick_read(float init_x, float init_y){
	//return position in number form
	direction_var_type direction = CENTER;  //create/set direction variable

	x_voltage = getX(); //store new values
	y_voltage = getY();

	if (joystick_moved(init_x, init_y) == 1){ //true
		//already knows its not center, and past deadzone
		if ((x_voltage >= (init_x + DELTAX))) { //xRight
			if ((y_voltage >= (init_y + DELTAY))){ //yUP
				direction = UPRIGHT; //4
			}
			else if ((y_voltage <= (init_y - DELTAY))){ //yDown
				direction = DOWNRIGHT; //6
			}
			else{
				direction = RIGHT; //5
			}
		}
		else if ((x_voltage <= (init_x - DELTAX))){ //xLeft
			if ((y_voltage >= (init_y + DELTAY))){ //yUP
				direction = UPLEFT; //2
			}
			else if ((y_voltage <= (init_y - DELTAY))){ //yDown
				direction = DOWNLEFT; //8
			}
			else{
				direction = LEFT; //1
			}
		}
		else { //xCenter
			if ((y_voltage >= (init_y + DELTAY))){ //yUP
				direction = UP; //3
			}
			else if ((y_voltage <= (init_y - DELTAY))){ //yDown
				direction = DOWN; //7
			}
		}
	}
	else{
		direction = CENTER; //centered and false, 0
	}

	return direction; //0-8 indicates position

}
