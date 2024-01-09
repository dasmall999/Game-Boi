/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "I2C.h"
#include "matrix.h"
#include "led.h"
#include "buttons.h"
#include "joystick.h"
#include <stdint.h>

#define VERT 1
#define HORIZ 0

#define DISPLAY_LENGTH 8 //same as display width
#define DISPLAY_POS_LIMIT 7
#define DISPLAY_NEG_LIMIT 15

#define BLANK 0
#define GREEN 1
#define RED 2
#define ORANGE 3

#define HALFSECDELAY 500000
#define BUTTON_DELAY 20000
#define HIT_DELAY 200000
#define SHIP_DELAY 2000000

#define WIN_LIMIT 5 //first to 5 hits wins

uint16_t displayBuffer[DISPLAY_LENGTH];

uint8_t player1Ships[DISPLAY_LENGTH][DISPLAY_LENGTH]; // filled with single ints, 1 = green
uint8_t player2Ships[DISPLAY_LENGTH][DISPLAY_LENGTH];
uint8_t player1HitMiss[DISPLAY_LENGTH][DISPLAY_LENGTH]; //what player 1 uses to hit player 2, 0-3 for colors,
uint8_t player2HitMiss[DISPLAY_LENGTH][DISPLAY_LENGTH];

uint8_t currX;
uint8_t currY;
uint8_t prevX;
uint8_t prevY;
uint8_t currPlayer;
uint8_t prevColor;

int8_t num1; // for joystick read
int8_t num2; // for button read

uint8_t player1Points;
uint8_t player2Points;


//need gpio for LEDS

/* Private includes ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);


/**
  * @brief  The application entry point.
  * @retval int
  */

typedef enum {

	  INIT,
	  SHIPSELECT,
	  WAITFORBUTTON,
	  CHECKWIN


} state_var_type; //create variable type

state_var_type state = INIT;  //create state variable

//vert = 1, horiz = 0
void drawShip(uint8_t x_start, uint8_t y_start, uint8_t length, uint8_t dir, uint8_t player){ //NOTE: values entered must be in bounds
	for (int i = 0; i < length; i++){
		if (dir == 1){ //vert
			writeGreen(x_start, y_start + i, displayBuffer); // modify display buffer
			if (player == 1){
				player1Ships[y_start + i][x_start] = 1;; //write to p1's array
			}
			else{
				player2Ships[y_start + i][x_start] = 1;; //write to p2's array
			}
		}
		else{ // horiz
			writeGreen(x_start + i, y_start, displayBuffer); // modify display buffer
			if (player == 1){
				player1Ships[y_start][x_start + i] = 1;; //write to p1's array
			}
			else{
				player2Ships[y_start][x_start + i] = 1;; //write to p2's array
			}
		}
	}
}


void makeP1Ship(void){ //ships drawn to the right and down from init val
	// (x_start, y_start, length, dir, player)
	drawShip(0,0,3,VERT,1);
	drawShip(5,1,3,HORIZ,1);
	drawShip(3,1,4,VERT,1);
	drawShip(2,5,4,HORIZ,1);
	drawShip(1,7,5,HORIZ,1);

}

void makeP2Ship(void){ //ships drawn to the right and down from init val
	// (x_start, y_start, length, dir, player)
	drawShip(1,0,3,VERT,2);
	drawShip(0,5,3,HORIZ,2);
	drawShip(6,2,4,VERT,2);
	drawShip(2,0,4,HORIZ,2);
	drawShip(2,6,5,HORIZ,2);
}

void checkBounds(uint8_t x, uint8_t y){ // checks after coord chnage and adjusts, hard borders
	if (x > DISPLAY_NEG_LIMIT){ // no negatives allowed due to unsigned num, check this way
		currX = 0;
	}
	else if (x > DISPLAY_POS_LIMIT){
		currX = DISPLAY_POS_LIMIT;
	}
	if (y > DISPLAY_NEG_LIMIT){
		currY = 0;
	}
	else if (y > DISPLAY_POS_LIMIT){
		currY = DISPLAY_POS_LIMIT;
	}
}

void restorePixel(uint8_t x, uint8_t y, uint8_t color, uint8_t player){
	if (color == GREEN){
		writeGreen(x, y, displayBuffer);
		if (player == 1){
			player1HitMiss[y][x] = GREEN;
		}
		else{
			player2HitMiss[y][x] = GREEN;
		}

	}
	else if (color == RED){
		writeRed(x, y, displayBuffer);
		if (player == 1){
			player1HitMiss[y][x] = RED;
		}
		else{
			player2HitMiss[y][x] = RED;
		}

	}
	else if (color == BLANK){

		if (player == 1){
			player1HitMiss[y][x] = BLANK;
		}
		else{
			player2HitMiss[y][x] = BLANK;
		}
	}
}

void reinit_display(uint8_t player){
	  if (player == 1){
		  //redisplay p1's display
		  for(int i = 0; i < DISPLAY_LENGTH; i++){
			  for(int j = 0; j < DISPLAY_LENGTH; j++){
				  if (player1HitMiss[i][j] == GREEN){
					  writeGreen(j, i, displayBuffer);
				  }
				  else if (player1HitMiss[i][j] == RED){
					  writeRed(j, i, displayBuffer);

				  }
				  else if (player1HitMiss[i][j] == ORANGE){
					  writeOrange(j, i, displayBuffer);
				  }
			  }
		  }
	  }
	  else{
		  //redisplay p2's display
		  for(int i = 0; i < DISPLAY_LENGTH; i++){
			  for(int j = 0; j < DISPLAY_LENGTH; j++){
				  if (player2HitMiss[i][j] == GREEN){
					  writeGreen(j, i, displayBuffer);
				  }
				  else if (player2HitMiss[i][j] == RED){
					  writeRed(j, i, displayBuffer);

				  }
				  else if (player2HitMiss[i][j] == ORANGE){
					  writeOrange(j, i, displayBuffer);
				  }
			  }
		  }

	  }
}

void movePixel(int8_t x, int8_t y){ //x and y are signed increments to move pixel by
	  prevX = currX; //store value before color change
	  prevY = currY;

	  currX += x; // change current position, (-1, 0) = go left by one unit
	  currY += y;

	  checkBounds(currX, currY); //hard borders, adjust if over or under

	  clearOrange(prevX, prevY, displayBuffer); //blank the old cursor position

	  restorePixel(prevX, prevY, prevColor, currPlayer); //restore old color, prev color already been set
	  if (currPlayer == 1){

		prevColor = player1HitMiss[currY][currX]; //take value before color change
	  }
	  else{

		prevColor = player2HitMiss[currY][currX];
	  }

	  writeOrange(currX, currY, displayBuffer); //write the new cursor position, modify color
	  if (currPlayer == 1){
		  player1HitMiss[currY][currX] = ORANGE;
	  }
	  else{
		  player2HitMiss[currY][currX] = ORANGE;
	  }

	  I2C_writeFrame(displayBuffer); //display the updates
}

int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();

  matrix_init(); //turn on matrix display
  led_init(); //init gpio for player leds
  joystick_setup(); //turn on ADCs
  button_init(); //turn on button gpio

  /*float * a; //[x,y]
  a = joystick_calibration(); //take center position data
  float init_x = a[0];
  float init_y = a[1];*/

  float init_x = joystick_x_calib();
  float init_y = joystick_y_calib();

  while (1)
  {
	  switch(state) {

	  	  case INIT:
	  		  clear(displayBuffer); // clear display buffer and arrays
	  		  clear2D(player1Ships);
	  		  clear2D(player2Ships);
	  		  clear2D(player1HitMiss);
	  		  clear2D(player2HitMiss);

	  		  currX = 0; //clear variables
	  		  currY = 0;
	  		  prevX = 0;
	  		  prevY = 0;
	  		  currPlayer = 1; //player 1 starts
	  		  prevColor = 0; //blank

	  		  player1Points = 0;
	  		  player2Points = 0;

	  		  state = SHIPSELECT;
	  		  break; //skips the other states

	  	  case SHIPSELECT: //both players have same ship config for simplicity
	  		 currPlayer = 1;
	  		 displayPlayerLED(currPlayer);
	  		 makeP1Ship(); //modify data for ship
	  		 I2C_writeFrame(displayBuffer); //display ships
	  		 while((num2 = read_green()) != 1); // wait for enter button confirm
	  		 for(int i = 0; i < (HALFSECDELAY / 5); i++); //software delay for button bounce
	  		 currPlayer = 2;
	  		 displayPlayerLED(currPlayer);

	  		 clear(displayBuffer); //clear display buffer

	  		 I2C_writeFrame(displayBuffer); //display nothing
	  		 for(int i = 0; i < SHIP_DELAY; i++); //delay for handoff


	  		 makeP2Ship(); //modify data for ship
	  		 I2C_writeFrame(displayBuffer); //display ships
	  		 while((num2 = read_green()) != 1); // wait for enter button confirm
	  		 for(int i = 0; i < (HALFSECDELAY / 5); i++); //software delay for button bounce

	  		 currPlayer = 1; //reset to player one
	  		 displayPlayerLED(currPlayer);

	  		 clear(displayBuffer); //clear display buffer

	  		 writeOrange(0,0, displayBuffer); //init orange cursor
	  		 player1HitMiss[0][0] = ORANGE; //3 = orange, update p1's screen

	  		 I2C_writeFrame(displayBuffer); //display blank screen with orange cursor in upper left
	  		 state = WAITFORBUTTON;
	  		 break;

	  	  case WAITFORBUTTON:

	  		  while(((num1 = joystick_read(init_x, init_y)) == 0) && ((num2 = read_green()) == 0)); //wait for joystick/fire
	  		  for(int i = 0; i < BUTTON_DELAY; i++); //delay for button
	  		  if (num2 == 1){ //drop bomb
	  			  for(int i = 0; i < HIT_DELAY; i++); // extra delay for button
	  			  if(currPlayer == 1){
  					  if(player1HitMiss[currY][currX] == ORANGE && (prevColor != BLANK)){
  						  state = WAITFORBUTTON; // prevent remarking
  					  }
	  				  //check to see if ship is at cursor position
  					  else if(player2Ships[currY][currX] != BLANK){
	  					  //hit

	  					  player1Points++; //tick the count

	  					  clearOrange(currX, currY, displayBuffer); //delete cursor, no need to restore since color should be blank
	  					  writeRed(currX, currY, displayBuffer); //mark hit
	  					  player1HitMiss[currY][currX] = RED; //update array

	  					  prevX = currX;
	  					  prevY = currY;
	  					  prevColor = player1HitMiss[0][0]; //store origin info before color change

	  					  currX = 0;
	  					  currY = 0;

	  					  writeOrange(currX,currY, displayBuffer); //init orange at origin
	  					  player1HitMiss[currY][currX] = ORANGE; //update array

	  					  I2C_writeFrame(displayBuffer); //display the updates
	  					  state = CHECKWIN;

	  				  }
	  				  else{
	  					  //miss

	  					  clearOrange(currX, currY, displayBuffer); //delete cursor, no need to restore since color should be blank
	  					  writeGreen(currX, currY, displayBuffer); //mark miss
	  					  player1HitMiss[currY][currX] = GREEN; //update array

	  					  prevX = currX;
	  					  prevY = currY;
	  					  prevColor = player2HitMiss[0][0]; //store origin info before color change

	  					  currX = 0;
	  					  currY = 0;

	  					  player2HitMiss[currY][currX] = ORANGE; //update array, for next player

	  					  I2C_writeFrame(displayBuffer); //display the updates

	  					  for(int i = 0; i < HALFSECDELAY; i++); //small delay
	  					  clear(displayBuffer); //clear display buffer for next player

	  					  currPlayer = 2; //change player
	  					  displayPlayerLED(currPlayer);

	  			  		  reinit_display(currPlayer); //display previously stored info from hit/miss array
	  			  		  I2C_writeFrame(displayBuffer); //display the screen

	  					  state = WAITFORBUTTON;
	  				  }
	  			  }
	  			  else{ //for player 2
  					  if(player2HitMiss[currY][currX] == ORANGE && (prevColor != BLANK)){
  						  state = WAITFORBUTTON; // prevent remarking,
  					  }
	  				  //check to see if ship is at cursor position
  					  else if(player1Ships[currY][currX] != BLANK){
	  					  //hit
	  					  player2Points++; //tick the count

	  					  clearOrange(currX, currY, displayBuffer); //delete cursor, no need to restore since color should be blank
	  					  writeRed(currX, currY, displayBuffer); //mark hit
	  					  player2HitMiss[currY][currX] = RED; //update array

	  					  prevX = currX;
	  					  prevY = currY;
	  					  prevColor = player2HitMiss[0][0]; //store origin info before color change

	  					  currX = 0;
	  					  currY = 0;

	  					  writeOrange(currX,currY, displayBuffer); //init orange at origin
	  					  player2HitMiss[currY][currX] = ORANGE; //update array

	  					  I2C_writeFrame(displayBuffer); //display the updates
	  					  state = CHECKWIN;

	  				  }
	  				  else{
	  					  //miss

	  					  clearOrange(currX, currY, displayBuffer); //delete cursor, no need to restore since color should be blank
	  					  writeGreen(currX, currY, displayBuffer); //mark miss
	  					  player2HitMiss[currY][currX] = GREEN; //update array

	  					  prevX = currX;
	  					  prevY = currY;
	  					  prevColor = player1HitMiss[0][0]; //store origin info before color change

	  					  currX = 0;
	  					  currY = 0;

	  					  player1HitMiss[currY][currX] = ORANGE; //update array

	  					  I2C_writeFrame(displayBuffer); //display the updates

	  					  for(int i = 0; i < HALFSECDELAY; i++); //small delay
	  					  clear(displayBuffer); //clear display buffer for next player

	  					  currPlayer = 1; //change player
	  					  displayPlayerLED(currPlayer);

	  			  		  reinit_display(currPlayer); //display previously stored info from hit/miss array
	  			  		  I2C_writeFrame(displayBuffer); //display the screen

	  					  state = WAITFORBUTTON;
	  				  }

	  			  }

	  		  }
	  		  else if((num1 == 2) || (num1 == 4) || (num1 == 6) || (num1 == 8)){ // move cursor diag
	  			  if (num1 == 2){
	  				  movePixel(-1, 0); //move left
	  				  movePixel(0, -1); //move up
	  			  }
	  			  else if (num1 == 4){
	  				  movePixel(1, 0); //move right
	  				  movePixel(0, -1); //move up
	  			  }
	  			  else if (num1 == 6){
	  				  movePixel(1, 0); //move right
	  				  movePixel(0, 1); //move down
	  			  }
	  			  else if (num1 == 8){
	  				  movePixel(-1, 0); //move left
	  				  movePixel(0, 1); //move down
	  			  }

	  			  state = WAITFORBUTTON;
	  		  }

	  		  else if((num1 == 1) || (num1 == 3) || (num1 == 5) || (num1 == 7)){ // move cursor
	  			  if (num1 == 1){
	  				  movePixel(-1, 0); //move left
	  			  }
	  			  else if (num1 == 5){
	  				  movePixel(1, 0); //move right
	  			  }
	  			  else if (num1 == 3){
	  				  movePixel(0, -1); //move up
	  			  }
	  			  else if (num1 == 7){
	  				  movePixel(0, 1); //move down
	  			  }

	  			  state = WAITFORBUTTON;
	  		  }

	  		  else{ //anything else
	  			  state = WAITFORBUTTON; //wait again for proper press
	  		  }

	  		  break;

	  	  case CHECKWIN:
	  		 if (player1Points >= WIN_LIMIT){
	  			displayWinner(currPlayer);
	  			state = INIT;
	  		 }
	  		 else if (player2Points >= WIN_LIMIT){
	  			displayWinner(currPlayer);
	  			state = INIT;
	  		 }
	  		 else{
	  			 state = WAITFORBUTTON; //keep going until threshold
	  		 }

	  		 break;

	  	  default: // catchall-something went wrong??
	  		  break;
	  }



  }

}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
