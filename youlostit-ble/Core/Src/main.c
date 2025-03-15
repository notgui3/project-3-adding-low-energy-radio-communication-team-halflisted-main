/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include "ble_commands.h"
#include "ble.h"

//#include "ble_commands.h"

//Include Standard Library
#include <stdlib.h>

//includes std io
#include <stdio.h>

//includes string
#include <string.h>

//Include Math Functions
#include <math.h>

/* Include LED driver */
#include "leds.h"

//Include Timer driver
#include "timer.h"

#include "i2c.h"

#include "lsm6dsl.h"

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

// Important for TIM2_IRQHandler. See comment in function definition.
#define bitmask_uint8 0xC0    // 0b1100_0000
#define bitmask_uint16 0xC000 // 0b1100_0000_0000_0000

//Timer for blinking, scaled by TIM2 interrupt --> Currently at 50ms for ARR register --> 1200 * 50ms = 1 minute
#define timeToBlink 1200
//Sensitiviy for the +- margin of error for checking if the board moved
#define sensitivity 1200

//current X, Y, Z Acceleration values, used to compare to see if the MCU has moved
static int16_t currX = 0;
static int16_t currY = 0;
static int16_t currZ = 0;

//X, Y, Z Acceleration values for the previous call, used to compare to see if MCU has moved
static int16_t prevX = 0;
static int16_t prevY = 0;
static int16_t prevZ = 0;

// Preamble hex value for setting the LEDs
static volatile uint8_t preamble = 0x99; // 0b10011001
// Yoav's Student ID for setting the LEDs
static volatile uint16_t student_id = 7031; // Yoav's student ID 0b0001_1011_0111_0111
// Time passed for setting the LEDs, should be equal to the lostCount/timeToBlink --> tells how many minutes have passed
static volatile uint8_t min_lost = 0;

// How many 50ms cycles has passed without moving, resets when changeCount passes a threshhold
static volatile uint32_t lostCount = 0;
// How many times the current and previous acceleration values haven't matched in a row
// Has a +- margin of error according to the sensitivity
static int changeCount = 0;

//Base code, indicates the nonDiscoverablity for catchBLE and sending the messages
static volatile uint8_t nonDiscoverable = 0;



// Redefine the libc _write() function so you can use printf in your code
int _write(int file, char *ptr, int len) {
	int i = 0;
	for (i = 0; i < len; i++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();

  //RESET BLE MODULE
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);

  ble_init();

  // intialize LEDs
  leds_init();

  // Initialize Timer
  timer_init(TIM2);

  // Initialize I2C2 in master mode to connect with the accelerometer
  i2c_init();

  // Initialize the accelerometer
  lsm6dsl_init();

  //Set the Timer2 Interrupt to a period of 50 ms
  timer_set_ms(TIM2, 49);

  //initially set it undiscoverable
  setDiscoverability(0);


  HAL_Delay(10);


  while (1)
  {

	  //Read the current accelerometer X, Y, Z Values
	  		lsm6dsl_read_xyz(&currX, &currY, &currZ);
	  		// 16393 = 1G


	  		// If the difference of the current and previous values X, Y, Z acceleration values are
	  		// outside the margin of error, increment the changeCount
	  		// Otherwise set the change count to 0
	  		if((currX >= prevX - sensitivity)& (currX <= prevX + sensitivity)&
	  				(currY >= prevY - sensitivity)&(currY <= prevY + sensitivity)&
	  				(currZ >= prevZ - sensitivity)& (currZ <= prevZ + sensitivity))
	  		{
	  			changeCount = 0;
	  		}else{
	  			changeCount++;
	  		}

	  		// If the changeCount is large than 8, we considered it moved,
	  		// set the lostCount and changeCount to 0
	  		if(changeCount > 8){
	  			lostCount = 0;
	  			changeCount = 0;
	  		}

	  		// Print the values of lostCount and changeCount and the current accelerometer values
	  		printf("lostCount: %u      changeCount: %u      %d, %d, %d \n", lostCount, changeCount, currX, currY, currZ);

	  		// Set previous X, Y, Z acceleration values to the current values for comparison in the next loop
	  		prevX = currX;
	  		prevY = currY;
	  		prevZ = currZ;


	  		//Default code
	  		if(!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){
	  			catchBLE();
	  		}


	  		// If the lostCount is lower than the timeToBlink (1 min), which indicates its not lost,
	  		//then disconnect the BLE and make it so its not discoverable by setting the discoverability to 0
	  		//Else set the discoverability to 0 and the IRQ handler will set discoverability to 1
	  		if(lostCount < timeToBlink){
	  			leds_set(0);
	  			disconnectBLE();
	  			setDiscoverability(0);
	  			nonDiscoverable = 1;
	  		}else{
	  			nonDiscoverable = 0;
	  		}



	  // Wait for interrupt, only uncomment if low power is needed
	  //__WFI();
  }
}


void TIM2_IRQHandler(){
	//Interrupt fires every 50ms
	// If the lostCount is larger or equal than the timeToBlink (been a minute or over),
	//It indicates that is lost and sets the discoverability to 1 to allow it to be connect through BLE
	//Every 10 seconds send a message with PTGui lost with the amount of seconds has passed in lost mode
	if( (lostCount >= timeToBlink) && ((lostCount%200) == 1) ){
//
		setDiscoverability(1);
		if(!nonDiscoverable){
			  // Send a string to the NORDIC UART service, remember to not include the newline
			  unsigned char test_str[20] = "PTGui lost ";
			  //Subtract and divide down the lostCount to get the amount of seconds after being lost
			  // Store the number of seconds in lost mode in lostCountDiv
			  int lostCountDiv = (lostCount-timeToBlink)/20;
			  //Concatenate the lostCountDiv with the message needed to be sent
			  snprintf(test_str, 20, "PTGui lost %ds ", lostCountDiv);
			  //Send the message through BLE
			  updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(test_str)-1, test_str);
		  }
	}

	//Increment the lostCounter every time Timer 2 interrupt activates
	lostCount++;

	// Reset update flag.
	TIM2->SR &= ~(TIM_SR_UIF);

}

// ----------------------------------------
//OLD LED BLINKING ID IMPLEMENTATION
// ----------------------------------------

/* The `leds_set` function maps the second bit to LED 2 and the first bit to
	 * LED 1. The variable `preamble` encodes the writeup's preamble according
	 * to this format. It was converted by hand.
	 *
	 * The bitmasks `bitmask_uint16` and `bitmask_uint8` are used to isolate the
	 * top two bits in `preamble` and `student_id` respectively. These bits are
	 * used as input for `leds_set`, then shifted out of the sequence.
	 *
	 * We use static variables because they persist between iterations and are
	 * quite handy here.
	 */

// Process preamble two bits at a time, until there are no more bits.
//		if (preamble){
//			// map top two bits to LEDs via function
//			uint16_t top_two_bits = (preamble & bitmask_uint8) >> (8-2);
//			leds_set(top_two_bits);
//			preamble <<= 2;         // shift out top two bits
//		}
//
//		// Process student_id once there are no more preamble bits. Same process as
//		// preamble, but for 16 bits.
//		else if (student_id){
//			uint16_t top_two_bits = (student_id & bitmask_uint16) >> (16-2);
//			leds_set(top_two_bits);
//			student_id <<= 2;
//		}
//		// Process the min_lost two bits at a time and set the LEDs to the value to indicate amount of minutes lost
//		else if(min_lost){
//			// map top two bits to LEDs via function
//			uint16_t top_two_bits = (min_lost & bitmask_uint8) >> (8-2);
//			leds_set(top_two_bits);
//			min_lost <<= 2;	// shift out top two bits we used
//		}
//		// Set preamble and student_id variables to default values and update min_lost to check amount of minustes lost when loop is done.
//		else {
//			preamble = 0x99;
//			student_id = 7031; // yoav student id
//			min_lost = lostCount/timeToBlink;
//		}




/**
  * @brief System Clock Configuration
  * @attention This changes the System clock frequency, make sure you reflect that change in your timer
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
  // This lines changes system clock frequency
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLE_INT_Pin */
  GPIO_InitStruct.Pin = BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_Pin|BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
