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


//Include Standard Library
#include <stdlib.h>

//includes std io
#include <stdio.h>

////includes string
//#include <string.h>

////Include Math Functions
//#include <math.h>

///* Include LED driver */
//#include "leds.h"

//Include Timer driver
#include "timer.h"

#include "i2c.h"

#include "lsm6dsl.h"

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

//// Important for TIM2_IRQHandler. See comment in function definition.
//#define bitmask_uint8 0xC0    // 0b1100_0000
//#define bitmask_uint16 0xC000 // 0b1100_0000_0000_0000

//Timer for blinking, scaled by TIM2 interrupt --> Currently at 50ms for ARR register --> 1200 * 50ms = 1 minute
#define timeToBlink 6
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

//// Preamble hex value for setting the LEDs
//static volatile uint8_t preamble = 0x99; // 0b10011001
//// Yoav's Student ID for setting the LEDs
//static volatile uint16_t student_id = 7031; // Yoav's student ID 0b0001_1011_0111_0111
//// Time passed for setting the LEDs, should be equal to the lostCount/timeToBlink --> tells how many minutes have passed
//static volatile uint8_t min_lost = 0;

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


void GPIO_OFF(){
	//GPIO
		GPIOA->MODER |= ~GPIOA->MODER;
		GPIOA->OTYPER &= ~GPIOA->OTYPER;
		GPIOA->OSPEEDR &= ~GPIOA->OSPEEDR;
		GPIOA->PUPDR &= ~GPIOA->PUPDR;
		GPIOA->ODR &= ~GPIOA->ODR;
//		GPIOA->ASCR &= ~GPIOA->ASCR;

		GPIOB->MODER |= ~GPIOB->MODER;
		GPIOB->OTYPER &= ~GPIOB->OTYPER;
		GPIOB->OSPEEDR &= ~GPIOB->OSPEEDR;
		GPIOB->PUPDR &= ~GPIOB->PUPDR;
		GPIOB->ODR &= ~GPIOB->ODR;
//		GPIOB->ASCR &= ~GPIOB->ASCR;
//
		GPIOC->MODER |= ~GPIOC->MODER;
		GPIOC->OTYPER &= ~GPIOC->OTYPER;
		GPIOC->OSPEEDR &= ~GPIOC->OSPEEDR;
		GPIOC->PUPDR &= ~GPIOC->PUPDR;
		GPIOC->ODR &= ~GPIOC->ODR;
//		GPIOC->ASCR &= ~GPIOC->ASCR;
//
		GPIOD->MODER |= ~GPIOD->MODER;
		GPIOD->OTYPER &= ~GPIOD->OTYPER;
		GPIOD->OSPEEDR &= ~GPIOD->OSPEEDR;
		GPIOD->PUPDR &= ~GPIOD->PUPDR;
		GPIOD->ODR &= ~GPIOD->ODR;
//		GPIOD->ASCR &= ~GPIOD->ASCR;

		GPIOE->MODER |= ~GPIOE->MODER;
		GPIOE->OTYPER &= ~GPIOE->OTYPER;
		GPIOE->OSPEEDR &= ~GPIOE->OSPEEDR;
		GPIOE->PUPDR &= ~GPIOE->PUPDR;
		GPIOE->ODR &= ~GPIOE->ODR;
//		GPIOE->ASCR &= ~GPIOE->ASCR;

		GPIOF->MODER |= ~GPIOF->MODER;
		GPIOF->OTYPER &= ~GPIOF->OTYPER;
		GPIOF->OSPEEDR &= ~GPIOF->OSPEEDR;
		GPIOF->PUPDR &= ~GPIOF->PUPDR;
		GPIOF->ODR &= ~GPIOF->ODR;
		GPIOF->ASCR &= ~GPIOF->ASCR;

		GPIOG->MODER |= ~GPIOG->MODER;
		GPIOG->OTYPER &= ~GPIOG->OTYPER;
		GPIOG->OSPEEDR &= ~GPIOG->OSPEEDR;
		GPIOG->PUPDR &= ~GPIOG->PUPDR;
		GPIOG->ODR &= ~GPIOG->ODR;
		GPIOG->ASCR &= ~GPIOG->ASCR;

		GPIOH->MODER |= ~GPIOH->MODER;
		GPIOH->OTYPER &= ~GPIOH->OTYPER;
		GPIOH->OSPEEDR &= ~GPIOH->OSPEEDR;
		GPIOH->PUPDR &= ~GPIOH->PUPDR;
		GPIOH->ODR &= ~GPIOH->ODR;
		GPIOH->ASCR &= ~GPIOH->ASCR;
}



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	for(int i = 0; i < 82; i++){
		NVIC_DisableIRQ(i);
	}

//
//	RCC->CFGR |= RCC_CFGR_STOPWUCK;
//	RCC->APB1SMENR1 |= RCC_APB1SMENR1_TIM2SMEN;
	RCC->CR &= ~RCC->CR;
	RCC->AHB1ENR &= ~RCC->AHB1ENR;
	RCC->AHB2ENR &= ~RCC->AHB2ENR;
	RCC->AHB3ENR &= ~RCC->AHB3ENR;
	RCC->APB1ENR1 &= ~RCC->APB1ENR1;
	RCC->APB1ENR2 &= ~RCC->APB1ENR2;
	RCC->APB2ENR &= ~RCC->APB2ENR;
	I2C1->CR1 &= ~I2C_CR1_PE;
	I2C3->CR1 &= ~I2C_CR1_PE;

	//Turn voltage to 1.0
	FLASH->ACR |= FLASH_ACR_LATENCY_1WS;
	PWR->CR1 &= ~PWR_CR1_VOS;
	PWR->CR1 |= PWR_CR1_VOS_1;

	GPIO_OFF();


	//Turn off RTC
	RCC->BDCR &= ~RCC_BDCR_RTCEN;
	RCC->BDCR &= ~RCC_BDCR_LSEON;

	//LPUART
	LPUART1->CR1 &= ~LPUART1->CR1;

	//Turn off all clocks in sleep and stop
	RCC->AHB1SMENR &= ~RCC->AHB1SMENR;
	RCC->AHB2SMENR &= ~RCC->AHB2SMENR;
	RCC->APB1SMENR1 &= ~RCC->APB1SMENR1;
	RCC->APB1SMENR2 &= ~RCC->APB1SMENR2;
	RCC->APB2SMENR &= ~RCC->APB2SMENR;

//	//Disable Voltage Reference Buffer
//	VREFBUF->CSR &= ~VREFBUF->CSR;




  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();

  GPIOA->OSPEEDR &= ~GPIOA->OSPEEDR;
  GPIOB->OSPEEDR &= ~GPIOB->OSPEEDR;
  GPIOC->OSPEEDR &= ~GPIOC->OSPEEDR;
  GPIOD->OSPEEDR &= ~GPIOD->OSPEEDR;
  GPIOE->OSPEEDR &= ~GPIOE->OSPEEDR;



  //RESET BLE MODULE
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);

  ble_init();

//  // intialize LEDs
//  leds_init();
//  leds_set(0);

//  // Initialize Timer
//  timer_init(TIM2);
//
//  //Set the Timer2 Interrup,t to a period of 50 ms
//  timer_set_ms(TIM2, 1227);

  lptim_init();
  lptim_set_ms(2500);

  // Initialize I2C2 in master mode to connect with the accelerometer
  i2c_init();

  // Initialize the accelerometer
  lsm6dsl_init();

  //initially set it undiscoverable
  setDiscoverability(0);


  HAL_Delay(10);


  while (1)
  {
//	  	  	//Enable GPIOB Clock
	  		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	  		//Enable I2C2 Clock
	  		RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

	  	  	//Read the current accelerometer X, Y, Z Values
	  		lsm6dsl_read_xyz(&currX, &currY, &currZ);
	  		// 16393 = 1G

//	  		//Enable GPIOB Clock
	  		RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOBEN;

			//Enable I2C2 Clock
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C2EN;


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
	  		if(changeCount > 0){
	  			lostCount = 0;
	  			changeCount = 0;
	  			//lptim_reset();
	  		}

	  		// Print the values of lostCount and changeCount and the current accelerometer values
	  		//printf("lostCount: %u      changeCount: %u      %d, %d, %d \n", lostCount, changeCount, currX, currY, currZ);

	  		// Set previous X, Y, Z acceleration values to the current values for comparison in the next loop
	  		prevX = currX;
	  		prevY = currY;
	  		prevZ = currZ;



	  		//Default code
	  		if(!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){
	  			RCC->CR |= RCC_CR_MSIRANGE_7;
	  			catchBLE();
	  			RCC->CR |= RCC_CR_MSIRANGE_1;
	  		}

	  		//need to make disconnectBLE and setDiscoverability to fire only once
	  		//--> used the discoverability variable

	  		// If the lostCount is lower than the timeToBlink (1 min), which indicates its not lost,
	  		//then disconnect the BLE and make it so its not discoverable by setting the discoverability to 0
	  		//Else set the discoverability to 0 and the IRQ handler will set discoverability to 1
	  		if((lostCount < timeToBlink) & (nonDiscoverable == 0)){
	  			RCC->CR |= RCC_CR_MSIRANGE_7;
	  			disconnectBLE();
	  			setDiscoverability(0);
	  			//standbyBle();
	  			RCC->CR |= RCC_CR_MSIRANGE_1;
	  			nonDiscoverable = 1;
	  		}else{
	  			nonDiscoverable = 0;
	  		}

	  		//Interrupt fires every 50ms
			// If the lostCount is larger or equal than the timeToBlink (been a minute or over),
			//It indicates that is lost and sets the discoverability to 1 to allow it to be connect through BLE
			//Every 10 seconds send a message with PTGui lost with the amount of seconds has passed in lost mode
			if( (lostCount >= timeToBlink) && ((lostCount%(1)) == 0) ){

				RCC->CR |= RCC_CR_MSIRANGE_7;

				setDiscoverability(1);

				RCC->CR |= RCC_CR_MSIRANGE_1;
				if(!nonDiscoverable){
					  // Send a string to the NORDIC UART service, remember to not include the newline
					  unsigned char test_str[20] = "PTGui lost ";
					  //Subtract and divide down the lostCount to get the amount of seconds after being lost
					  // Store the number of seconds in lost mode in lostCountDiv
					  int lostCountDiv = (lostCount-timeToBlink)*10;
					  //Concatenate the lostCountDiv with the message needed to be sent
					  snprintf(test_str, 20, "PTGui lost %ds ", lostCountDiv);
					  //Send the message through BLE
					  RCC->CR |= RCC_CR_MSIRANGE_7;
					  updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(test_str)-1, test_str);
					  RCC->CR |= RCC_CR_MSIRANGE_1;
				 }
			}

	  		//Clear LPMS bits to set them to "000” (Stop mode)
	  		PWR->CR1 &= ~PWR_CR1_LPMS;
	  		PWR->CR1 |= PWR_CR1_LPMS_STOP2;

	  		// Prepare to enter deep sleep mode (Stop mode)
	  		// Set the SLEEPDEEP bit in the System Control Register
	  		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	  		// Execute the Wait-For-Interrupt instruction.
	  		// This puts the CPU into deep sleep mode until an interrupt occurs.
	  		HAL_SuspendTick();
//	  		FLASH->ACR |= FLASH_ACR_SLEEP_PD;
	  		RCC->CR |= RCC_CR_MSIRANGE_1;
//	  		GPIO_OFF();
//	  		PWR->CR1 |= PWR_CR1_LPR;
//	  		RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

	  		__WFI();
	  		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
//	  		MX_GPIO_Init();
//	  		i2c_init();
	  		//HAL_PWREx_EnterSTOP2Mode(PWR_SLEEPENTRY_WFI);

//	  		PWR->CR1 &= ~PWR_CR1_LPR;
//	  		while((PWR->SR2 & PWR_SR2_REGLPF) > 0){
//
//	  		}
//	  		RCC->CR |= RCC_CR_MSIRANGE_;
	  		//RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM2EN;
	  		HAL_ResumeTick();


  }
}

//volatile int led_set = 2;
////Can turn off the clocks that are not TIM2
////Can turn off I2C while not reading --> fast and turn off longer or slow
//void TIM2_IRQHandler(){
//	if(led_set == 1){
//		led_set = 2;
//	}
//	else{
//		led_set = 1;
//	}
//	leds_set(led_set);
//
//
//	//Increment the lostCounter every time Timer 2 interrupt activates
//	lostCount++;
//
//	// Reset update flag.
//	TIM2->SR &= ~(TIM_SR_UIF);
//
//}

void LPTIM1_IRQHandler(){

//	if(led_set == 1){
//			led_set = 2;
//		}
//		else{
//			led_set = 1;
//		}
//		leds_set(led_set);

	lostCount++;
	// Reset update flag.
	LPTIM1->ICR |= LPTIM_ICR_ARRMCF;

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
