/*
 * leds.c
 *
 *  Created on: Oct 3, 2023
 *      Author: schulman
 */


/* Include memory map of our MCU */
#include <stm32l475xx.h>

void leds_init()
{
	/* Enable Peripheral clocks for GPIO ports A and B*/
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;


  /* Configure PA5 as an output by clearing all bits and setting the mode */
  GPIOA->MODER &= ~GPIO_MODER_MODE5;
  GPIOA->MODER |= GPIO_MODER_MODE5_0;

  /* Configure the GPIO output as push pull (transistor for high and low) */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;

  /* Disable the internal pull-up and pull-down resistors */
  GPIOA->PUPDR &= GPIO_PUPDR_PUPD5;

  /* Configure the GPIO to use low speed mode */
  GPIOA->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED5_Pos);

  /* Turn off the LED */
  GPIOA->ODR &= ~GPIO_ODR_OD5;



  /* Configure PB14 as an output by clearing all bits and setting the mode */
    GPIOB->MODER &= ~GPIO_MODER_MODE14;
    GPIOB->MODER |= GPIO_MODER_MODE14_0;

    /* Configure the GPIO output as push pull (transistor for high and low) */
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT14;

    /* Disable the internal pull-up and pull-down resistors */
    GPIOB->PUPDR &= GPIO_PUPDR_PUPD14;

    /* Configure the GPIO to use low speed mode */
    GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED14_Pos);

    /* Turn off the LED */
    GPIOB->ODR &= ~GPIO_ODR_OD14;




}

void leds_set(uint8_t led)
{
  // TODO implement this, enable gpio clock
	// read led parameter and set GPIO ports depending on led parameter
	// if first bit is 1, turn on first LED on GPIO PA5,
	//if second bit is 1, turn on second LED on GPIO PB14

	//if else for first LED to turn on or off
	uint8_t ld1 = led & 0b1;
	uint8_t ld2 = led & 0b10;
	// 1... [ld1] ...1


	//GPIOA->ODR = (GPIOA->ODR & 0xFFFFFFEF) | (ld1 << 4) ;
	//GPIOB->ODR = (GPIOB->ODR & 0xFFFFDFFF) | (ld2 << 12) ;
	if(ld1){
		GPIOA->ODR |= GPIO_ODR_OD5;
	}else{
		GPIOA->ODR &= ~GPIO_ODR_OD5;
	}

	if(ld2){
			GPIOB->ODR |= GPIO_ODR_OD14;
		}else{
			GPIOB->ODR &= ~GPIO_ODR_OD14;
		}


}
