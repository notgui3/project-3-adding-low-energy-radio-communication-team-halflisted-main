/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"


void timer_init(TIM_TypeDef* timer)
{
  // TODO implement this
	//Turn on TIM2 Clock, TIM2 Clock is bit 0 of APB1ENR1
		RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

	//change bit 0 to to disable timer
		timer->CR1 &= ~TIM_CR1_CEN;

	//Reset timer count
		timer->CNT = 0;

	//Reset State/Status Register
		timer->SR = 0x00000000;

	//Set Auto Reload Register to a value
		timer->ARR = 49;


	//Slow down both AHB and APB1 to get a 4000 division of the 4MHz to 1Khz for ms auto reload
		timer->PSC |= 7999;
//		RCC->CFGR |= RCC_CFGR_HPRE_DIV512;
//		RCC->CFGR |= RCC_CFGR_PPRE1_DIV8;

	// change bit 4 to 0 for up	counting
		timer->CR1 &= ~TIM_CR1_DIR;
	
	// Enable update interrupts
	// (Setting this to 1 disables interrupts. Set it to 0 by force.)
		timer->CR1 &= ~TIM_CR1_UDIS;

	//Enable interrupts
		timer->DIER |= TIM_DIER_UIE;
		timer->EGR  |= TIM_EGR_UG; // This seems to not have an impact...
		NVIC_EnableIRQ(TIM2_IRQn);
		NVIC_SetPriority(TIM2_IRQn, 1);


	//Enable Timer
		timer->CR1 |= TIM_CR1_CEN;
}

void timer_reset(TIM_TypeDef* timer)
{
  // TODO implement this

	//Set timer's counter to 0
		timer->CNT = 0;
}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
  // TODO implement this

	//set timer to 0
		timer->CNT = 0;

	//Set period time ARR with period_ms
		timer->ARR = period_ms;
}
