/*
 * lptim.c
 *
 *  Created on: Mar 12, 2025
 *      Author: gdyan
 */
#include "lptim.h"

void lptim_init(){

	//Disable LPTIM1
	LPTIM1->CR &= ~LPTIM_CR_ENABLE;

	//Turn on LSI Clock
	RCC->CSR |= RCC_CSR_LSION;
	//Wait for LSI Clock to be ready
	while((RCC->CSR & RCC_CSR_LSIRDY) == 0){

	}
	//Enable LPTIM1 Clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
	//Set LPTIM1 Clock to LSI
	RCC->CCIPR |= RCC_CCIPR_LPTIM1SEL_0;
	//Enable LPTIM1 Clock to work in Sleep and Stop modes
	RCC->APB1SMENR1 |= RCC_APB1SMENR1_LPTIM1SMEN;


	//LPTIM interrupts
	//LPTIM1->IER |= LPTIM_IER_UPIE;
	//LPTIM1->IER |= LPTIM_IER_ARROKIE;
	LPTIM1->IER |= LPTIM_IER_ARRMIE;

	//Set Clock to internal
	LPTIM1->CFGR &= ~LPTIM_CFGR_CKSEL;
	//Use internal LSI clock to count
	LPTIM1->CFGR &= ~LPTIM_CFGR_COUNTMODE;
	//Set prescaler to 128
	LPTIM1->CFGR |= LPTIM_CFGR_PRESC_0 | LPTIM_CFGR_PRESC_1 | LPTIM_CFGR_PRESC_2;


	LPTIM1->ARR = 2500;

	NVIC_EnableIRQ(LPTIM1_IRQn);
	NVIC_SetPriority(LPTIM1_IRQn, 1);

	LPTIM1->CR |= LPTIM_CR_ENABLE;
	LPTIM1->CR |= LPTIM_CR_CNTSTRT;
	//Enable LPTIM1 Clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;


}
void lptim_reset(){
	LPTIM1->CNT = 0;
}
void lptim_set_ms(uint16_t period){
	LPTIM1->CNT = 0;
	LPTIM1->ARR = period;
}
