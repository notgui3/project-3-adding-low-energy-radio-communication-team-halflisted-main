/*
 * lptim.h
 *
 *  Created on: Mar 12, 2025
 *      Author: gdyan
 */

#ifndef INC_LPTIM_H_
#define INC_LPTIM_H_

/* Include the type definitions for the timer peripheral */
#include <stm32l475xx.h>

void lptim_init();
void lptim_reset();
void lptim_set_ms(uint16_t period);

#endif /* INC_LPTIM_H_ */
