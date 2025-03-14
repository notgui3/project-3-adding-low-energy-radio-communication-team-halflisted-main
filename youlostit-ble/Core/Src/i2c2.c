#include <stm32l475xx.h>



void i2c_init(){



	//Enable GPIOB Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	//Enable I2C2 Clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

	//Disable I2C2
	I2C2->CR1 &= ~I2C_CR1_PE;


	//Set the GPIOB mode of pin 10 to use alternate function
	GPIOB->MODER &= ~GPIO_MODER_MODE10;
	GPIOB->MODER |= GPIO_MODER_MODE10_1;

	//Clear Pin 10 Alternate function and set it to 4
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL10;
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL10_2;

	/* Configure the GPIO output as open drain */
	GPIOB->OTYPER |= GPIO_OTYPER_OT10;

	//Set PB 10 to Open Drain
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD10;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_0;


	//Set the GPIOB Pin 11 mode to use alternate function
	GPIOB->MODER &= ~GPIO_MODER_MODE11;
	GPIOB->MODER |= GPIO_MODER_MODE11_1;

	//Clear Pin 11 Alternate function and set it to 4
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11;
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL11_2;


	/* Configure the GPIO output as push pull (transistor for high and low) */
	GPIOB->OTYPER |= GPIO_OTYPER_OT11;

	//Set PB 11 to Open Drain
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD11;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD11_0;

	/* Configure the GPIO to use low speed mode */
	GPIOB->OSPEEDR &= ~GPIOB->OSPEEDR;


	// Baud Rate is 100khz
	// Base Clock is 4Mhz, meaning a 250ns period
	// We set SCLL and SCLH to 16, and SCLDEL and SDADEL to 4
	// Adding the 4 together, we get 16 + 16 + 4 + 4 = 40
	// 40 * 250ns = 10000ns which is 100Khz
	//Set I2C2 SCLL to x
	I2C2->TIMINGR |= 0x0000000a;
	//Set I2C2 SCLH to x
	I2C2->TIMINGR |= 0x00000a00;
	//Set SCLDEL to 1
	I2C2->TIMINGR |= 0x00300000;
	//Set SDADEL to 1
	I2C2->TIMINGR |= 0x00030000;
	//Set PREC to x
	I2C2->TIMINGR &= ~I2C_TIMINGR_PRESC;

//	//Enable RX interrupts
//	I2C2->CR1 |= I2C_CR1_RXIE;
//	//Enable TX interrupts
//	I2C2->CR1 |= I2C_CR1_TXIE;
//	//Enable NACK interrupts
//	I2C2->CR1 |= I2C_CR1_NACKIE;
//	//Enable STOP interrupts
//	I2C2->CR1 |= I2C_CR1_STOPIE;
//	//Enable TC interrupts
//	I2C2->CR1 |= I2C_CR1_TCIE;



	//Enable I2C2 Peripheral
	I2C2->CR1 |= I2C_CR1_PE;
}


uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len){






	//set address mode to 7 bit
	I2C2->CR2 &= ~I2C_CR2_ADD10;
	//Set the Slave Address of the Peripheral
	I2C2->CR2 |= address;
	//Enable Autoend
	I2C2->CR2 |= I2C_CR2_AUTOEND;
	//Bitmask for checking register values
	uint32_t bit0_mask = 0x00000001;
	//Store the amount of bytes read or written
	uint8_t dataRW = 0;




	//If dir is 1, read, else dir = 0, write
	if(dir){
		//Set the NBYTES
		I2C2->CR2 |= 0x00010000 * len;

		//Set transactions to write
		I2C2->CR2 &= ~I2C_CR2_RD_WRN;
		//Start I2C transaction
		I2C2->CR2 |= I2C_CR2_START;

		//Wait until TXIS set to 1 and is ready to receive data
		while((I2C2->ISR &(bit0_mask << 1)) != I2C_ISR_TXIS){

		}
		//Write the register address value from data to TXDR to set register to read/write tos
		I2C2->TXDR = *data;

		//Set the transaction to read
		I2C2->CR2 |= I2C_CR2_RD_WRN;
		//Start te I2C transaction
		I2C2->CR2 |= I2C_CR2_START;


		//while((I2C2->CR2 & (bit0_mask << 14)) != I2C_CR2_STOP){

		//While the number of bytes we need to read is less than len, keep checking RXNE and reading values
		while(dataRW < len){
			//Check if RXNE == 1 and data is ready to be read
			while((I2C2->ISR & (bit0_mask << 2)) != I2C_ISR_RXNE){
			}
			//Read the data value offset by dataRW and put in in the data
			//Data being read replaces the register address value in the first entry in the array and onwards
			*(data + dataRW) = I2C2->RXDR;
			//Increment dataRW as we have read a value
			dataRW++;


		}


		//}
	}
	else{
		//Set the NBYTES, set to + 1 because we also need to write the register address value
		I2C2->CR2 |= 0x00010000 * (len + 1);

		//Set the I2C transaction to write
		I2C2->CR2 &= ~I2C_CR2_RD_WRN;
		//Start the I2C transactions
		I2C2->CR2 |= I2C_CR2_START;

		//Wait for TXIS == 1 to be ready to write and write in register address
		while((I2C2->ISR &(bit0_mask << 1)) != I2C_ISR_TXIS){

		}
		//Write it register address to TXDR
		I2C2->TXDR = *data;


		//while((I2C2->CR2 & (bit0_mask << 14)) != I2C_CR2_STOP){


		//While the number of bytes we have written is less than len or NACKF isn't set to high, keep writing
		while((dataRW < len) & !((I2C2->ISR & (bit0_mask << 4)) == I2C_ISR_NACKF)){


			//Wait until TXIS == 1 and is TXDR is ready to receive data
			while((I2C2->ISR &(bit0_mask << 1)) != I2C_ISR_TXIS){

			}
			//Write to TXDR the data values offset by dataRW and +1 for the register address in the first array slot
			I2C2->TXDR = *(data + dataRW + 1);
			dataRW++;

		}


		//}

	}


	//Clear CR2 for next transaction
	I2C2->CR2 &= ~I2C2->CR2;

	return 0;

}
