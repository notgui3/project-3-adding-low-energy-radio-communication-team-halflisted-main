#include <stm32l475xx.h>


void lsm6dsl_init(){
	// Variable to check accelerometer is there, passes in register address and replaces it with 0x6A if there
	uint8_t who = 0x0F;
	// Array for passing in register address values, data values to write, and values read to and from the i2c transaction
	uint8_t reg_data[2] = {0};

	//Read Who Am I
	i2c_transaction(0b11010101, 1, &who, 1);
	//0b11010101

	//Check if whoAmI is working and accelerometer is present
	if(who == 0x6A){
		printf("Accelerometer Found \n");
	}else{
		printf("Accelerometer Not Found \n");
	}

	//Write to CTRL1_XL, set the top 4 bits to 0110 --> 104hz normal mode
	//Set first value to register address value of CTRL1_XL, and second value to data value you want to write to the register
	reg_data[0] = 0x10;
	reg_data[1] = 0b01100000;
	i2c_transaction(0b11010100, 0, &reg_data, 1);

//	//Read CTRL1_XL  and print it out to make sure we put int he value
//	reg_data[0] = 0x10;
//	reg_data[1] =  1;
//	i2c_transaction(0b11010101, 1, &reg_data, 1);
//	printf("%d \n", reg_data[0]);


	//Write to INT1_CTRL to turn on accelerometer data interrupts
	//Set first value to register address value of INT1_CTRL, and second value to data value you want to write to the register
	reg_data[0] = 0x0d;
	reg_data[1] = 0x01;
	i2c_transaction(0b11010100, 0, &reg_data, 1);

//
//	reg_data[0] = 0x0d;
//	reg_data[1] = 0x01;
//	i2c_transaction(0b11010101, 1, &reg_data, 1);
//	printf("%d \n", reg_data[0]);

//	//Write to CTRL6_C to disable high performance mode for accelerometer
//	reg_data[0] = 0x15;
//	reg_data[1] =  0b00010000;
//	i2c_transaction(0b11010100, 0, &reg_data, 1);


}

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z){

	// Variable to check status register
	int8_t status_data = 0;

	// Read accelerometer data status register and check if it is ready, if not stall until ready
	while(status_data & 0x1){
		status_data = 0x1E;
		i2c_transaction(0b11010101, 1, &status_data, 1);

	}

	//Send in register address values for the X, Y, Z low bits and high bits
	//and recieve the read values back in the same variable
	uint8_t xL = 0x28;
	i2c_transaction(0b11010101, 1, &xL, 1);
	uint8_t xH = 0x29;
	i2c_transaction(0b11010101, 1, &xH, 1);

	uint8_t yL = 0x2A;
	i2c_transaction(0b11010101, 1, &yL, 1);
	uint8_t yH = 0x2B;
	i2c_transaction(0b11010101, 1, &yH, 1);

	uint8_t zL = 0x2C;
	i2c_transaction(0b11010101, 1, &zL, 1);
	uint8_t zH = 0x2D;
	i2c_transaction(0b11010101, 1, &zH, 1);



	// Cast low and high bit acceleration values to 16 bit
	// Left shift the 8 high bits by 8 and add the 8 lower bits to create full 16 bit acceleration values for X, Y, Z
	// Set the dereferenced x, y, z address to the 16 bit acceleration values
	*x = (((int16_t)(xH)) << 8) | (((int16_t)(xL)));
	*y = (((int16_t)(yH)) << 8) | (((int16_t)(yL)));
	*z = (((int16_t)(zH)) << 8) | (((int16_t)(zL)));
}
