/*
 * maxim61861.c
 *
 *  Created on: 28 dic. 2022
 *      Author: rego_
 */


/*==================[inclusions]=============================================*/
#include "MAXM86161_I2C.h"
#include "maxm86161_hrm_config.h"
/*==================[macros and definitions]=================================*/

#define DELAY_CHARACTER 500000
#define EXAMPLE_I2C_MASTER_BASE    (I2C0_BASE)
#define EXAMPLE_I2C_MASTER ((I2C_Type *)EXAMPLE_I2C_MASTER_BASE)
#define I2C_MASTER_SLAVE_ADDR_7BIT   0b1100010//0x62//




//#define I2C_MASTER_SLAVE_ADDR_7BIT 0b1100011
/*==================[external functions definition]==========================*/
//void maxm86161_i2c_write_byte_data(uint8_t dir,uint8_t data);
//void maxm86161_i2c_read_byte_data(uint8_t dir,uint8_t data);
//void maxm86161_i2c_write_i2c_block_data(uint8_t dir,uint8_t data,uint8_t length);
//void maxm86161_i2c_read_i2c_block_data(uint8_t dir,uint8_t data,uint8_t length);



void maxm86161_i2c_write_to_register(uint8_t dir,uint8_t data){
	I2C_writeByte(I2C_MASTER_SLAVE_ADDR_7BIT, dir, data);
}

void maxm86161_i2c_read_from_register(uint8_t dir,uint8_t * data){
    I2C_readByte(I2C_MASTER_SLAVE_ADDR_7BIT, dir, data, 0);
}


void maxm86161_i2c_block_write(uint8_t dir,uint16_t length,uint8_t *data){
	I2C_writeBytes(I2C_MASTER_SLAVE_ADDR_7BIT, dir, length, data);
}

void maxm86161_i2c_block_read(uint8_t dir,uint16_t length,uint8_t *data){
	I2C_readBytes(I2C_MASTER_SLAVE_ADDR_7BIT, dir, length, data, 0);
}





/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
