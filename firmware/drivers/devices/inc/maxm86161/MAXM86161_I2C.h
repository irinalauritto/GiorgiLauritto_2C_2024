/*
 * MAXM86161.h
 *
 *  Created on: 28 dic. 2022
 *      Author: rego_
 */

#ifndef MAXM86161_I2C_H_
#define MAXM86161_I2C_H_


/*==================[inclusions]=============================================*/
#include <stdint.h>
#include "i2c_mcu.h"
/*==================[cplusplus]==============================================*/

/*==================[macros]=================================================*/


/*==================[typedef]================================================*/
#define MAXM86161_EN_GPIO_PORT    0   // Puerto 0
#define MAXM86161_EN_GPIO_PIN     0   // Pin 0
/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

void maxm86161_i2c_read_from_register(uint8_t dir,uint8_t * data);
void maxm86161_i2c_write_to_register(uint8_t dir,uint8_t data);
void maxm86161_i2c_block_write(uint8_t dir,uint16_t length,uint8_t *data);
void maxm86161_i2c_block_read(uint8_t dir,uint16_t length,uint8_t *data);

#endif /* #ifndef CIAAPOSIX_STDBOOL_H */



//#endif /*MAXM86161_H_*/
