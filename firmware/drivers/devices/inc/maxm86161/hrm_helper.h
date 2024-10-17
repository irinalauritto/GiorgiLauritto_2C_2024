/*

 *
 *  Created on: 28 dic. 2022
 *      Author: rego_
 */

#ifndef MAXM86161_HELPER_H_
#define MAXM86161_HELPER_H_

#include "MAXM86161.h"
#include "maxm86161_hrm_spo2.h"
#include "maxm86161_hrm_config.h"


typedef void *  max86161_hrm_helper_handle_t;


/**************************************************************************//**
 * @brief Empty the samples in queue.
 *****************************************************************************/
void maxm86161_hrm_helper_sample_queue_clear(void);

/**************************************************************************//**
 * @brief Query number of entries in the queue.
 *****************************************************************************/
int32_t maxm86161_hrm_helper_sample_queue_numentries(void);

/**************************************************************************//**
 * @brief Get sample from the queue.
 *****************************************************************************/
int32_t maxm86161_hrm_helper_sample_queue_get(maxm86161_hrm_irq_sample_t *samples);
/**************************************************************************//**
 * @brief Initialize and clear the queue.
 *****************************************************************************/
int32_t maxm86161_hrm_helper_initialize(void);

/**************************************************************************//**
 * @brief Main interrupt processing routine for MAX86161.
 *****************************************************************************/
void maxm86161_hrm_helper_process_irq(void);

/**************************************************************************//**
 * @brief Use to check maxm86161 in proximity mode or normal mode
 *****************************************************************************/
#ifdef PROXIMITY
bool maxm86161_get_prox_mode(void);
#endif

/**************************************************************************//**
 * @brief Prints heart rate and spo2 to USB debug interface
 *****************************************************************************/
void hrm_helper_output_debug_message(int16_t heart_rate, int16_t spo2);

/**************************************************************************//**
 * @brief Prints samples to USB debug interface
 *****************************************************************************/
void hrm_helper_output_raw_sample_debug_message(maxm86161_hrm_irq_sample_t *sample);


#endif /* MAXM86161_HELPER_H_ */
