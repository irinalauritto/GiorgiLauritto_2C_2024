/*
 * maxim61861.c
 *
 *  Created on: 28 dic. 2022
 *      Author: rego_
 */

#include "hrm_helper.h"
#include "MAXM86161.h"
#include "app.h"
#include "maxm86161_hrm_config.h"

static int16_t local_heart_rate;
static int16_t local_spo2;
uint8_t sampleCnt = 0;
//#include "MAXM86161_I2C.h"
//#include <sl_app_log.h>

#include "led.h"
maxm86161_fifo_queue_t ppg_queue;
maxm86161_ppg_sample_t maxm86161_irq_queue[APP_QUEUE_SIZE];
static bool maxm86161_prox_mode = false;

/**************************************************************************//**
 * @brief Empty the samples in queue.
 *****************************************************************************/
void maxm86161_helper_sample_queue_clear(void)
{
  maxm86161_clear_queue(&ppg_queue);
}

/**************************************************************************//**
 * @brief Query number of entries in the queue.
 *****************************************************************************/
int32_t maxm86161_hrm_helper_sample_queue_numentries(void)
{
  int16_t count=0;

  count = maxm86161_num_samples_in_queue(&ppg_queue);
  return count;
}

/**************************************************************************//**
 * @brief Get sample from the queue.
 *****************************************************************************/
int32_t maxm86161_hrm_helper_sample_queue_get(maxm86161_hrm_irq_sample_t *samples)
{

  int ret = MAXM86161_HRM_SUCCESS;
  maxm86161_ppg_sample_t s;


 ret= maxm86161_dequeue_ppg_sample_data(&ppg_queue, &s);
  if (ret == 0) {
    samples->ppg[0] = s.ppg1;
    samples->ppg[1] = s.ppg2;
    samples->ppg[2] = s.ppg3;
	//PRINTF("s.ppg1:%d\n\r",s.ppg1);

  }
  else {
    ret = MAXM86161_HRM_ERROR_SAMPLE_QUEUE_EMPTY;
    goto Error;
  }

Error:
  return ret;
}

/**************************************************************************//**
 * @brief Initialize and clear the queue.
 *****************************************************************************/
int32_t maxm86161_hrm_helper_initialize(void)
{
  int16_t error = 0;
  maxm86161_allocate_ppg_data_queue(&ppg_queue,
                                    maxm86161_irq_queue,
                                    APP_QUEUE_SIZE * MAXM86161DRV_PPG_SAMPLE_SIZE_BYTES);
  maxm86161_helper_sample_queue_clear();
  return error;
}

/**************************************************************************//**
 * @brief Main interrupt processing routine for MAX86161.
 *****************************************************************************/
#ifdef PROXIMITY
void maxm86161_hrm_helper_process_irq(void)
{
  uint8_t reg_status;
  uint8_t ppg_sr_status;

  maxm86161_i2c_read_from_register(MAXM86161_REG_IRQ_STATUS1, &reg_status);
//  maxm86161_i2c_read_from_register(MAXM86161_REG_IRQ_ENABLE1, &reg_status);

  if (reg_status & MAXM86161_INT_1_FULL) {
      maxm86161_read_samples_in_fifo(&ppg_queue);
  }

  if (reg_status & MAXM86161_INT_1_PROXIMITY_INT) {
    maxm86161_i2c_read_from_register(MAXM86161_REG_PPG_CONFIG2, &ppg_sr_status);
    if ((ppg_sr_status >> 3) == 0x0A) {
      maxm86161_prox_mode = true;
    }
    else {
      maxm86161_prox_mode = false;
    }
  }
}
#else
void maxm86161_hrm_helper_process_irq(void)
{
  uint8_t reg_status;

  maxm86161_i2c_read_from_register(MAXM86161_REG_IRQ_STATUS1, &reg_status);
  if (reg_status & MAXM86161_INT_1_FULL) {
    maxm86161_read_samples_in_fifo(&ppg_queue);
    //printf("interrup recibida\n\r");
  }
}
#endif

/**************************************************************************//**
 * @brief Use to check maxm86161 in proximity mode or normal mode
 *****************************************************************************/
#ifdef PROXIMITY
bool maxm86161_get_prox_mode(void)
{
  return maxm86161_prox_mode;
}
#endif

/**************************************************************************//**
 * @brief Prints heart rate and spo2 to USB debug interface
 *****************************************************************************/
void hrm_helper_output_debug_message(int16_t heart_rate, int16_t spo2)
{
 printf(">Heart_rate:%d,SpO2:%d\r\n", heart_rate, spo2);
}

/**************************************************************************//**
 * @brief Prints samples to USB debug interface
 *****************************************************************************/
void hrm_helper_output_raw_sample_debug_message(maxm86161_hrm_irq_sample_t *sample)
{
	printf(">hrm:%ld,red:%ld,ir:%ld\r\n", sample->ppg[0], sample->ppg[1], sample->ppg[2]);
}
/*==================[end of file]============================================*/
