/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*  Standard C Included Files */
//#include "MAXM86161.h"
#include "app.h"
#include <hrm_helper.h>
#include "maxm86161_hrm_config.h"
#include <stdio.h>
#include"maxm86161_hrm_spo2.h"
#include <string.h>
#include "led.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_LED_PORT 0U
//#define BOARD_LED_PIN 29U
#define BOARD_LED_PIN_2 30U
#define BOARD_LED_PIN_3 31U



#define I2C_MASTER_CLOCK_FREQUENCY (200000)
#define WAIT_TIME                  100U
#define MAXM86161_EN_GPIO_PORT    0   // Puerto 0
#define MAXM86161_EN_GPIO_PIN     0   // Pin 0

#define I2C_MASTER_SLAVE_ADDR_7BIT  0b1100010 //0x62//
#define I2C_BAUDRATE               100000U
#define I2C_DATA_LENGTH            33U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t g_master_txBuff[I2C_DATA_LENGTH];
uint8_t g_master_rxBuff[I2C_DATA_LENGTH];

/*******************************************************************************
 * Code

 ******************************************************************************/
/**************************************************************************//**
       * Global variables
 *****************************************************************************/
      /** Sensor handle for the maxm86161 hrm algorithm */
       maxm_hrm_handle_t *hrmHandle;

       /** Data Storage memory bock required by the maxm86161 hrm algorithm */
       mamx86161_hrm_data_storage_t hrm_data_storage;
       maxm86161_spo2_data_storage_t spo2_data;
       maxm86161_data_storage_t data_storage;

      /** Heart Rate result from the si117xhrm algorithm */
      static int16_t heart_rate;

      /** SpO2 result from the si117xhrm algorithm */
      static int16_t spo2;

      static int32_t hrm_status = 0;

      //static bool update_display = false;
      static bool hrm_contac_status = false;

      mamx86161_hrm_data_t hrm_data;

      /** Flag used to indicate HRM/SpO2 state */
      static volatile hrm_spo2_state_t hrm_spo2_state = HRM_STATE_IDLE;

      /**************************************************************************//**
       * Local prototypes
       *****************************************************************************/
      static void hrm_gpio_setup(void);

      /**************************************************************************//**
       * @brief Initialize the HRM application.
       *****************************************************************************/
      uint32_t hrm_init_app(void){

    	  int32_t err;

        /* Initialize peripherals*/
        hrm_gpio_setup();

        data_storage.spo2 = &spo2_data;
        data_storage.hrm = &hrm_data_storage;

        err = maxm86161_hrm_initialize(&data_storage, &hrmHandle);
       
         if (err != SL_STATUS_OK) {
           printf("Error in initialization: %ld\r\n",err);
           return err;
         }

        err = maxm86161_hrm_configure(hrmHandle, NULL, true);
       
          if (err != SL_STATUS_OK) {
            printf("Error in configuration: %ld\r\n",err);
            return err;
          }

          return SL_STATUS_OK;

      }
      /**************************************************************************//**
      * @brief Setup GPIO, enable sensor isolation switch
      *****************************************************************************/
       void hrm_gpio_setup(void)
      {
   	   
      }

      /**************************************************************************//**
       * @brief Process HRM IRQ events.
       *****************************************************************************/
      void hrm_process_event(uint32_t event_flags)
      {
    	  if (event_flags & MAXM86161_IRQ_EVENT) {
          maxm86161_hrm_helper_process_irq();
        }
        if (event_flags == BTN0_IRQ_EVENT) {
            if (hrm_spo2_state == HRM_STATE_IDLE) {
              hrm_spo2_state = HRM_STATE_ACQUIRING;
              maxm86161_hrm_run(hrmHandle);
            } else {
              hrm_spo2_state = HRM_STATE_IDLE;
              maxm86161_hrm_pause();
            }
          }
      }

      /**************************************************************************//**
       * @brief HRM process main loop.
       *****************************************************************************/
      void hrm_loop(void){
        int16_t num_samples_processed;
        int32_t err;

     //GPIO_PortSet(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN_2);

        err = maxm86161_hrm_process(hrmHandle,
                                    &heart_rate,
                                    &spo2,
                                    1,
                                    &num_samples_processed,
                                    &hrm_status, &hrm_data);

        switch (hrm_spo2_state) {
          case HRM_STATE_IDLE:
           // update_display = true;
            break;
          case HRM_STATE_ACQUIRING:
          case HRM_STATE_ACTIVE:
             //hrm_helper_output_debug_message(heart_rate, spo2);

            if((err == MAXM86161_HRM_SUCCESS)&&(hrm_status & MAXM86161_HRM_STATUS_FRAME_PROCESSED)) {
              hrm_status &= ~MAXM86161_HRM_STATUS_FRAME_PROCESSED;

      #if (UART_DEBUG & HRM_LEVEL)
            LedToggle(LED_1); 
            //hrm_helper_output_debug_message(heart_rate, spo2);
      #endif
              hrm_spo2_state = HRM_STATE_ACTIVE;
            }

      #ifdef PROXIMITY
          else if ((hrm_status & MAXM86161_HRM_STATUS_FINGER_OFF)
              || (hrm_status & MAXM86161_HRM_STATUS_SPO2_FINGER_OFF)
              || (hrm_status & MAXM86161_HRM_STATUS_ZERO_CROSSING_INVALID)
              || (maxm86161_get_prox_mode())) {
      #else
          else if ((hrm_status & MAXM86161_HRM_STATUS_FINGER_OFF)
              || (hrm_status & MAXM86161_HRM_STATUS_SPO2_FINGER_OFF)
              || (hrm_status & MAXM86161_HRM_STATUS_ZERO_CROSSING_INVALID)) {
      #endif
            heart_rate = 0;
            spo2 = 0;
            //update_display = true;
            hrm_spo2_state = HRM_STATE_ACQUIRING;
          }
          break;

          default:
            break;
          }
        }






      /**************************************************************************//**
       * @brief This function returns the current heart rate.
       *****************************************************************************/
      int16_t hrm_get_heart_rate(void)
      {
        return heart_rate;
      }

      /**************************************************************************//**
       * @brief This function returns the current finger contact status.
       *****************************************************************************/
      bool hrm_get_status(void)
      {
        hrm_contac_status = (hrm_spo2_state == HRM_STATE_ACTIVE);
        return hrm_contac_status;
      }

      /**************************************************************************//**
       * @brief This function returns the SpO2.
       *****************************************************************************/
      int16_t hrm_get_spo2(void)
      {
        return spo2;
      }





