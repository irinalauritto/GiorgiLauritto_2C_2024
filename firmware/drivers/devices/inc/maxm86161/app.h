/*
 * app.h
 *
 *  Created on: 8 jun. 2023
 *      Author: rego_
 */

#ifndef APP_H_
#define APP_H_

#include <stdint.h>
#include <stdbool.h>


#define HRM_DEMO_NAME "si117xHRM Demo"
#define HRM_DEMO_VERSION "1.0.0"

/*  External Events  */
// constantes de interrupcion
#define MAXM86161_IRQ_EVENT     0x1
#define BTN0_IRQ_EVENT          0x2

//estados de medicion
typedef enum hrm_spo2_state
{
   HRM_STATE_IDLE,
   HRM_STATE_NOSIGNAL,
   HRM_STATE_ACQUIRING,
   HRM_STATE_ACTIVE,
   HRM_STATE_INVALID
}hrm_spo2_state_t;

/**************************************************************************//**
 * @brief Initialize the HRM application.
 *****************************************************************************/
uint32_t hrm_init_app(void);

/**************************************************************************//**
 * @brief Process HRM IRQ events.
 *****************************************************************************/
void hrm_process_event(uint32_t event_flags);

/**************************************************************************//**
 * @brief HRM process main loop.
 *****************************************************************************/
void hrm_loop(void);

/**************************************************************************//**
 * @brief This function returns the current heart rate.
 *****************************************************************************/
int16_t hrm_get_heart_rate(void);

/**************************************************************************//**
 * @brief This function returns the current finger contact status.
 *****************************************************************************/
bool hrm_get_status(void);

/**************************************************************************//**
 * @brief This function returns the current SpO2.
 *****************************************************************************/
int16_t hrm_get_spo2(void);



#endif /* APP_H_ */
