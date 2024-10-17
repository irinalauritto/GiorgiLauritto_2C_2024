/*! @mainpage Blinking
 *
 * \section genDesc General Description
 *
 * This section describes how the program works.
 *
 * <a href="https://drive.google.com/...">Operation Example</a>
 *
 * \section hardConn Hardware Connection
 *
 * |   LED		    |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	PIN1	 	| 	GPIO3		|
 *
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 12/09/2023 | Document creation		                         |
 *
 * @author Albano Peñalva (albano.penalva@uner.edu.ar)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
#include "app.h"
#include "gpio_mcu.h"
#include "i2c_mcu.h"
#include "lcditse0803.h"

/*==================[macros and definitions]=================================*/
#define CONFIG_HRM_PERIOD 50
/*==================[internal data definition]===============================*/
TaskHandle_t hrm_loop_handle = NULL;
TaskHandle_t hrm_process_event_handle = NULL;

/**
 * @brief This function is the ISR for the GPIO interrupt
 *
 * When the interrupt is triggered, it notifies the hrm_process_event_task
 * to run.
 */
void pint_intr_callback(void)
{
	/* Toggle the state of LED_3 */
	LedToggle(LED_3);
	/* Notify the hrm_process_event_task to run */
	vTaskNotifyGiveFromISR(hrm_process_event_handle, pdFALSE);
}


/**
 * @brief HRM process event task
 */
static void hrm_process_event_task(void *pvParameter){
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    /* La tarea espera en este punto hasta recibir una notificación */
        hrm_process_event(MAXM86161_IRQ_EVENT);
    }
}
/**
 * @brief HRM main process task	
 */
static void hrm_loop_task(void *pvParameter){
    while(true){
		hrm_loop();
		LedToggle(LED_1);
		vTaskDelay(CONFIG_HRM_PERIOD / portTICK_PERIOD_MS);	
		LcdItsE0803Write(hrm_get_heart_rate());
		printf("Estado del dedo: %d --- SpO2: %d --- Heart_rate: %d\r\n", hrm_get_status(), hrm_get_spo2(), hrm_get_heart_rate());
    }
}

/*==================[external functions definition]==========================*/
void app_main(void){
	LedsInit();
	LcdItsE0803Init();
	GPIOInit(GPIO_1, GPIO_INPUT);
	GPIOActivInt(GPIO_1, pint_intr_callback, 0, NULL);
	printf("Init MAXM86161 test.\r\n");
	I2C_initialize(400000);
	uint8_t part_id;
    	
	hrm_init_app();
    hrm_process_event(BTN0_IRQ_EVENT); /* Start the device's autonomous measurement operation. */
	
    xTaskCreate(&hrm_loop_task, "HRM LOOP", 4096, NULL, 5, &hrm_loop_handle);
    xTaskCreate(&hrm_process_event_task, "HRM PROCESS", 4096, NULL, 5, &hrm_process_event_handle);

}
/*==================[end of file]============================================*/
