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
#include "ble_mcu.h"
#include "delay_mcu.h"
#include "timer_mcu.h"
#include "string.h"
/*==================[macros and definitions]=================================*/
#define CONFIG_BLINK_PERIOD 500
#define LED_BT	            LED_1
#define RETARDO_ODP 50 
#define RETARDO_MOSTRAR     1000
#define CHUNK               8
#define T_SENIAL            8000 
/*==================[internal data definition]===============================*/
TaskHandle_t adquirirODP_task_handle= NULL;
TaskHandle_t hrm_process_event_handle = NULL;
TaskHandle_t mostrarTaskHandle = NULL;

bool ENVIAR_DATA = false;
uint16_t saturacionOxigeno;
float frecuenciaCardiaca;
/*==================[internal functions declaration]=========================*/

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
 * @brief Función a ejecutarse ante un interrupción de recepción 
 * a través de la conexión BLE.
 * 
 * @param data      Puntero a array de datos recibidos
 * @param length    Longitud del array de datos recibidos
 */
void read_data(uint8_t * data, uint8_t length){
    switch(data[0]){
        case 'D':
            ENVIAR_DATA = true;
            break;
        case 'd':
            ENVIAR_DATA = false;
            break;
    }
}
static void adquirirODP(void *pvParameter) {
 	while(true){
		hrm_loop();
		LedToggle(LED_1);
		vTaskDelay(RETARDO_ODP / portTICK_PERIOD_MS);	
		//LcdItsE0803Write(hrm_get_heart_rate());
		saturacionOxigeno = hrm_get_spo2();
        frecuenciaCardiaca = hrm_get_heart_rate();
    }
}

//void FuncTimerSenial(void *param)
//{
 //   xTaskNotifyGive(mostrarTaskHandle);
//}

static void mostrar(void *pvParameter)
{
	
    while(1)
    {
        char msgSaturacion[128];
        char msgFrecuencia[128];
        char valorSaturacion[10];
        char valorFrecuencia[10];

        if(ENVIAR_DATA)
        {	

            sprintf(valorSaturacion, "%d", saturacionOxigeno); // Convierte saturacionOxigeno a string
            strcat(valorSaturacion, "\n");

            sprintf(valorFrecuencia, "%.2f", frecuenciaCardiaca);
            strcat(valorFrecuencia, "\n");

            sprintf(msgFrecuencia, "*F%s", valorFrecuencia);
            sprintf(msgSaturacion, "*O%s", valorSaturacion);

            BleSendString(msgSaturacion);
            BleSendString(msgFrecuencia);
         }

    vTaskDelay(RETARDO_MOSTRAR / portTICK_PERIOD_MS);
    }
}

/*==================[external functions definition]==========================*/
void app_main(void){
	ble_config_t ble_configuration = {
        "ESP_EDU_IRI_JOSE_OXIMETRIA",
        read_data
    };
    
    /*timer_config_t timer_senial = {
        .timer = TIMER_B,
        .period = T_SENIAL*CHUNK,
        .func_p = FuncTimerSenial,
        .param_p = NULL
    };*/

    //TimerInit(&timer_senial);
    LedsInit();
	//LcdItsE0803Init();
	GPIOInit(GPIO_1, GPIO_INPUT);
	GPIOActivInt(GPIO_1, pint_intr_callback, 0, NULL);
	printf("Init MAXM86161 test.\r\n");
	I2C_initialize(400000);
	uint8_t part_id;

	BleInit(&ble_configuration);

  

	hrm_init_app();
    hrm_process_event(BTN0_IRQ_EVENT); /* Start the device's autonomous measurement operation. */
	
	//Creacion de tareas
    xTaskCreate(&hrm_process_event_task, "HRM PROCESS", 4096, NULL, 5, &hrm_process_event_handle);	
	xTaskCreate(&adquirirODP, "Adquirir ODP", 4096, NULL, 5, &adquirirODP_task_handle);
	xTaskCreate(&mostrar, "mostrar", 2048, NULL, 5, &mostrarTaskHandle);

	//TimerStart(timer_senial.timer);


    while(1){
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
        switch(BleStatus()){
            case BLE_OFF:
                LedOff(LED_BT);
            break;
            case BLE_DISCONNECTED:
                LedToggle(LED_BT);
            break;
            case BLE_CONNECTED:
                LedOn(LED_BT);
            break;
        }
    }
}
/*==================[end of file]============================================*/
