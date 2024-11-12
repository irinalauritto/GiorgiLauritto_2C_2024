/*! @mainpage Proyecto Final Integrador - Oximetría de Pulso
 *
 * @section genDesc General Description
 *
 * En este proyecto se encuentra una parte del proyecto final presentado en la cátedra de Electronica Programable por las alumnas Josefina Giorgi e Irina Lauritto.
 * Este código adquiere datos de la oximetría de pulso y envía parámetros como la frecuencia cardíaca y la saturación de oxígeno en sangre, a través de BLE para su visualización en una aplicación móvil.
 *
 * 
 *
 * \section hardConn Hardware Connection
 *
 * |    ODP Módulo  |   EDU-ESP  	|
 * |:--------------:|:--------------|
 * | 	SCL	 	    | 	SCL	    	|
 * | 	SDA 	    | 	SDA		    |
 * | 	GND	 	    | 	GND		    |
 * | 	+3V	 	    | 	+3V	        |
 * | 	EN	 	    | 	+3V	    	|
 * | 	INT	 	    | 	GPIO_1		|
 *
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 11/11/2024 | Entrega del proyecto Final Integrador.         |
 *
 * @author Josefina Giorgi (josefina.giorgi@ingenieria.uner.edu.ar) 
 * @author Irina Lauritto (irina.lauritto@ingenieria.uner.edu.ar)
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
/** @brief Periodo de parpadeo del LED en milisegundos. */
#define CONFIG_BLINK_PERIOD 500

/** @brief LED utilizado para indicar el estado de Bluetooth. */
#define LED_BT LED_1

/** @brief Retardo en milisegundos para la adquisición de datos de oximetría de pulso. */
#define RETARDO_ODP 50

/** @brief Retardo en milisegundos para mostrar los datos en pantalla. */
#define RETARDO_MOSTRAR 1000

/*==================[internal data definition]===============================*/
/** @brief Handle de la tarea para adquirir datos de oximetría de pulso. */
TaskHandle_t adquirirODP_task_handle = NULL;

/** @brief Handle de la tarea para procesar eventos de frecuencia cardíaca. */
TaskHandle_t hrm_process_event_handle = NULL;

/** @brief Handle de la tarea para mostrar los datos en pantalla. */
TaskHandle_t mostrarTaskHandle = NULL;

/** @brief Indicador de si se deben enviar los datos a través de BLE. */
bool ENVIAR_DATA = false;

/** @brief Saturación de oxígeno en sangre medida por el dispositivo. */
uint16_t saturacionOxigeno;

/** @brief Frecuencia cardíaca medida por el dispositivo. */
float frecuenciaCardiaca;

/*==================[internal functions declaration]=========================*/

/**
 * @brief Interrupción para el GPIO
 *
 * Cambia el estado del LED_3 y notifica a la tarea `hrm_process_event_task`.
 */
void pint_intr_callback(void)
{
	/* Toggle the state of LED_3 */
	LedToggle(LED_3);
	/* Notify the hrm_process_event_task to run */
	vTaskNotifyGiveFromISR(hrm_process_event_handle, pdFALSE);
}

/**
 * @brief Tarea para procesar eventos.
 *
 * Espera a recibir notificaciones para procesar eventos de oximetría de pulso.
 */
static void hrm_process_event_task(void *pvParameter){
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    /* La tarea espera en este punto hasta recibir una notificación */
        hrm_process_event(MAXM86161_IRQ_EVENT);
    }
}

/**
 * @brief Función ejecutada ante una interrupción de recepción por BLE.
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

/**
 * @brief Tarea para adquirir datos de oximetría de pulso.
 *
 * Alterna el estado de LED_1, adquiere la saturación de oxígeno y la frecuencia cardíaca.
 */
static void adquirirODP(void *pvParameter) {
 	while(true){
		hrm_loop();
		LedToggle(LED_1);
		vTaskDelay(RETARDO_ODP / portTICK_PERIOD_MS);	
		saturacionOxigeno = hrm_get_spo2();
        frecuenciaCardiaca = hrm_get_heart_rate();
    }
}

/**
 * @brief Tarea para mostrar los datos de oximetría de pulso en pantalla.
 *
 * Envía la saturación de oxígeno y la frecuencia cardíaca a través de BLE cuando ENVIAR_DATA es verdadero.
 */
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
/**
 * @brief Función principal de la aplicación.
 *
 * Inicializa el BLE y las tareas para manejar el dispositivo de oximetría.
 */
void app_main(void){
	ble_config_t ble_configuration = {
        "ESP_EDU_IRI_JOSE_OXIMETRIA",
        read_data
    };
    
    LedsInit();
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
