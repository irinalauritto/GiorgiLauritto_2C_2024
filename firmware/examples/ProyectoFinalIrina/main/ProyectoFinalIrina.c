/*! @mainpage Ejemplo Bluetooth - FFT
 *
 * @section genDesc General Description
 *
 * Este proyecto ejemplifica el uso del módulo de comunicación 
 * Bluetooth Low Energy (BLE), junto con el de cálculo de la FFT 
 * de una señal.
 * Permite graficar en una aplicación móvil la FFT de una señal. 
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 02/04/2024 | Document creation		                         |
 *
 * @author Albano Peñalva (albano.penalva@uner.edu.ar)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led.h"
#include "neopixel_stripe.h"
#include "ble_mcu.h"
#include "delay_mcu.h"
#include "timer_mcu.h"
#include "uart_mcu.h"
#include "analog_io_mcu.h"


#include "fft.h"
#include "iir_filter.h"
/*==================[macros and definitions]=================================*/
TaskHandle_t adquirirProcesarECGTaskHandle = NULL;
TaskHandle_t calcularParametrosECGTaskHandle = NULL;

#define RETARDO_ECG 5000 //5 milisegundos

#define CONFIG_BLINK_PERIOD 500
#define LED_BT	            LED_1
#define BUFFER_SIZE         500
#define SAMPLE_FREQ	        220
/*==================[internal data definition]===============================*/
float ecg[BUFFER_SIZE];
static float ecg_filt[BUFFER_SIZE];
static float ecg_fft[BUFFER_SIZE/2];
static float ecg_filt_fft[BUFFER_SIZE/2];
static float f[BUFFER_SIZE/2];
TaskHandle_t fft_task_handle = NULL;

uint8_t datoConversionAD;

bool BRADICARDIA = false;
bool TAQUICARDIA = false;

uint8_t frecuenciaCardiaca;

// Limites en bpm
uint8_t limiteTaquicardia = 90;
uint8_t limiteBradicardia = 60;

uint8_t i = 0;
/*==================[internal functions declaration]=========================*/

void funcTimerECG(void* param){
	vTaskNotifyGiveFromISR(adquirirProcesarECGTaskHandle, pdFALSE);
}

static void adquirirProcesarECG(void *pvParameter){

    while(true)
    {
    	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    
        AnalogInputReadSingle(CH1, &datoConversionAD);
        
		UartSendString(UART_PC, (char*)UartItoa(datoConversionAD, 10));
		UartSendString(UART_PC, "\r");
        
        if(i<BUFFER_SIZE)
        {
            ecg[i] = datoConversionAD;
            i++;
        }
        else
        {
            i = 0;
            ecg[i] = datoConversionAD;
            i++;
        }

        if(i = BUFFER_SIZE)
        {
            vTaskNotifyGiveFromISR(calcularParametrosECGTaskHandle, pdFALSE);
        }

    }
}

static void calcularParametrosECG(void *pvParameter){
    while(true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint8_t umbralVoltaje = 200;  

        // Cálculo de frecuencia cardíaca
        uint8_t contadorQRS = 0;

        int j = 0;
        while (j<BUFFER_SIZE)
        {
            if(ecg[j]>umbralVoltaje && contadorQRS<2)
            {
                contadorQRS++;
                j = j+50;
            }

        }
        
        frecuenciaCardiaca = contadorQRS/BUFFER_SIZE; // ¿Como relacionar con el tiempo? cada 5miliseg 1 muestra. En 2,5 segundos se levanta el vector
        frecuenciaCardiaca = frecuenciaCardiaca*((1/2.5)*60);

        // Comparación con taquicardia
        if(frecuenciaCardiaca>limiteTaquicardia)
        {
            TAQUICARDIA = true;
        }
        else
        {
            TAQUICARDIA = false;
        }
        // Comparación con bradicardia
        if(frecuenciaCardiaca<limiteBradicardia)
        {
            BRADICARDIA = true;
        }
        else
        {
            BRADICARDIA = false;
        }
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
	if(data[0] == 'R'){
        xTaskNotifyGive(fft_task_handle);
    }
}

/**
 * @brief Tarea para el cálculo de la FFT y el envío de datos
 * por BLE.
 * 
 */
/*static void FftTask(void *pvParameter){
    char msg[48];
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        FFTMagnitude(ecg, ecg_fft, BUFFER_SIZE);
        HiPassFilter(ecg, ecg_filt, BUFFER_SIZE);
        LowPassFilter(ecg_filt, ecg_filt, BUFFER_SIZE);
        FFTFrequency(SAMPLE_FREQ, BUFFER_SIZE, f);
        FFTMagnitude(ecg_filt, ecg_filt_fft, BUFFER_SIZE);
        for(int16_t i=0; i<BUFFER_SIZE/2; i++){
            //Formato de datos para que sean graficados en la aplicación móvil 
            sprintf(msg, "*HX%2.2fY%2.2f,X%2.2fY%2.2f*\n", f[i], ecg_fft[i], f[i], ecg_filt_fft[i]);
            BleSendString(msg);
        }
    }
}
*/
/*==================[external functions definition]==========================*/
void app_main(void){

    timer_config_t timerECG = {
        .timer = TIMER_A,
        .period = RETARDO_ECG,
        .func_p = funcTimerECG,
        .param_p = NULL
    };
	TimerInit(&timerECG);

    // Inicialización del Convertidor AD
	analog_input_config_t convertidorAD = {
		.input = CH1,
		.mode = ADC_SINGLE,
	};
	AnalogInputInit(&convertidorAD);
	AnalogOutputInit();

    xTaskCreate(&adquirirProcesarECG, "adquirirProcesarECG", 2048, NULL, 5, &adquirirProcesarECGTaskHandle);
    xTaskCreate(&calcularParametrosECG, "calcularParametrosECG", 2048, NULL, 5, &calcularParametrosECGTaskHandle);

    //ble_config_t ble_configuration = {
      //  "ESP_EDU_1",
        //read_data
   // };

   // LedsInit();  
   // FFTInit();  
    //LowPassInit(SAMPLE_FREQ, 30, ORDER_2);
    //HiPassInit(SAMPLE_FREQ, 1, ORDER_2);
   // BleInit(&ble_configuration);

   // xTaskCreate(&FftTask, "FFT", 2048, NULL, 5, &fft_task_handle);

    /*while(1){
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
    }*/
    
    //Inicialización del puerto serie
	serial_config_t myUart = {
		.port = UART_PC,
		.baud_rate = 115200,
		.func_p = NULL,
		.param_p = NULL,
	};
	UartInit(&myUart);

    // Inicialización del conteo de timers 
    TimerStart(timerECG.timer);
}

/*==================[end of file]============================================*/
