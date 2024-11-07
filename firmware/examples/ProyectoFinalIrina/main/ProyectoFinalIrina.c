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
#include "iir_filter.h"
/*==================[macros and definitions]=================================*/
#define CONFIG_BLINK_PERIOD 500
#define LED_BT	            LED_1
#define BUFFER_SIZE         500
#define SAMPLE_FREQ	        220
#define RETARDO_ECG         5000 //5 milisegundos
#define CHUNK               4 
#define T_SENIAL            4000 
/*==================[internal data definition]===============================*/
float ecg[BUFFER_SIZE];

TaskHandle_t mostrarTaskHandle = NULL;
TaskHandle_t adquirirProcesarECGTaskHandle = NULL;
TaskHandle_t calcularParametrosECGTaskHandle = NULL;

uint16_t datoConversionAD;

bool BRADICARDIA = false;
bool TAQUICARDIA = false;
bool PROCESANDO = false;
bool FRECUENCIA_NORMAL = false;

bool filter = false;

float frecuenciaCardiaca;

static float ecg_filt[CHUNK];

// Limites en bpm
uint8_t limiteTaquicardia = 91;
uint8_t limiteBradicardia = 59;

/*==================[internal functions declaration]=========================*/

void funcTimerECG(void* param){
	vTaskNotifyGiveFromISR(adquirirProcesarECGTaskHandle, pdFALSE);
}

static void adquirirProcesarECG(void *pvParameter){

 uint16_t i = 0;
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (PROCESANDO == false)
        {
            AnalogInputReadSingle(CH1, &datoConversionAD);

            UartSendString(UART_PC, (char *)UartItoa(datoConversionAD, 10));
            UartSendString(UART_PC, "\r");

            if (i < BUFFER_SIZE)
            {
                ecg[i] = datoConversionAD;
                i++;
                //printf("i:%d\n",i);
            }
            if (i == BUFFER_SIZE)
            {
                vTaskNotifyGiveFromISR(calcularParametrosECGTaskHandle, pdFALSE);
                i = 0;
                ecg[i] = datoConversionAD;
                i++;
            }
        }
    }
}

static void calcularParametrosECG(void *pvParameter){
    while (true)
    {
        char frecuencia[128];
        char frecCardiaca[128];
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        PROCESANDO = true;
        uint16_t umbralVoltaje = 350;
        uint16_t posicionAnteriorQRS = 0;
        float periodoCardiaco;

        // Cálculo de frecuencia cardíaca
        uint8_t contadorQRS = 0;

        uint16_t j = 0;

        while (j < BUFFER_SIZE)
        {
            if (ecg[j] > umbralVoltaje)
            {
                contadorQRS++;
                printf("contador: %d\n", contadorQRS);
                float frecuenciaAUX = frecuenciaCardiaca;
                uint16_t deltaPosicion = j - posicionAnteriorQRS;
                posicionAnteriorQRS = j;
                periodoCardiaco = deltaPosicion * 2.5 / BUFFER_SIZE; // delta en tiempo
                frecuenciaCardiaca = 1 / (periodoCardiaco/60);

            if(contadorQRS==1){
                    printf("frec:%.2f\n",frecuenciaAUX);
                    frecuenciaCardiaca=frecuenciaAUX;
                }
            else{
                    printf("frec:%.2f\n", frecuenciaCardiaca);
                }
          
        
            //Compara si tiene taquicardia o no:   
             if (frecuenciaCardiaca > limiteTaquicardia)
                {
                    TAQUICARDIA = true;
                }
                else
                {
                    TAQUICARDIA = false;
                }
                // Comparación con bradicardia
            if (frecuenciaCardiaca < limiteBradicardia)
                {
                    BRADICARDIA = true;
                }
            else
                {
                    BRADICARDIA = false;
                }
            if(limiteBradicardia<frecuenciaCardiaca && frecuenciaCardiaca<limiteTaquicardia)
            {
                FRECUENCIA_NORMAL = true;
            }
            else
            {
                FRECUENCIA_NORMAL = false;
            }
            
                j = j + 70;
            }
            j++;
        }
        
        PROCESANDO = false;
        
    
    }
}

void FuncTimerSenial(void* param){
    xTaskNotifyGive(mostrarTaskHandle);
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
        case 'A':
            filter = true;
            break;
        case 'a':
            filter = false;
            break;
    }
}

/**
 * @brief Tarea para el cálculo de la FFT y el envío de datos
 * por BLE.
 * 
 */
static void mostrar(void *pvParameter){
    char msg[128];
    char msg_chunk[24];
    char frecuencia[128];
    static uint8_t indice = 0;
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(filter){
            HiPassFilter(&ecg[indice], ecg_filt, CHUNK);
            LowPassFilter(ecg_filt, ecg_filt, CHUNK);
        } else{
            memcpy(ecg_filt, &ecg[indice], CHUNK*sizeof(float));
        }
        strcpy(msg, "");
        for(uint8_t i=0; i<CHUNK; i++){
            sprintf(msg_chunk, "*G%.2f*", ecg_filt[i]);
            strcat(msg, msg_chunk);
        }
        indice += CHUNK;

        BleSendString(msg);
        if(TAQUICARDIA)
        {
            sprintf(frecuencia, "*J%s", "Taquicardia.\n");
            BleSendString(frecuencia);
        }

        if(BRADICARDIA)
        {
            sprintf(frecuencia, "*J%s", "Bradicardia.\n ");
            BleSendString(frecuencia);
        }

        if(FRECUENCIA_NORMAL)
        {
            sprintf(frecuencia, "*J%s", "Frecuencia normal.\n");
            BleSendString(frecuencia);
        }
    }
}

/*==================[external functions definition]==========================*/
void app_main(void){

      ble_config_t ble_configuration = {
        "ESP_EDU_IRI_JOSE",
        read_data
    };
    
    timer_config_t timer_senial = {
        .timer = TIMER_B,
        .period = T_SENIAL*CHUNK,
        .func_p = FuncTimerSenial,
        .param_p = NULL
    };


    TimerInit(&timer_senial);
    LedsInit();  
    LowPassInit(SAMPLE_FREQ, 30, ORDER_2);
    HiPassInit(SAMPLE_FREQ, 1, ORDER_2);
   
    BleInit(&ble_configuration);

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
    //xTaskCreate(&calcularParametrosECG, "calcularParametrosECG", 2048, NULL, 5, &calcularParametrosECGTaskHandle);
    xTaskCreate(&mostrar, "mostrar", 2048, NULL, 5, &mostrarTaskHandle);
   
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
    TimerStart(timer_senial.timer);


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
