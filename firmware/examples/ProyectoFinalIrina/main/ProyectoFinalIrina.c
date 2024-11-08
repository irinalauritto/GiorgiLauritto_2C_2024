/*! @mainpage Proyecto Final Integrador
 *
 * @section genDesc General Description
 *
 * Este es el proyecto final presentado en la ctaedra de Electronica Programable, donde el 
 * código adquiere datos de ECG, calcula parámetros como la frecuencia cardíaca y diagnostica arritmias, 
 * como bradicardia y taquicardia. Estos datos se envían a través de BLE para su visualización en una aplicación móvil.
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 02/04/2024 | Proyecto Final Integrador	                     |
 *
 * @author Josefina Giorgi (josefina.giorgi@ingenieria.uner.edu.ar) y Irina Lauritto (irina.lauritto@ingenieria.uner.edu.ar)
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
/** @brief Periodo para mostrar el diagnóstico (ms) */ 
#define CONFIG_BLINK_PERIOD_DIAGNOSTICO 1000

/** @brief Periodo asociado a BLE (ms) */ 
#define CONFIG_BLINK_PERIOD 500

/** @brief LED utilizado para la notificación de estado Bluetooth */ 
#define LED_BT LED_1

/** @brief Tamaño del buffer de datos de ECG */ 
#define BUFFER_SIZE 500

/** @brief Frecuencia de muestreo en Hz */ 
#define SAMPLE_FREQ 220

/** @brief Retardo en ms para la adquisición de señal de ECG */ 
#define RETARDO_ECG 5000

/** @brief Número de muestras en cada fragmento (chunk) de señal */ 
#define CHUNK 4

/** @brief Periodo de señal en ms */ 
#define T_SENIAL 4000

/*==================[internal data definition]===============================*/
/** @brief Buffer para almacenar los datos de ECG */
float ecg[BUFFER_SIZE];

/** @brief Maneja la tarea de visualización de datos */
TaskHandle_t mostrarTaskHandle = NULL;

/** @brief Maneja la tarea de adquisición y procesamiento de ECG */
TaskHandle_t adquirirProcesarECGTaskHandle = NULL;

/** @brief Maneja la tarea de cálculo de parámetros ECG */
TaskHandle_t calcularParametrosECGTaskHandle = NULL;

/** @brief Maneja la tarea de diagnóstico */
TaskHandle_t mandarDiagnosticoTaskHandle = NULL;

/** @brief Dato de conversión del canal analógico */
uint16_t datoConversionAD;

/** @brief Indicador de detección de bradicardia */
bool BRADICARDIA = false;

/** @brief Indicador de detección de taquicardia */
bool TAQUICARDIA = false;

/** @brief Indicador de procesamiento en curso */
bool PROCESANDO = false;

/** @brief Indicador de frecuencia cardíaca normal */
bool FRECUENCIA_NORMAL = false;

/** @brief Indicador de filtro activado */
bool filter = false;

/** @brief Variable para almacenar la frecuencia cardíaca calculada */
float frecuenciaCardiaca;

/** @brief Almacenamiento temporal para los datos filtrados */
static float ecg_filt[CHUNK];

/** @brief Límite superior para detección de taquicardia (bpm) */
uint8_t limiteTaquicardia = 91;

/** @brief Límite inferior para detección de bradicardia (bpm) */
uint8_t limiteBradicardia = 59;

/*==================[internal functions declaration]=========================*/

/**
 * @brief Función de temporización para la adquisición de datos de ECG.
 * @param param Puntero a parámetros de función (opcional).
 * @details Activa la tarea de adquisición de datos ECG cada vez que se cumple el temporizador.
 */
void funcTimerECG(void* param){
	vTaskNotifyGiveFromISR(adquirirProcesarECGTaskHandle, pdFALSE);
}

/**
 * @brief Tarea para adquirir y procesar la señal de ECG.
 * @param pvParameter Puntero a parámetros de función (opcional).
 * @details Lee el valor del canal analógico para obtener datos de ECG y los almacena en el buffer hasta completarlo; luego, activa la tarea de cálculo de parámetros.
 */
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

/**
 * @brief Tarea para calcular los parámetros de ECG, como la frecuencia cardíaca y la detección de arritmias.
 * @param pvParameter Puntero a parámetros de función (opcional).
 * @details Procesa el buffer de ECG para contar complejos QRS y calcular la frecuencia cardíaca, indicando taquicardia, bradicardia o ritmo normal según los valores obtenidos.
 */
static void calcularParametrosECG(void *pvParameter){
    while (true)
    {
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

/**
 * @brief Función de temporización para la mostrar de datos de ECG.
 * @param param Puntero a parámetros de función (opcional).
 * @details Activa la tarea de mostrar datos  cada vez que se cumple el temporizador.
 */
void FuncTimerSenial(void* param){
    xTaskNotifyGive(mostrarTaskHandle);
}

/**
 * @brief Función ejecutada ante una interrupción de recepción de datos por BLE.
 * @param data Puntero a los datos recibidos.
 * @param length Longitud del array de datos recibidos.
 * @details Activa o desactiva el filtro de señal de acuerdo con los datos recibidos.
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
 * @brief Tarea para el procesamiento y envío de datos de ECG a través de BLE.
 * @param pvParameter Puntero a parámetros de función (opcional).
 * @details Procesa los datos de ECG con filtros pasa bajo y pasa alto y los envía mediante BLE en segmentos para visualización.
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
        
        }
}

/**
 * @brief Tarea para el envío de diagnóstico basado en la frecuencia cardíaca calculada.
 * @param pvParameter Puntero a parámetros de función (opcional).
 * @details Envía el diagnóstico de taquicardia, bradicardia o frecuencia normal según la frecuencia cardíaca detectada.
 */
static void mandarDiagnostico(void *pvParameter){
  char frecuencia[128];  
  char valor[128];
  while(true){

            if(TAQUICARDIA)
         {
            sprintf(valor,"*J%.2f",frecuenciaCardiaca);
            sprintf(frecuencia, "*J%s", ": Tiene Taquicardia.\n");
            strcat(valor,frecuencia);
            BleSendString(valor);
         }

         if(BRADICARDIA)
         {
            sprintf(valor,"*J%.2f",frecuenciaCardiaca);
            sprintf(frecuencia, "*J%s", ": Tiene Bradicardia.\n ");
            strcat(valor,frecuencia);
            BleSendString(valor);
        }

         if(FRECUENCIA_NORMAL)
         {
            sprintf(valor,"*J%.2f",frecuenciaCardiaca);
            sprintf(frecuencia, "*J%s", ": Frecuencia normal.\n");
            strcat(valor,frecuencia);
            BleSendString(valor);
         }

        vTaskDelay(CONFIG_BLINK_PERIOD_DIAGNOSTICO / portTICK_PERIOD_MS);
    }
}
/*==================[external functions definition]==========================*/
/**
 * @brief Función principal del programa. Configura y ejecuta las tareas de adquisición de ECG y comunicación BLE.
 * @details Inicializa la configuración de BLE, temporizadores y tareas para la obtención de datos de ECG, cálculo de parámetros y diagnóstico.
 */
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
    xTaskCreate(&calcularParametrosECG, "calcularParametrosECG", 2048, NULL, 5, &calcularParametrosECGTaskHandle);
    xTaskCreate(&mostrar, "mostrar", 2048, NULL, 5, &mostrarTaskHandle);
    xTaskCreate(&mandarDiagnostico, "mandarDiagnostico", 2048, NULL, 5, &mandarDiagnosticoTaskHandle);
   
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
