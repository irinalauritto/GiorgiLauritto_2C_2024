#ifndef MAXM86161_HRM_CONFIG_H_
#define MAXM86161_HRM_CONFIG_H_

#define PPG_1_SCALER  0 // green, if SNR is too small, can set SCALER to 1 or 2
#define PPG_2_SCALER  0 // ir, if SNR is too small, can set SCALER to 1 or 2
#define PPG_3_SCALER  0 // red, if SNR is too small, can set SCALER to 1 or 2

#define DEBUG_NONE    0
#define PPG_LEVEL     1
#define HRM_LEVEL     2
#define ALL_LEVEL     3
#define UART_DEBUG    ALL_LEVEL

//#define PROXIMITY

#define PROX_USE_GREEN  (1<<0) // use GREEN led for proximity is default
#define PROX_USE_IR     (1<<1)
#define PROX_USE_RED    (1<<2)
#define PROX_SELECTION  PROX_USE_GREEN

#define APP_QUEUE_SIZE              50 /* Number of sample in 2 seconds*/

#endif /* MAXM86161_HRM_CONFIG_H_ */
