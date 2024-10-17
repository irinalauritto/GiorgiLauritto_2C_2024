# Ejemplo MAXM86161

Este proyecto ejemplifica el uso del dispositivo MAXM86161.

En el mismo se obtiene cada 50 milisegundos el estado del dedo, el nivel de SpO2 y la frecuencia cardíaca.

### Hardware requerido

* ESP-EDU
* Módulo MAXM86161
* Módulo LCD

Además de el coneccionado del bus I2C, es necesario conectar el pin EN del módulo a 3.3V y la salida INT a un pin del ESP32, el ejemplo utiliza el pin 1. 

### Ejecutar la aplicación

1. Luego de grabar la placa, correr el `Serial Monitor`
2. Se podrá observar una salida con los valores como se muestra a continuación:
```PowerShell
.
.
.
Estado del dedo: 0 --- SpO2: 0 --- Heart_rate: 0
Estado del dedo: 0 --- SpO2: 0 --- Heart_rate: 0
Estado del dedo: 1 --- SpO2: 99 --- Heart_rate: 66
Estado del dedo: 1 --- SpO2: 99 --- Heart_rate: 65
Estado del dedo: 1 --- SpO2: 99 --- Heart_rate: 68
Estado del dedo: 1 --- SpO2: 99 --- Heart_rate: 62
.
.
.
``