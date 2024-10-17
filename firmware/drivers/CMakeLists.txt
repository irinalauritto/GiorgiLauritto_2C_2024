idf_build_get_property(target IDF_TARGET)

# Always compiled source files
set(srcs
    "microcontroller/src/gpio_mcu.c"
    "microcontroller/src/i2c_mcu.c"
    "microcontroller/src/delay_mcu.c"
    "microcontroller/src/timer_mcu.c"
    "microcontroller/src/uart_mcu.c"
    "microcontroller/src/spi_mcu.c"
    "microcontroller/src/pwm_mcu.c"
    "microcontroller/src/gpio_fast_out_mcu.c"
    "microcontroller/src/analog_io_mcu.c"
    "devices/src/led.c"
    "devices/src/switch.c"
    "devices/src/lcditse0803.c"
    "devices/src/hc_sr04.c"
    "devices/src/ws2812b.c"
    "devices/src/neopixel_stripe.c"
    "devices/src/ili9341.c"
    "devices/src/fonts.c"
    "devices/src/Si7007.c"
    #"devices/src/CCS811.c"
    #"devices/src/sensor_server.c"
    "devices/src/servo_sg90.c"
    "devices/src/ADXL335.c"
    "devices/src/mpu6050.c"
    #"devices/src/MFRC522.c"
    "devices/src/rfid_utils.c"
    "devices/src/max3010X.c"
    "devices/src/spo2_algorithm.c"
    "devices/src/heartRate.c"
    "devices/src/maxm86161/hrm_helper.c"
    "devices/src/maxm86161/maxm86161_hrm_spo2.c"
    "devices/src/maxm86161/MAXM86161_I2C.c"
    "devices/src/maxm86161/MAXM86161.c"
    "devices/src/maxm86161/app.c"
    )

# Always included headers
set(includes "microcontroller/inc"
             "devices/inc"
             "devices/inc/maxm86161")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes}
                       REQUIRES driver esp_adc spi_flash esp_http_server esp_wifi nvs_flash)