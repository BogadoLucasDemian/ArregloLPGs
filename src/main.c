#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

// Definición de pines (GPIOs)
#define PIN_STEP GPIO_NUM_18
#define PIN_DIR  GPIO_NUM_19

// Datos del motor Epson EM-257
#define PASOS_PARA_36_GRADOS 20  // (36° / 1.8° por paso en Full Step)
#define RETARDO_PULSO_US     1000 // Retardo en microsegundos entre pulsos

void mover_pasos(int pasos) {
    for (int i = 0; i < pasos; i++) {
        gpio_set_level(PIN_STEP, 1);
        esp_rom_delay_us(RETARDO_PULSO_US);
        gpio_set_level(PIN_STEP, 0);
        esp_rom_delay_us(RETARDO_PULSO_US);
    }
}

void app_main(void) {
    // Configuración del pin STEP como salida
    gpio_reset_pin(PIN_STEP);
    gpio_set_direction(PIN_STEP, GPIO_MODE_OUTPUT);

    // Configuración del pin DIR como salida
    gpio_reset_pin(PIN_DIR);
    gpio_set_direction(PIN_DIR, GPIO_MODE_OUTPUT);

    // Fijar dirección inicial (1 = Horario, 0 = Antihorario)
    gpio_set_level(PIN_DIR, 1);
    gpio_set_level(PIN_STEP, 0);

    while (1) {
        // 1. Dar los 20 pasos para avanzar 36 grados
        mover_pasos(PASOS_PARA_36_GRADOS);

        // 2. Esperar 3 segundos (convertidos a ticks de FreeRTOS)
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}