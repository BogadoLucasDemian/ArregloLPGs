#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_spiffs.h"
#include "esp_http_server.h"

// ============================================================================
// 1. DEFINICIÓN DE PINES Y PARÁMETROS DEL HARDWARE
// ============================================================================
#define PIN_STEP        GPIO_NUM_18   // Pin conectado a STEP en el DRV8825
#define PIN_DIR         GPIO_NUM_19   // Pin conectado a DIR en el DRV8825
#define PIN_SENSE_POWER GPIO_NUM_34   // Pin de lectura del divisor resistivo (Censa 12V/24V)

// Retardo entre pulsos en microsegundos (determina la velocidad de giro)
#define RETARDO_PULSO_US 1000  

// ============================================================================
// 2. CONFIGURACIÓN DEL PUNTO DE ACCESO WI-FI (AP)
// ============================================================================
#define WIFI_SSID       "ESP32_Motor_Control"
#define WIFI_PASS       "12345678"
#define MAX_STA_CONN    4

static const char *TAG = "MOTOR_CONTROL";

// Cola de tareas para independizar el servidor HTTP de la generación de pulsos
static QueueHandle_t motor_queue = NULL;

// Posición absoluta acumulada del motor (en pasos)
static int32_t posicion_actual = 0;

// ============================================================================
// 3. FUNCIONES AUXILIARES DE NVS Y CONTROL DEL MOTOR
// ============================================================================
/**
 * @brief Guarda la posición acumulada actual en la memoria Flash (NVS)
 */
esp_err_t guardar_posicion_nvs(int32_t pos) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("motor_state", NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        err = nvs_set_i32(handle, "last_pos", pos);
        if (err == ESP_OK) {
            nvs_commit(handle);
        }
        nvs_close(handle);
    }
    return err;
}

/**
 * @brief Lee la última posición acumulada desde la memoria Flash (NVS)
 */
int32_t cargar_posicion_nvs(void) {
    nvs_handle_t handle;
    int32_t pos_guardada = 0; // Valor por defecto si no existe registro previo
    esp_err_t err = nvs_open("motor_state", NVS_READONLY, &handle);
    if (err == ESP_OK) {
        nvs_get_i32(handle, "last_pos", &pos_guardada);
        nvs_close(handle);
    }
    return pos_guardada;
}

/**
 * @brief Verifica si la fuente del motor está encendida leyendo el GPIO 34
 * @return true si hay tensión, false si la fuente está apagada
 */
bool fuente_alimentacion_activa(void) {
    // GPIO 34 en 1 significa que el divisor resistivo entrega tensión (~3.3V)
    return (gpio_get_level(PIN_SENSE_POWER) == 1);
}

/**
 * @brief Genera una secuencia de pulsos en el pin STEP para mover el motor
 * @param pasos Cantidad de pasos físicos a ejecutar
 */
void mover_pasos(int pasos) {
    for (int i = 0; i < pasos; i++) {
        gpio_set_level(PIN_STEP, 1);        // Pulso HIGH
        esp_rom_delay_us(RETARDO_PULSO_US);
        gpio_set_level(PIN_STEP, 0);        // Pulso LOW
        esp_rom_delay_us(RETARDO_PULSO_US);
    }
}

/**
 * @brief Tarea de FreeRTOS que atiende las órdenes provenientes de la interfaz Web
 */
void motor_task(void *pvParameters) {
    int pasos_a_mover = 0;
    
    // Al iniciar la tarea, recuperamos la última posición almacenada en NVS
    posicion_actual = cargar_posicion_nvs();
    ESP_LOGI(TAG, "Posición inicial cargada desde NVS: %ld pasos", (long)posicion_actual);

    while (1) {
        // Bloquea en espera de comandos sin consumir CPU
        if (xQueueReceive(motor_queue, &pasos_a_mover, portMAX_DELAY)) {
            if (pasos_a_mover < 0) {
                gpio_set_level(PIN_DIR, 0); // Sentido Antihorario
                mover_pasos(-pasos_a_mover);
            } else if (pasos_a_mover > 0) {
                gpio_set_level(PIN_DIR, 1); // Sentido Horario
                mover_pasos(pasos_a_mover);
            }

            // Actualizar la posición absoluta y persistirla en NVS
            posicion_actual += pasos_a_mover;
            guardar_posicion_nvs(posicion_actual);
            ESP_LOGI(TAG, "Movimiento completado. Nueva posición absoluta: %ld pasos", (long)posicion_actual);
        }
    }
}

// ============================================================================
// 4. SISTEMA DE ARCHIVOS Y MANEJADORES DEL SERVIDOR HTTP
// ============================================================================
/**
 * @brief Inicializa la memoria Flash SPIFFS para leer el index.html
 */
void init_spiffs(void) {
    ESP_LOGI(TAG, "Inicializando sistema de archivos SPIFFS...");
    
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = "spiffs",
      .max_files = 5,
      .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Error al montar o formatear la partición SPIFFS.");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "No se encontró la partición SPIFFS.");
        } else {
            ESP_LOGE(TAG, "Error al inicializar SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS montado exitosamente. Total: %d bytes, Usado: %d bytes", total, used);
    }
}

/**
 * @brief HTTP GET / : Sirve el archivo index.html almacenado en SPIFFS
 */
esp_err_t get_root_handler(httpd_req_t *req) {
    FILE* f = fopen("/spiffs/index.html", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Error al abrir /spiffs/index.html");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    char chunk[512];
    size_t chunksize;
    httpd_resp_set_type(req, "text/html");

    while ((chunksize = fread(chunk, 1, sizeof(chunk), f)) > 0) {
        if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {
            fclose(f);
            return ESP_FAIL;
        }
    }

    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/**
 * @brief HTTP GET /api/status : Devuelve la posición actual y el estado de la fuente
 */
esp_err_t get_status_handler(httpd_req_t *req) {
    char resp[96];
    bool pwr = fuente_alimentacion_activa();
    snprintf(resp, sizeof(resp), "{\"position\":%ld,\"power\":%s}", (long)posicion_actual, pwr ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

/**
 * @brief HTTP POST /api/move : Recibe "steps=X" y verifica la fuente antes de mover
 */
esp_err_t post_move_handler(httpd_req_t *req) {
    // Validar si la fuente de alimentación está encendida
    if (!fuente_alimentacion_activa()) {
        ESP_LOGE(TAG, "Intento de movimiento rechazado: Fuente de motor apagada (GPIO 34 = 0)");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Fuente del motor apagada");
        return ESP_FAIL;
    }

    char buf[100];
    int ret = httpd_req_recv(req, buf, req->content_len);
    
    if (ret > 0) {
        buf[ret] = '\0';
        int pasos = 0;
        
        if (sscanf(buf, "steps=%d", &pasos) == 1) {
            ESP_LOGI(TAG, "Instrucción recibida desde el frontend: %d pasos", pasos);
            
            if (xQueueSend(motor_queue, &pasos, pdMS_TO_TICKS(100)) == pdTRUE) {
                char resp[64];
                snprintf(resp, sizeof(resp), "{\"status\":\"OK\",\"position\":%ld}", (long)(posicion_actual + pasos));
                httpd_resp_set_type(req, "application/json");
                httpd_resp_sendstr(req, resp);
                return ESP_OK;
            } else {
                ESP_LOGE(TAG, "Cola saturada, se rechaza la petición");
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cola de tareas llena");
                return ESP_FAIL;
            }
        }
    }

    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Parámetros inválidos");
    return ESP_FAIL;
}

/**
 * @brief HTTP POST /api/home : Retorna a la posición 0 verificando la fuente
 */
esp_err_t post_home_handler(httpd_req_t *req) {
    if (!fuente_alimentacion_activa()) {
        ESP_LOGE(TAG, "Intento de retorno a cero rechazado: Fuente de motor apagada");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Fuente del motor apagada");
        return ESP_FAIL;
    }

    int32_t pasos_para_cero = -posicion_actual;
    
    if (pasos_para_cero == 0) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"status\":\"OK\",\"position\":0}");
        return ESP_OK;
    }

    int pasos = (int)pasos_para_cero;
    if (xQueueSend(motor_queue, &pasos, pdMS_TO_TICKS(100)) == pdTRUE) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"status\":\"OK\",\"position\":0}");
        return ESP_OK;
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cola de tareas llena");
        return ESP_FAIL;
    }
}

/**
 * @brief HTTP POST /api/set_zero : Forzar la posición actual a 0 en RAM y NVS sin mover el motor
 */
esp_err_t post_set_zero_handler(httpd_req_t *req) {
    posicion_actual = 0;
    guardar_posicion_nvs(posicion_actual);
    ESP_LOGI(TAG, "Posición reiniciada manualmente a 0 por el usuario");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"OK\",\"position\":0}");
    return ESP_OK;
}

/**
 * @brief Registra los endpoints HTTP en el servidor web de ESP-IDF
 */
void iniciar_servidor_web(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = get_root_handler };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t status_uri = { .uri = "/api/status", .method = HTTP_GET, .handler = get_status_handler };
        httpd_register_uri_handler(server, &status_uri);

        httpd_uri_t move_uri = { .uri = "/api/move", .method = HTTP_POST, .handler = post_move_handler };
        httpd_register_uri_handler(server, &move_uri);

        httpd_uri_t home_uri = { .uri = "/api/home", .method = HTTP_POST, .handler = post_home_handler };
        httpd_register_uri_handler(server, &home_uri);

        httpd_uri_t set_zero_uri = { .uri = "/api/set_zero", .method = HTTP_POST, .handler = post_set_zero_handler };
        httpd_register_uri_handler(server, &set_zero_uri);
        
        ESP_LOGI(TAG, "Servidor HTTP iniciado en http://192.168.4.1");
    }
}

// ============================================================================
// 5. INICIALIZACIÓN DE LA RED WI-FI (SoftAP)
// ============================================================================
void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = 1,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Punto de Acceso Wi-Fi activado. SSID: %s | Pass: %s", WIFI_SSID, WIFI_PASS);
}

// ============================================================================
// 6. FUNCIÓN PRINCIPAL DE ENTRADA (app_main)
// ============================================================================
void app_main(void) {
    // 1. Inicializar la memoria Flash NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Montar el sistema de archivos SPIFFS
    init_spiffs();

    // 3. Configuración de pines para el DRV8825 y lectura de potencia
    gpio_reset_pin(PIN_STEP);
    gpio_set_direction(PIN_STEP, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_DIR);
    gpio_set_direction(PIN_DIR, GPIO_MODE_OUTPUT);
    
    gpio_set_level(PIN_STEP, 0);
    gpio_set_level(PIN_DIR, 1);

    // Configurar GPIO 34 como entrada de lectura digital (solamente entrada, sin pull-up/down interno)
    gpio_reset_pin(PIN_SENSE_POWER);
    gpio_set_direction(PIN_SENSE_POWER, GPIO_MODE_INPUT);

    // 4. Crear cola de comunicación y lanzar la tarea en el Core 1
    motor_queue = xQueueCreate(10, sizeof(int));
    xTaskCreatePinnedToCore(motor_task, "motor_task", 2048, NULL, 5, NULL, 1);

    // 5. Iniciar Wi-Fi y Servidor HTTP
    wifi_init_softap();
    iniciar_servidor_web();
}