//* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ESP32 Secure OTA Bridge (Courier)
  * @project        : Demedukit Secure OTA System
  * @author         : hrnkrc
  * @date           : December 2025
  * @description    : Acts as a secure bridge between the update server and STM32.
  * - Fetches encrypted/signed firmware via HTTP.
  * - Handshakes with STM32 Bootloader via UART.
  * - Streams data chunks with flow control.
  * - Manages version control via NVS.
  ******************************************************************************
  */
/* USER CODE END Header */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_http_client.h"

/* ========================================================================== */
/* CONFIGURATION & DEFINITIONS                                                */
/* ========================================================================== */

/* Network Credentials (consider moving to Kconfig for production) */
#define ESP_WIFI_SSID           "Harun_A55"
#define ESP_WIFI_PASS           "123456789"       
#define ESP_SERVER_BASE_URL     "http://xx.xxx.xxx.xx:8000"
#define ESP_URL_FIRMWARE        ESP_SERVER_BASE_URL "/secure_app.bin"

/* Hardware Pinout */
#define ESP_UART_PORT           UART_NUM_2
#define ESP_UART_TX_PIN         25
#define ESP_UART_RX_PIN         26

/* Secure Bootloader Protocol (Must match STM32) */
#define BL_CMD_INIT             0xABU
#define BL_CMD_ERASE            0x50U
#define BL_CMD_WRITE            0x51U
#define BL_CMD_VERIFY           0x53U
#define BL_CMD_JUMP             0x52U
#define BL_ACK                  0xC1U

static const char *TAG = "SECURE_BRIDGE";
static bool g_wifi_connected = false;

/* ========================================================================== */
/* HELPERS & DRIVERS                                                          */
/* ========================================================================== */

/**
 * @brief Initialize UART for communication with STM32.
 * Baudrate: 115200, 8N1
 */
void ESP_UART_Init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(ESP_UART_PORT, 512, 0, 0, NULL, 0);
    uart_param_config(ESP_UART_PORT, &uart_config);
    uart_set_pin(ESP_UART_PORT, ESP_UART_TX_PIN, ESP_UART_RX_PIN, -1, -1);
}

/**
 * @brief Retrieve the last flashed firmware version from Non-Volatile Storage.
 */
int32_t ESP_NVS_GetVersion(void) {
    nvs_handle_t h;
    int32_t ver = 0;
    if (nvs_open("ota_storage", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_i32(h, "fw_version", &ver);
        nvs_close(h);
    }
    return ver;
}

/**
 * @brief Save the new firmware version to NVS after successful update.
 */
void ESP_NVS_SetVersion(int32_t ver) {
    nvs_handle_t h;
    if (nvs_open("ota_storage", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "fw_version", ver);
        nvs_commit(h);
        nvs_close(h);
    }
}

/**
 * @brief Wait for ACK byte from STM32 within a timeout window.
 */
bool ESP_Target_WaitAck(uint32_t timeout_ms) {
    uint8_t rx;
    int len = uart_read_bytes(ESP_UART_PORT, &rx, 1, pdMS_TO_TICKS(timeout_ms));
    return (len > 0 && rx == BL_ACK);
}

/* ========================================================================== */
/* WIFI EVENT HANDLER                                                         */
/* ========================================================================== */

static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi Disconnected. Retrying...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) data;
        ESP_LOGI(TAG, "Connected! IP:" IPSTR, IP2STR(&event->ip_info.ip));
        g_wifi_connected = true;
    }
}

void ESP_Wifi_Init(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    wifi_config_t wifi_config = { .sta = { .ssid = ESP_WIFI_SSID, .password = ESP_WIFI_PASS } };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

/* ========================================================================== */
/* OTA CORE TASK                                                              */
/* ========================================================================== */

void OTA_Manager_Task(void *pvParameters) {
    uint8_t buffer[128]; // Data chunk buffer
    
    /* 1. Wait for Network */
    while (!g_wifi_connected) vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Syncing with Secure Server...");
    
    esp_http_client_config_t http_cfg = { 
        .url = ESP_URL_FIRMWARE,
        .timeout_ms = 10000, 
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);

    if (esp_http_client_open(client, 0) == ESP_OK) {
        esp_http_client_fetch_headers(client);

        /* -----------------------------------------------------------
           Step 1: Header Inspection & Version Control
           ----------------------------------------------------------- */
        int head_len = esp_http_client_read(client, (char*)buffer, 64);
        if (head_len < 64) {
            ESP_LOGE(TAG, "Invalid Header or Connection Error.");
            goto cleanup;
        }

        uint32_t server_ver = *(uint32_t*)&buffer[4]; // Offset 4 is Version
        int32_t current_ver = ESP_NVS_GetVersion();

        if (server_ver <= current_ver) {
            ESP_LOGI(TAG, "System Up-to-Date (Server: V%d, Device: V%d). Sleeping...", (int)server_ver, (int)current_ver);
            goto cleanup;
        }

        /* -----------------------------------------------------------
           Step 2: STM32 Handshake (Wait for Manual Reset)
           ----------------------------------------------------------- */
        ESP_LOGW(TAG, "New Firmware Found (V%d)! WAITING FOR TARGET RESET...", (int)server_ver);
        
        uint8_t init_cmd = BL_CMD_INIT;
        bool linked = false;

        /* Infinite loop until STM32 Bootloader responds */
        while (!linked) {
            // Send "Ping" to Target
            uart_write_bytes(ESP_UART_PORT, &init_cmd, 1);
            
            // Check for ACK
            if (ESP_Target_WaitAck(100)) { 
                linked = true; 
                ESP_LOGI(TAG, "Target Captured! Starting Update Process...");
            }
            
            // Non-aggressive polling (500ms) to allow user to press Reset
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        /* -----------------------------------------------------------
           Step 3: Erase Slot B
           ----------------------------------------------------------- */
        ESP_LOGI(TAG, "Commanding Target to Erase Slot B...");
        uint8_t erase_cmd = BL_CMD_ERASE;
        uart_write_bytes(ESP_UART_PORT, &erase_cmd, 1);
        
        if (!ESP_Target_WaitAck(5000)) { // Erase can take time
             ESP_LOGE(TAG, "Erase Failed or Timed Out.");
             goto cleanup;
        }

        /* -----------------------------------------------------------
           Step 4: Secure Data Streaming
           ----------------------------------------------------------- */
        ESP_LOGI(TAG, "Streaming Firmware...");
        uint8_t write_cmd = BL_CMD_WRITE;
        
        // A. Send Header First (Already buffered)
        uint8_t h_size = 64;
        uart_write_bytes(ESP_UART_PORT, &write_cmd, 1);
        uart_write_bytes(ESP_UART_PORT, &h_size, 1);
        uart_write_bytes(ESP_UART_PORT, buffer, 64); 
        if (!ESP_Target_WaitAck(1000)) goto cleanup;

        // B. Download and Stream Payload
       int total_bytes = 64;
        while (1) {
            int read_len = esp_http_client_read(client, (char*)buffer, 64);
            if (read_len <= 0) break; // End of File

            uint8_t chunk_len = (uint8_t)read_len;
            
            // Protocol: [CMD] [LEN] [DATA...]
            uart_write_bytes(ESP_UART_PORT, &write_cmd, 1);
            uart_write_bytes(ESP_UART_PORT, &chunk_len, 1);
            uart_write_bytes(ESP_UART_PORT, buffer, read_len);

            /* Flow Control: Throttle to prevent UART buffer overflow */
            vTaskDelay(pdMS_TO_TICKS(100)); 

            /* Wait for Flash Write ACK */
            if (!ESP_Target_WaitAck(2000)) { 
                ESP_LOGE(TAG, "Write Failed at offset %d", total_bytes); 
                break; 
            }
            total_bytes += read_len;
            
            // Verbose Progress Log
            if(total_bytes % 1024 == 0) ESP_LOGI(TAG, "Progress: %d bytes transmitted...", total_bytes);
        }

        /* -----------------------------------------------------------
           Step 5: Verification & Activation
           ----------------------------------------------------------- */
        ESP_LOGI(TAG, "Initiating Integrity Check...");
        uint8_t verify_cmd = BL_CMD_VERIFY;
        uart_write_bytes(ESP_UART_PORT, &verify_cmd, 1);
        
        // Wait for SHA-256 Calculation
        if (ESP_Target_WaitAck(5000)) { 
            ESP_LOGI(TAG, "SUCCESS! Firmware Verified. Rebooting Target...");
            ESP_NVS_SetVersion(server_ver); // Update Local Version
            
            // Send Jump Command
            uint8_t jump_cmd = BL_CMD_JUMP;
            uart_write_bytes(ESP_UART_PORT, &jump_cmd, 1);
        } else {
            ESP_LOGE(TAG, "Integrity Check Failed! Security Violation or Corruption.");
        }
    } else {
        ESP_LOGE(TAG, "Failed to connect to Update Server.");
    }

cleanup:
    esp_http_client_cleanup(client);
    ESP_LOGI(TAG, "OTA Task Finished. System Pending.");
    vTaskDelete(NULL);
}

/* ========================================================================== */
/* APPLICATION ENTRY POINT                                                    */
/* ========================================================================== */

void app_main(void) {
    // 1. Initialize NVS (Needed for Wi-Fi credentials)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // 2. Initialize Peripherals
    ESP_UART_Init();
    ESP_Wifi_Init();

    // 3. Start Manager Task
    xTaskCreate(OTA_Manager_Task, "OTA_Manager", 8192, NULL, 5, NULL);
}
