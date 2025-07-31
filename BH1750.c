#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define SDA GPIO_NUM_21
#define SCL GPIO_NUM_22
#define BH1750_ADDR 0x23
#define BH1750_CMD_START 0x10

static const char *TAG = "BH1750_SLAVE";

uint8_t esp_master_mac[] = {0xA0, 0xDD, 0x6C, 0xB2, 0x3C, 0x3C}; // MAC của ESP32 Master

typedef struct {
    uint8_t request_id;
} request_packet_t;

typedef struct {
    uint8_t id;
    float value1;
    float value2;
} response_packet_t;

void bh1750_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA,
        .scl_io_num = SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "BH1750 initialized");
}

void bh1750_start() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BH1750_CMD_START, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    ESP_LOGI(TAG, "BH1750 start command sent, result: %s", esp_err_to_name(ret));
}

float bh1750_read() {
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        uint16_t raw = (data[0] << 8) | data[1];
        float lux = raw / 1.2;
        ESP_LOGI(TAG, "BH1750 raw: %d, lux: %.2f", raw, lux);
        return lux;
    } else {
        ESP_LOGE(TAG, "Failed to read BH1750: %s", esp_err_to_name(ret));
        return -1.0;
    }
}

void on_request_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len == sizeof(request_packet_t)) {
        request_packet_t req;
        memcpy(&req, data, sizeof(req));
        ESP_LOGI(TAG, "Received request from %02X:%02X:%02X:%02X:%02X:%02X with ID=%d",
            info->src_addr[0], info->src_addr[1], info->src_addr[2],
            info->src_addr[3], info->src_addr[4], info->src_addr[5],
            req.request_id);

        if (req.request_id == 2) {
            bh1750_start();
            vTaskDelay(pdMS_TO_TICKS(180)); // Wait for BH1750 measurement time
            float lux = bh1750_read();

            response_packet_t res = {
                .id = 2,
                .value1 = lux,
                .value2 = 0
            };

            esp_err_t status = esp_now_send(info->src_addr, (uint8_t *)&res, sizeof(res));
            if (status == ESP_OK) {
                ESP_LOGI(TAG, "Sent lux = %.2f to Master", lux);
            } else {
                ESP_LOGE(TAG, "Failed to send response: %s", esp_err_to_name(status));
            }
        } else {
            ESP_LOGW(TAG, "Unhandled request_id: %d", req.request_id);
        }
    }
}

void app_main() {
    ESP_LOGI(TAG, "Starting BH1750 Slave...");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // ⚠️ Set channel giống Master
    ESP_ERROR_CHECK(esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE));

    bh1750_init();
    ESP_ERROR_CHECK(esp_now_init());

    esp_now_peer_info_t peer = {.channel = 0, .encrypt = false};
    memcpy(peer.peer_addr, esp_master_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_request_recv));

    ESP_LOGI(TAG, "BH1750 Slave ready.");
}
