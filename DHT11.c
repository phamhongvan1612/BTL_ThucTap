// ==================== FILE 2: ESP32 SLAVE DHT11 ====================
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"

#define DHT11_PIN GPIO_NUM_4
static const char *TAG = "DHT11_SLAVE";

uint8_t esp_master_mac[] = {0xA0, 0xDD, 0x6C, 0xB2, 0x3C, 0x3C};

typedef struct {
    uint8_t request_id;
} request_packet_t;

typedef struct {
    uint8_t id;
    float value1;
    float value2;
} response_packet_t;

int8_t a[5] = {0};
void delay_us(uint32_t us) { esp_rom_delay_us(us); }
void gpio_high() { gpio_set_level(DHT11_PIN, 1); }
void gpio_low() { gpio_set_level(DHT11_PIN, 0); }
int gpio_read() { return gpio_get_level(DHT11_PIN); }

int check_time(int expected, int timeout) {
    int count = 0;
    while (gpio_read() != expected) {
        if (++count > timeout) return -1;
        delay_us(1);
    }
    return 0;
}

void read_dht_and_respond(const uint8_t *mac) {
    memset(a, 0, sizeof(a));
    gpio_set_direction(DHT11_PIN, GPIO_MODE_OUTPUT);
    gpio_low(); vTaskDelay(pdMS_TO_TICKS(20));
    gpio_high(); delay_us(40);
    gpio_set_direction(DHT11_PIN, GPIO_MODE_INPUT);
    if (check_time(0, 100) < 0) return;
    if (check_time(1, 100) < 0) return;
    if (check_time(0, 100) < 0) return;

    for (int i = 0; i < 5; i++) for (int j = 0; j < 8; j++) {
        if (check_time(1, 100) < 0) return;
        delay_us(30);
        if (gpio_read()) a[i] |= (1 << (7 - j));
        if (check_time(0, 100) < 0) return;
    }

    if ((a[0] + a[1] + a[2] + a[3]) == a[4]) {
        float temp1 = a[2] + a[3] / (a[3] < 10 ? 10.0 : 100.0);
        float hum1  = a[0] + a[1] / (a[1] < 10 ? 10.0 : 100.0);
        response_packet_t res = {.id = 1, .value1 = temp1, .value2 = hum1};
        esp_now_send(mac, (uint8_t*)&res, sizeof(res));
    }
}

void on_request_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len == sizeof(request_packet_t)) {
        request_packet_t req;
        memcpy(&req, data, sizeof(req));
        if (req.request_id == 1) {
            read_dht_and_respond(info->src_addr);
        }
    }
}

void app_main() {
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    gpio_reset_pin(DHT11_PIN);
    gpio_set_pull_mode(DHT11_PIN, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);  // ⚠️ Thay số 6 theo channel log bên Master

    ESP_LOGI(TAG, "Wi-Fi đã khởi động.");

    ESP_ERROR_CHECK(esp_now_init());
    esp_now_peer_info_t peer = {.channel = 0, .encrypt = false};
    memcpy(peer.peer_addr, esp_master_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_request_recv));

    ESP_LOGI(TAG, "ESP-NOW DHT11 SLAVE sẵn sàng.");
}
