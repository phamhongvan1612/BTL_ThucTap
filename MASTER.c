// ==================== FILE 1: ESP32 MASTER ====================
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_now.h"
#include "esp_sntp.h"
#include "cJSON.h"

#define WIFI_SSID "Long"
#define WIFI_PASS "16181985"
#define FIREBASE_URL "https://btl1-1451d-default-rtdb.asia-southeast1.firebasedatabase.app/"

static const char *TAG = "ESP_MASTER";

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t wifi_event_group;
uint8_t wifi_channel = 1;

uint8_t slave1_mac[] = {0xAC, 0x15, 0x18, 0xD5, 0x81, 0x4C}; // DHT11
uint8_t slave2_mac[] = {0xAC, 0x15, 0x18, 0xD5, 0x49, 0x8C}; // BH1750

typedef struct {
    uint8_t request_id;
} request_packet_t;

typedef struct {
    uint8_t id;
    float value1;
    float value2;
} sensor_data_t;

QueueHandle_t firebase_queue;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "📶 Đã kết nối Wi-Fi");
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        wifi_event_handler, NULL, NULL);

    esp_wifi_start();

    ESP_LOGI(TAG, "⏳ Chờ kết nối Wi-Fi...");
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                                           false, true, pdMS_TO_TICKS(10000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "✅ Kết nối Wi-Fi thành công");

        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            wifi_channel = ap_info.primary;
            ESP_LOGI(TAG, "📡 Kênh Wi-Fi: %d", wifi_channel);
        }
    } else {
        ESP_LOGE(TAG, "❌ Không thể kết nối Wi-Fi sau thời gian chờ");
    }

    esp_netif_dns_info_t dns;
    ip4_addr_t dns_ip;
    IP4_ADDR(&dns_ip, 8, 8, 8, 8);
    memcpy(&dns.ip.u_addr.ip4, &dns_ip, sizeof(dns_ip));
    dns.ip.type = IPADDR_TYPE_V4;
    esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns);
}

void sync_time() {
    ESP_LOGI(TAG, "🌐 Đồng bộ thời gian NTP...");
    setenv("TZ", "ICT-7", 1);
    tzset();

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0, retry_count = 10;

    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "⌛ Đang chờ NTP (%d)...", retry);
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (retry < retry_count) {
        ESP_LOGI(TAG, "✅ Thời gian đồng bộ: %s", asctime(&timeinfo));
    } else {
        ESP_LOGE(TAG, "❌ Lỗi đồng bộ thời gian!");
    }
}

void get_time_string(char *buffer, size_t len) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(buffer, len, "%d-%m-%Y %H:%M:%S", &timeinfo);
}

bool is_wifi_connected() {
    EventBits_t bits = xEventGroupGetBits(wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT);
}

void firebase_send_json(const char *path, const char *json_str) {
    if (!is_wifi_connected()) {
        ESP_LOGW(TAG, "⚠️ Không có Wi-Fi, không gửi Firebase.");
        return;
    }

    char url[512];
    time_t now = time(NULL);
    snprintf(url, sizeof(url), "%s%s/%ld.json", FIREBASE_URL, path, (long)now);
    ESP_LOGI(TAG, "🔗 Gửi tới: %s", url);
    
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_PUT,  // dùng PUT để ghi bản ghi mới theo thời gian
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 10000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_str, strlen(json_str));

    for (int attempt = 1; attempt <= 3; attempt++) {
        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            int status = esp_http_client_get_status_code(client);
            ESP_LOGI(TAG, "✅ Firebase OK (HTTP %d)", status);
            break;
        } else {
            ESP_LOGE(TAG, "❌ Lỗi lần %d: %s", attempt, esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    esp_http_client_cleanup(client);
}


void on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    sensor_data_t sensor;
    memcpy(&sensor, data, sizeof(sensor));
    ESP_LOGI(TAG, "📩 ESP-NOW: ID=%d, Val1=%.2f, Val2=%.2f", sensor.id, sensor.value1, sensor.value2);
    xQueueSend(firebase_queue, &sensor, portMAX_DELAY);
}

void firebase_task(void *pvParameters) {
    sensor_data_t sensor;
    while (1) {
        if (xQueueReceive(firebase_queue, &sensor, portMAX_DELAY)) {
            cJSON *root = cJSON_CreateObject();
            char path[64];

            float val1 = roundf(sensor.value1 * 10) / 10.0f;
            float val2 = roundf(sensor.value2 * 10) / 10.0f;

            char str_val1[16], str_val2[16];
            snprintf(str_val1, sizeof(str_val1), "%.1f", val1);
            snprintf(str_val2, sizeof(str_val2), "%.1f", val2);

            if (sensor.id == 1) {
                cJSON_AddStringToObject(root, "temperature", str_val1);
                cJSON_AddStringToObject(root, "humidity", str_val2);
                strcpy(path, "/sensor/dht11");
            } else if (sensor.id == 2) {
                cJSON_AddStringToObject(root, "lux", str_val1);
                strcpy(path, "/sensor/bh1750");
            }

            char time_str[32];
            get_time_string(time_str, sizeof(time_str));
            cJSON_AddStringToObject(root, "timestamp", time_str);

            char *json_str = cJSON_PrintUnformatted(root);
            firebase_send_json(path, json_str);
            free(json_str);
            cJSON_Delete(root);
        }
    }
}

void send_request(uint8_t *mac, uint8_t id) {
    request_packet_t req = {.request_id = id};
    esp_now_send(mac, (uint8_t*)&req, sizeof(req));
}

void app_main(void) {
    nvs_flash_init();
    wifi_init_sta();
    sync_time();

    ESP_ERROR_CHECK(esp_now_init());

    esp_now_peer_info_t peer = {.encrypt = false, .channel = wifi_channel};

    memcpy(peer.peer_addr, slave1_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    memcpy(peer.peer_addr, slave2_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    esp_now_register_recv_cb(on_data_recv);

    firebase_queue = xQueueCreate(10, sizeof(sensor_data_t));
    xTaskCreate(firebase_task, "firebase_task", 4096, NULL, 5, NULL);

    while (1) {
        send_request(slave1_mac, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        send_request(slave2_mac, 2);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
