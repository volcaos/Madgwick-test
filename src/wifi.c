#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_smartconfig.h"
#include "esp_log.h"
#include "tcpip_adapter.h"

#include "wifi.h"

static const char *TAG = "WIFI";
static EventGroupHandle_t s_wifi_event_group;
static ip4_addr_t s_ip_addr;
int reconnect_count;

static void on_got_ip( void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data )
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t*)event_data;
    memcpy(&s_ip_addr, &event->ip_info.ip, sizeof(s_ip_addr));
    xEventGroupSetBits(s_wifi_event_group, BIT(0));  // GOT_IPV4_BIT 
}

static void on_wifi_disconnect( void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data )
{
    ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
    if( ++reconnect_count < 5 ){
        ESP_ERROR_CHECK(esp_wifi_connect());
    } 
}

static void wifi_start()
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    // 
    reconnect_count = 0;
    ESP_LOGI(TAG, "Connecting to %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

static void wifi_stop()
{
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip));
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_deinit());
}

esp_err_t wifi_connect()
{
    if( s_wifi_event_group != NULL ){
        return ESP_ERR_INVALID_STATE;
    }
    s_wifi_event_group = xEventGroupCreate();
    wifi_start();
    xEventGroupWaitBits(s_wifi_event_group, BIT0, true, true, portMAX_DELAY);    // GOT_IPV4_BIT
    ESP_LOGI(TAG, "Connected to %s", wifi_config.sta.ssid);
    ESP_LOGI(TAG, "IPv4 address: " IPSTR, IP2STR(&s_ip_addr));
    return ESP_OK;
}

esp_err_t wifi_disconnect()
{
    if( s_wifi_event_group == NULL ){
        return ESP_ERR_INVALID_STATE;
    }
    vEventGroupDelete(s_wifi_event_group);
    s_wifi_event_group = NULL;
    wifi_stop();
    ESP_LOGI(TAG, "Disconnected from %s", wifi_config.sta.ssid);
    return ESP_OK;
}
