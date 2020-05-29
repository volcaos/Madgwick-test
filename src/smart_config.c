#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_smartconfig.h"
#include "esp_log.h"

#include "wifi.h"
#include "smart_config.h"


static EventGroupHandle_t s_sc_event_group;

static const int ESPTOUCH_DONE_BIT = BIT0;
static const char *TAG = "smartconfig";

static void smartconfig_task();

static void on_wifi_event( void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data )
{
    ESP_LOGI(TAG, "### on_wifi_event: %d",event_id);
    // 
    if( event_id == WIFI_EVENT_STA_START ){
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START");
        ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
        smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
        ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
    }else if( event_id == WIFI_EVENT_STA_DISCONNECTED ){
        ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED");
        //esp_wifi_connect();
    }
    ESP_LOGI(TAG,"--------------");
}

static void on_sc_event( void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data )
{
    ESP_LOGI(TAG, "### on_sc_event: %d",event_id);
    // 
    if( event_id == SC_EVENT_SCAN_DONE ){
        ESP_LOGI(TAG, "SC_EVENT_SCAN_DONE");
    }else if( event_id == SC_EVENT_FOUND_CHANNEL ){
        ESP_LOGI(TAG, "SC_EVENT_FOUND_CHANNEL");
    }else if( event_id == SC_EVENT_GOT_SSID_PSWD ){
        ESP_LOGI(TAG, "SC_EVENT_GOT_SSID_PSWD");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t*)event_data;
        wifi_config_t wc;
        bzero(&wc, sizeof(wc));
        memcpy(wc.sta.ssid, evt->ssid, sizeof(wc.sta.ssid));
        memcpy(wc.sta.password, evt->password, sizeof(wc.sta.password));
        wc.sta.bssid_set = evt->bssid_set;
        if( wc.sta.bssid_set == true ){
            memcpy(wc.sta.bssid, evt->bssid, sizeof(wc.sta.bssid));
        }
        memcpy(&wifi_config,&wc,sizeof(wc));
        // 
        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
        ESP_ERROR_CHECK( esp_wifi_connect() );

    }else if( event_id == SC_EVENT_SEND_ACK_DONE ){
        ESP_LOGI(TAG, "SC_EVENT_SEND_ACK_DONE");
        xEventGroupSetBits(s_sc_event_group, ESPTOUCH_DONE_BIT);
    }
    ESP_LOGI(TAG,"--------------");
}


static void smartconfig_task()
{
    s_sc_event_group = xEventGroupCreate();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &on_sc_event, NULL) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );

    while(1){
        EventBits_t uxBits = xEventGroupWaitBits(s_sc_event_group, ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY); 
        if( uxBits & ESPTOUCH_DONE_BIT ) {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

void smart_config_start()
{
    xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
}