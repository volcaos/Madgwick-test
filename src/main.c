/* WiFi station Example



*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "sdkconfig.h"

#include "wifi.h"
#include "smart_config.h"
#include "mpu9250.h"
#include "madgwick.h"

#define BLINK_GPIO GPIO_NUM_2

static const char *TAG = "MY1STAPP";

TaskHandle_t imu_task_handle = NULL;
TaskHandle_t led_task_handle = NULL;
bool enableDoFScan;

/*
static void uart_rx_task()
{
	static const char *RX_TASK_TAG = "UART_RX_TASK";
	esp_log_level_set(RX_TASK_TAG,ESP_LOG_INFO);
    uint8_t *rx_buf = (uint8_t*)malloc(RX_BUF_SIZE+1);
    while(1){
        const int rxBytes = uart_read_bytes(UART_NUM_0, rx_buf, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if( rxBytes > 0 ) {
            rx_buf[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, rx_buf);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, rx_buf, rxBytes, ESP_LOG_INFO);
        }
    }
    free(rx_buf);
}
*/
#if 0
void wifi_init_sta()
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}
#endif

static void led_task()
{
    uint64_t t0=0, t1;
    bool blink = false;
    //
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    //
    while(1){
        t1 = xTaskGetTickCount() / portTICK_PERIOD_MS;
        if( !blink && t1-t0 >= 950 ){
            t0 = t1;
            blink = true;
        }else if( blink && t1-t0 >= 50 ){
            t0 = t1;
            blink = false;
        }
        gpio_set_level(BLINK_GPIO,blink);
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
    gpio_set_level(BLINK_GPIO,false);
}



static void imu_task()
{
    const uint64_t interval = 10;
    mpu9250_data_t d = {0};
    uint64_t t0=0, t1;
    float q[4];
    float v[3];
    //
	vTaskDelay(100/portTICK_PERIOD_MS);
    mpu9250_init();
    enableDoFScan = true;
    // 
    while(1){
        t1 = xTaskGetTickCount() / portTICK_PERIOD_MS;
        if( enableDoFScan && t1-t0 > interval ){
            t0 = t1 - interval;
            mpu9250_read_acc(&d);
            mpu9250_read_cmp(&d);
            madgwick_filterIMU2(0.01f,d.gyr_x,d.gyr_y,d.gyr_z,d.acc_x,d.acc_y,d.acc_z);
            //madgwick_filterAHRS(0.01f,d.gyr_x,d.gyr_y,d.gyr_z,d.acc_x,d.acc_y,d.acc_z,d.cmp_x,d.cmp_y,d.cmp_z);
            read_quaternion(q);
            read_eulerAngle(v);
            //ESP_LOGI(TAG,"%llu,%.3f,%.3f,%.3f,%.3f",t1,q[0],q[1],q[2],q[3]);
            //ESP_LOGI(TAG,"%.3f,%.3f,%.3f",v[0],v[1],v[2]);
            printf("%.3f,%.3f,%.3f,%.3f\n",q[0],q[1],q[2],q[3]);
            //printf("%.3f,%.3f,%.3f\n",v[0],v[1],v[2]);
            //
            //printf("%f,%f,%f,%f,%f,%f\n",d.acc_x,d.acc_y,d.acc_z,d.gyr_x,d.gyr_y,d.gyr_z);
        }
        vTaskDelay(1/portTICK_PERIOD_MS);
    } 
}


static void main_task()
{
    while(1){
        int c = getchar();
        switch(c){
            case '0':
                ESP_LOGI(TAG,"smart_config_start");
                smart_config_start();
                break;
            case '1':
                ESP_LOGI(TAG,"wifi_connect");
                wifi_connect();
                break;
            case '2':
                ESP_LOGI(TAG,"wifi_disconnect");
                wifi_disconnect();
                break;
            case '3':
                ESP_LOGI(TAG,"mpu9250_init");
                mpu9250_init();
                break;
            case '4':
                for( int i=0; i<10; i++ ){
                    mpu9250_data_t d = {0};
                    mpu9250_read_acc(&d);
                    mpu9250_read_cmp(&d);
                    //
                    ESP_LOGI(TAG,"%f,%f,%f, %f,%f,%f, %f,%f,%f, %f",
                        d.acc_x,d.acc_y,d.acc_z,
                        d.gyr_x,d.gyr_y,d.gyr_z,
                        d.cmp_x,d.cmp_y,d.cmp_z,
                        d.temp
                        );
                    // 
                    vTaskDelay(1/portTICK_PERIOD_MS);
                }
                break;
            case '5':
                ak8963_selftest();
                break;
            case '6':
                enableDoFScan = !enableDoFScan;                
                break;
            default:
                break;
        }
        // 


        //
        vTaskDelay(1);
    }
}

void app_main()
{
    // init
    ESP_ERROR_CHECK( nvs_flash_init() );
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // 
    xTaskCreate(led_task,"led_task",1024*1,NULL,configMAX_PRIORITIES,&led_task_handle);
    xTaskCreate(imu_task,"imu_task",1024*2,NULL,configMAX_PRIORITIES,&imu_task_handle);
    xTaskCreate(main_task,"main_task",1024*2,NULL,configMAX_PRIORITIES,NULL);
}
