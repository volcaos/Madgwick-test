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
#include "nvs.h"

#include "tcpip_adapter.h"
#include "lwip/sockets.h"


#define BLINK_GPIO GPIO_NUM_2
#define NVS_NAME "storage"

static const char *TAG = "main";

TaskHandle_t imu_task_handle = NULL;
TaskHandle_t led_task_handle = NULL;
TaskHandle_t udp_task_handle = NULL;
bool enableDoFScan;
bool useAHRS = false;
SemaphoreHandle_t xBinSemaphore;

#define HOST_IP_ADDR	"192.168.1.108"
#define HOST_PORT		8080

static char udp_buf[128];

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




static void udp_task()
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;
	//
	struct sockaddr_in dest_addr;
	dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_port = htons(HOST_PORT);
	addr_family = AF_INET;
	ip_protocol = IPPROTO_IP;
	inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
	// 
	int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
	if( sock < 0 ){
		ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
	}
	ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, HOST_PORT);
	// 
    while(1){
		float q[4];
		if( xSemaphoreTake(xBinSemaphore,0) != pdTRUE ){
			vTaskDelay(1);
			continue;
		}
		read_quaternion(q);
		int err = 0;
		sprintf((char*)udp_buf,"%.6f,%.6f,%.6f,%.6f\n",q[0],q[1],q[2],q[3]);
		err = sendto(sock, udp_buf, strlen(udp_buf), 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
		if( err < 0 ){
			ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
			break;
		}
		/*
		struct sockaddr_in source_addr;
		socklen_t socklen = sizeof(source_addr);
		int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
		if( len < 0 ){
			ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
			break;
		}else{
			rx_buffer[len] = 0;
			ESP_LOGI(TAG, "RX:%s", rx_buffer);
		}
		*/
		// 
		xSemaphoreGive(xBinSemaphore);
		vTaskDelay(1);
	}
    vTaskDelete(NULL);
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
	enableDoFScan = false;
	gyro_error_rs = 5.0f;
	gyro_error_rss = 0.2f;
	enableDoFScan = true;
	useAHRS = true;
	// 
	while(1){
		t1 = xTaskGetTickCount() / portTICK_PERIOD_MS;
		if( enableDoFScan && t1-t0 > interval ){
			BaseType_t xStatus = xSemaphoreTake(xBinSemaphore,1000U);
			t0 = t1 - interval;
			mpu9250_read_acc(&d);
			mpu9250_read_cmp(&d);
			if( useAHRS ){
				madgwick_filterIMU2(0.01f,d.gyr_x,d.gyr_y,d.gyr_z,d.acc_x,d.acc_y,d.acc_z);
			}else{
				madgwick_filterAHRS(0.01f,d.gyr_x,d.gyr_y,d.gyr_z,d.acc_x,d.acc_y,d.acc_z,d.cmp_x,d.cmp_y,d.cmp_z);
			}
			read_quaternion(q);
			read_eulerAngle(v);
			xSemaphoreGive(xBinSemaphore);
		}
		vTaskDelay(1/portTICK_PERIOD_MS);
	} 
}


static void main_task()
{
	char buf[NVS_DATA_SIZE_MAX] = {0};
	char cmd[NVS_DATA_SIZE_MAX] = {0};
	char key[NVS_DATA_SIZE_MAX] = {0};
	char val[NVS_DATA_SIZE_MAX] = {0};
	// 
	while(1){
		int c = getchar();
		//
		if( c == '\n' ){
			memset(cmd,0,sizeof(cmd));
			memset(key,0,sizeof(key));
			memset(val,0,sizeof(val));
			sscanf(buf,"%s %s %s",cmd,key,val);
			memset(buf,0,sizeof(buf));
			//
			putchar(c);		// ECHO
			//
			if( strcmp(cmd,"get") == 0 && strlen(key) > 0 ){
				nvs_get_str_ex(key,val);
			}else if( strcmp(cmd,"set") == 0 && strlen(key) > 0 ){
				nvs_set_str_ex(key,val);
			}else if( strcmp(cmd,"del") == 0 && strlen(key) > 0 ){
				nvs_erase_key_ex(key);
			}else if( strcmp(cmd,"info") == 0 ){
				tcpip_adapter_ip_info_t ipInfo; 
				tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);
				printf("My IP: " IPSTR "\n", IP2STR(&ipInfo.ip));
			}else if( strcmp(cmd,"imu") == 0 ){
				enableDoFScan = ( strcmp(key,"0") == 0 ) ? false : true;
			}else if( strcmp(cmd,"mode") == 0 ){
				useAHRS = ( strcmp(key,"0") == 0 ) ? false : true;
			}else if( strcmp(cmd,"rss") == 0 ){
				gyro_error_rs = atoff(key);
				gyro_error_rss = atoff(val);
			}
		}else if( c == '\b' ){
			if( strlen(buf) > 0 ){
				buf[strlen(buf)-1] = '\0';
				putchar(c);		// ECHO
			}
		}else if( strlen(buf) >= NVS_DATA_SIZE_MAX-1 ){
			
		}else if( c == '\r' ){
			
		}else if( c > 0 ){
			char cs[2] = { (char)c, '\0' };
			strcat(buf,cs);
			putchar(c);		// ECHO
		}
		//
		vTaskDelay(1);
	}
}

void app_main()
{
	nvs_init();
	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	{
		char ssid[NVS_DATA_SIZE_MAX] = {0};
		char password[NVS_DATA_SIZE_MAX] = {0};
		//
		if( nvs_get_str_ex("ssid",ssid) == ESP_OK ){
			printf("ssid: %s\n",ssid);
			strcpy((char*)(wifi_config.sta.ssid),ssid);
		}
		if( nvs_get_str_ex("password",password) == ESP_OK ){
			printf("password: %s\n",password);
			strcpy((char*)(wifi_config.sta.password),password);
		}
		if( strlen((char*)(wifi_config.sta.ssid)) > 0 && strlen((char*)wifi_config.sta.password) > 0 ){
			wifi_connect();
		}
	}
	// 
	xBinSemaphore = xSemaphoreCreateBinary();
	//
	xTaskCreate(led_task,"led_task",1024*1,NULL,configMAX_PRIORITIES,&led_task_handle);
	xTaskCreate(imu_task,"imu_task",1024*2,NULL,configMAX_PRIORITIES,&imu_task_handle);
	xTaskCreate(udp_task,"udp_task",1024*10,NULL,configMAX_PRIORITIES,&udp_task_handle);
	xTaskCreate(main_task,"main_task",1024*2,NULL,configMAX_PRIORITIES,NULL);
}
