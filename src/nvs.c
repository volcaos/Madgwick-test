
#include "nvs.h"
#include "esp_system.h"
#include "esp_err.h"
#include "nvs_flash.h"


void nvs_init()
{
	esp_err_t err = nvs_flash_init();
	if( err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND ){
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK( err );
}

esp_err_t nvs_get_str_ex( char *key, char *value )
{
	nvs_handle_t nvs_handle;
	size_t size = NVS_DATA_SIZE_MAX;
	ESP_ERROR_CHECK(nvs_open(NVS_STORAGE_NAME,NVS_READWRITE,&nvs_handle));
	esp_err_t ret = nvs_get_str(nvs_handle,key,value,&size);
	if( ret == ESP_OK ){
		printf("NVS get: %s = %s\n",key,value);
	}
	nvs_close(nvs_handle);
	return ret;
}

esp_err_t nvs_set_str_ex( char *key, char *value )
{
	nvs_handle_t nvs_handle;
	ESP_ERROR_CHECK(nvs_open(NVS_STORAGE_NAME,NVS_READWRITE,&nvs_handle));
	esp_err_t ret = nvs_set_str(nvs_handle,key,value);
	if( ret == ESP_OK ){
		printf("NVS set: %s = %s\n",key,value);
	}
	nvs_close(nvs_handle);
	return ret;
}

esp_err_t nvs_erase_key_ex( char *key )
{
	nvs_handle_t nvs_handle;
	ESP_ERROR_CHECK(nvs_open(NVS_STORAGE_NAME,NVS_READWRITE,&nvs_handle));
	esp_err_t ret = nvs_erase_key(nvs_handle,key);
	if( ret == ESP_OK ){
		printf("NVS erase: %s\n",key);
	}
	nvs_close(nvs_handle);
	return ret;
}

