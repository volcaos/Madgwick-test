
#include "esp_err.h"


#define NVS_DATA_SIZE_MAX	64
#define NVS_STORAGE_NAME	"storage"

extern void nvs_init();
extern esp_err_t nvs_get_str_ex( char *key, char *value );
extern esp_err_t nvs_set_str_ex( char *key, char *value );
esp_err_t nvs_erase_key_ex( char *key );

