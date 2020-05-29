
#include "esp_err.h"


typedef enum {
    SC_NONE = 0,
    SC_SCANNING,
    SC_SCAN_DONE
} smart_config_status_t;

typedef struct
{
	smart_config_status_t status;

} smart_config_t;




extern void smart_config_start();
