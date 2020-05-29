#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "mpu9250.h"

#define PIN_NUM_MISO	GPIO_NUM_12
#define PIN_NUM_MOSI	GPIO_NUM_13
#define PIN_NUM_CLK		GPIO_NUM_14
#define PIN_NUM_CS		GPIO_NUM_15

#define ACC_FS 2.0f
#define GYR_FS 1000.0f
#define CMP_MSENSE (32760.0f/4912.0f)


static const char *TAG = "SPI";

spi_device_handle_t spi;

typedef struct
{
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t gyr_x;
	int16_t gyr_y;
	int16_t gyr_z;
	int16_t cmp_x;
	int16_t cmp_y;
	int16_t cmp_z;
	int16_t temp;
} mpu9250_raw_data_t;


uint8_t ASA[3];
float gyro_offset[3];




// 
esp_err_t mpu9250_write_reg( spi_device_handle_t spi, uint8_t reg, uint8_t data )
{
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t,0,sizeof(t));
	t.addr = reg & 0x7F;
	t.tx_buffer = &data;
	t.length = 8;
	ESP_ERROR_CHECK(spi_device_acquire_bus(spi,portMAX_DELAY));
	ESP_ERROR_CHECK(ret=spi_device_polling_transmit(spi,&t));
	spi_device_release_bus(spi);
	return ret;
}
// 
esp_err_t mpu9250_read_reg( spi_device_handle_t spi, uint8_t reg, uint8_t *data, uint32_t len )
{
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t,0,sizeof(t));
	t.addr = reg | 0x80;
	t.length = len * 8;
	t.rxlength = len * 8;
	t.rx_buffer = data;
	ESP_ERROR_CHECK(spi_device_acquire_bus(spi,portMAX_DELAY));
	ESP_ERROR_CHECK(ret=spi_device_polling_transmit(spi,&t));
	spi_device_release_bus(spi);
	return ret;
}
// 
esp_err_t ak8963_write_reg( spi_device_handle_t spi, uint8_t reg, uint8_t data )
{
	mpu9250_write_reg(spi,MPU9250_I2C_SLV0_ADDR,AK8963_I2C_ADDR);
	mpu9250_write_reg(spi,MPU9250_I2C_SLV0_REG,reg);
	mpu9250_write_reg(spi,MPU9250_I2C_SLV0_DO,data);
	mpu9250_write_reg(spi,MPU9250_I2C_SLV0_CTRL,0x81);
	return ESP_OK;
}
// 
esp_err_t ak8963_read_reg( spi_device_handle_t spi, uint8_t reg, uint8_t *data, uint32_t len )
{
	mpu9250_write_reg(spi,MPU9250_I2C_SLV0_ADDR,AK8963_I2C_ADDR|0x80);
	mpu9250_write_reg(spi,MPU9250_I2C_SLV0_REG,reg);
	mpu9250_write_reg(spi,MPU9250_I2C_SLV0_CTRL,0x80+len);
	mpu9250_read_reg(spi,MPU9250_EXT_SENS_DATA_00,data,len);
	return ESP_OK;
}
//
void ak8963_wait_DRDY()
{
	int cnt = 0;
	uint8_t d;
	while(cnt++<100){
		ak8963_read_reg(spi,AK8963_ST1,&d,1);
		if( d & 0x01 ){
			ESP_LOGI(TAG,"cnt:%d",cnt);
			break;
		}
		vTaskDelay(1);
	}
}
//
void mpu9250_read_acc( mpu9250_data_t *d )
{
	uint8_t buf[14];
	// 
	mpu9250_read_reg(spi,MPU9250_ACCEL_XOUT_H,buf,14);
	//
	d->acc_x = (int16_t)(buf[0]<<8|buf[1]) / (float)INT16_MAX * ACC_FS;
	d->acc_y = (int16_t)(buf[2]<<8|buf[3]) / (float)INT16_MAX * ACC_FS;
	d->acc_z = (int16_t)(buf[4]<<8|buf[5]) / (float)INT16_MAX * ACC_FS;
	d->temp  = (int16_t)(buf[6]<<8|buf[7]) / 333.87f + 21.0f;
	d->gyr_x = (int16_t)(buf[8]<<8|buf[9]) / (float)INT16_MAX * GYR_FS / 360.0f * 2.0f * M_PI;
	d->gyr_y = (int16_t)(buf[10]<<8|buf[11]) / (float)INT16_MAX * GYR_FS / 360.0f * 2.0f * M_PI;
	d->gyr_z = (int16_t)(buf[12]<<8|buf[13]) / (float)INT16_MAX * GYR_FS / 360.0f * 2.0f * M_PI;
	d->gyr_x -= gyro_offset[0];
	d->gyr_y -= gyro_offset[1];
	d->gyr_z -= gyro_offset[2];
}
//
void mpu9250_read_cmp( mpu9250_data_t *d )
{
	uint8_t buf[7];
	// 
	//ak8963_wait_DRDY();
	ak8963_read_reg(spi,AK8963_HXL,buf,7);
	//
	d->cmp_x = (int16_t)(buf[1]<<8|buf[0]) / CMP_MSENSE;
	d->cmp_y = (int16_t)(buf[3]<<8|buf[2]) / CMP_MSENSE;
	d->cmp_z = (int16_t)(buf[5]<<8|buf[4]) / CMP_MSENSE;
	d->cmp_x *= ((float)ASA[0]-128.0f)*0.5f/128.0f+1.0f;
	d->cmp_y *= ((float)ASA[1]-128.0f)*0.5f/128.0f+1.0f;
	d->cmp_z *= ((float)ASA[2]-128.0f)*0.5f/128.0f+1.0f;
}
//
void spi_pre_transfer_callback( spi_transaction_t *t )
{

}
//
void mpu9250_init()
{
	esp_err_t ret;
	spi_bus_config_t buscfg = {
		.miso_io_num = PIN_NUM_MISO,
		.mosi_io_num = PIN_NUM_MOSI,
		.sclk_io_num = PIN_NUM_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = SPI_MAX_DMA_LEN
	};
	spi_device_interface_config_t devcfg = {
		.command_bits = 0,
		.address_bits = 8,
		.dummy_bits = 0,
		.mode = 3,
		.duty_cycle_pos = 128,
		.cs_ena_pretrans = 0,
		.cs_ena_posttrans = 0,
		.clock_speed_hz = 1*10*1000,  
		.spics_io_num = PIN_NUM_CS,
		.flags = 0,		// MSB
		.queue_size = 100,
		.pre_cb = spi_pre_transfer_callback,
		.post_cb = NULL,
	};
	//
	ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
	ESP_ERROR_CHECK(ret);
	ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);
	// 
	mpu9250_write_reg(spi,MPU9250_PWR_MGMT_1,0x80);	// H_RESET
	vTaskDelay(10/portTICK_PERIOD_MS);
	// Rate, LPF
	mpu9250_write_reg(spi,MPU9250_SMPLRT_DIV,0x00);
	mpu9250_write_reg(spi,MPU9250_CONFIG,0x02);			// DLPF_CFG
	mpu9250_write_reg(spi,MPU9250_GYRO_CONFIG,0b00010000);
	mpu9250_write_reg(spi,MPU9250_ACCEL_CONFIG,0x00);	// 2G
	mpu9250_write_reg(spi,MPU9250_ACCEL_CONFIG_2,0x02);	// A_DLPFCFG:2
	// I2C
	mpu9250_write_reg(spi,MPU9250_USER_CTRL,0x22);	// I2C_MST_EN|I2C_MST_RST
	mpu9250_write_reg(spi,MPU9250_I2C_MST_CTRL,0x0D);	// 400kHz
	// Reset AK8963
	ak8963_write_reg(spi,AK8963_CNTL2,0x01);	// SRST
	vTaskDelay(100/portTICK_PERIOD_MS);
	// Read Fuze ROM
	ak8963_write_reg(spi,AK8963_CNTL1,0x00);
	ak8963_write_reg(spi,AK8963_CNTL1,0x1F);
	vTaskDelay(10/portTICK_PERIOD_MS);
	ak8963_read_reg(spi,AK8963_ASAX,ASA,3);
	ESP_LOGI(TAG,"AK8963_Fuze:%02X,%02X,%02X",ASA[0],ASA[1],ASA[2]);
	ak8963_write_reg(spi,AK8963_CNTL1,0x00);
	vTaskDelay(10/portTICK_PERIOD_MS);
	// 
	ak8963_write_reg(spi,AK8963_CNTL1,0x16);	// 16bit,100Hz
	// 
	{
		gyro_offset[0] = gyro_offset[1] = gyro_offset[2] = 0.0f;
		mpu9250_data_t d = {0};
		for( int i=0; i<100; i++ ){
			mpu9250_read_acc(&d);
			gyro_offset[0] += d.gyr_x / 100.0f;
			gyro_offset[1] += d.gyr_y / 100.0f;
			gyro_offset[2] += d.gyr_z / 100.0f;
			vTaskDelay(1);
		}
	}
}

void ak8963_selftest()
{
	uint8_t tmp;
	ak8963_read_reg(spi,AK8963_CNTL1,&tmp,1);
	// Self Test
	ak8963_write_reg(spi,AK8963_CNTL1,0x00);
	ak8963_write_reg(spi,AK8963_ASTC,0x40);
	ak8963_write_reg(spi,AK8963_CNTL1,0x18);
	ak8963_wait_DRDY();
	vTaskDelay(10/portTICK_PERIOD_MS);
	for( int i=0; i<3; i++ ){
		mpu9250_data_t d = {0};
		mpu9250_read_cmp(&d);
		ESP_LOGI(TAG,"AK8963_SelfTest:%f,%f,%f",d.cmp_x,d.cmp_y,d.cmp_z);
	}
	ak8963_write_reg(spi,AK8963_ASTC,0x00);
	ak8963_write_reg(spi,AK8963_CNTL1,0x00);
	ak8963_write_reg(spi,AK8963_CNTL1,tmp);
}