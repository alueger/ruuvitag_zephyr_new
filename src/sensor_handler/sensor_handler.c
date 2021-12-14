#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <sensor_handler.h>
#include <drivers/gpio.h>

#include "battery_handler.h"
#include "led_handler.h"

#ifdef CONFIG_BME280
#include "bme280_handler.h"
static bool has_bme280 = false;
#endif

#ifdef CONFIG_TMP117
#include "tmp117_handler.h"
static bool has_tmp117 = false;
#endif

#ifdef CONFIG_LIS2DH12
#include "lis2dh12_handler.h"
static bool has_lis2dh12 = false;
#endif

#ifdef CONFIG_SHTC3
#include "shtc3_handler.h"
static bool has_shtc3 = false;
#endif

#ifdef CONFIG_DPS310
#include "dps310_handler.h"
static bool has_dps310 = false;
#endif


#include "ruuvi_endpoint.h"
//#include "nfc_handler.h"
#include "bt_handler.h"
#include "ruuvi.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(sensor_handler);


#define SNP_TIME	10

/*
const struct device *sensor_pwr_1;
const struct device *sensor_pwr_2;

static bool snp1_enabled = false;
static bool snp2_enabled = false;
*/
static bool has_adc = false;

static uint16_t acceleration_events = 0;
static int64_t battery_check = 0;
static sensor_data_t sensor_data = {0};
static ble_data_t buffer = { .id = {0x99, 0x04}};


static void update_battery(void){
	if(k_uptime_get() - battery_check > 10000){
		battery_check 	= k_uptime_get();
		sensor_data.vbatt 			= get_battery_level();
		LOG_DBG("Battery: %d mV", sensor_data.vbatt);
	}
}


#ifdef CONFIG_BME280
	static void update_bme(void){
	bme280_fetch();
	sensor_data.temperature = 	bme280_get_temp();
	sensor_data.pressure    = 	bme280_get_press();
	sensor_data.humidity    = 	bme280_get_humidity();
	LOG_INF("Temperature: %d, Pressure: %d, Humidity: %d", 
						sensor_data.temperature, 
						sensor_data.pressure, 
						sensor_data.humidity);
}
#endif

#ifdef CONFIG_LIS2DH12
static void update_lis2dh12(void){
	lis2dh12_fetch();
	sensor_data.x = lis2dh12_get(0);
	sensor_data.y = lis2dh12_get(1);
	sensor_data.z = lis2dh12_get(2);
	LOG_DBG("X: %d, Y: %d, Z: %d", sensor_data.x, sensor_data.y, sensor_data.z);
}
#endif

#ifdef CONFIG_SHTC3
static void update_shtc3(void){
	shtc3_fetch();
	sensor_data.temperature = shtc3_get_temp();
	sensor_data.humidity    = shtc3_get_humidity();
	LOG_DBG("Temperature: %d, Humidity: %d",sensor_data.temperature, sensor_data.humidity);
}
#endif


#ifdef CONFIG_DPS310
static void update_dps310(void){
	dps310_fetch();
	sensor_data.temperature = dps310_get_temp();
	LOG_DBG("Temperature: %d", sensor_data.temperature);
}
#endif


#ifdef CONFIG_TMP117
static void update_tmp117(void){
	tmp117_fetch();
	sensor_data.temperature = tmp117_get_temp();
	LOG_DBG("Temperature: %d", sensor_data.temperature);
}
#endif


static void package_sensor_data(void)
{
	ruuvi_raw_v2_encode(buffer.data, sensor_data, acceleration_events);
	bt_update_packet(&buffer);
	//nfc_update(&buffer);
}

void udpate_sensor_data(void)
{
	if (has_adc){
		update_battery();
	}
#ifdef CONFIG_BME280
	if (has_bme280){
		update_bme();
	}
#endif

#ifdef CONFIG_LIS2DH12
	if (has_lis2dh12){
		update_lis2dh12();
	}
#endif

#ifdef CONFIG_SHTC3
	if (has_shtc3) {
		update_shtc3();
	}
#endif

#ifdef CONFIG_DPS310 
	if (has_dps310){
	update_dps310();
	}
#endif

#ifdef CONFIG_TMP117 
	if (has_tmp117){
	update_tmp117();
	}
#endif

	package_sensor_data();
}

void sensor_init(void){
	k_msleep(50);


	has_adc = init_adc();
	if (!has_adc) {
		LOG_ERR("Failed to initialize ADC\n");
		flash_red();
	}
#ifdef CONFIG_BME280
	has_bme280 = init_bme280();
	if (!has_bme280) {
		LOG_ERR("Failed to initialize BME280\n");
		flash_red();
	}
#endif

#ifdef CONFIG_LIS2DH12
	has_lis2dh12 = init_lis2dh12();
	if (!has_lis2dh12) {
		LOG_ERR("Failed to initialize LIS2DH12\n");
		flash_red();
	}
#endif

#ifdef CONFIG_DPS310
	has_dps310 = init_dps310();
	if (!has_dps310) {
		LOG_ERR("Failed to initialize DPS310\n");
		flash_red();
	}
#endif

#ifdef CONFIG_TMP117
	has_tmp117 = init_tmp117();
	if (!has_tmp117) {
		LOG_ERR("Failed to initialize TMP117\n");
		flash_red();
	}
#endif
 
#ifdef CONFIG_SHTC3
	has_shtc3 = init_shtc3();
	if (!has_shtc3) {
		LOG_ERR("Failed to initialize SHTCx\n");
		flash_red();
	}
#endif

}
