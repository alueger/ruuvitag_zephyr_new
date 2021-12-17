/*
 * Copyright (c) 2021 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sensirion_shtc3

#include <device.h>
#include <drivers/i2c.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <sys/byteorder.h>
#include <sys/crc.h>
#include <logging/log.h>

#include "shtc3.h"

LOG_MODULE_REGISTER(SHTC3, CONFIG_SENSOR_LOG_LEVEL);
#ifdef CONFIG_PM_DEVICE
	#include <pm/device_runtime.h>
	enum pm_device_state shtc3_pm_device_state;
#endif

#if DT_INST_NODE_HAS_PROP(0, supplyi2c_gpios)

#define SUPPLYI2C_PIN DT_INST_GPIO_PIN(0, supplyi2c_gpios)


static int set_supply_i2c(const struct device *dev, bool enable)
	{
	int err;
	struct shtc3_data *drv_data = dev->data;

	*drv_data = (struct shtc3_data){ 0 };
	drv_data->i2c = device_get_binding(DT_INST_BUS_LABEL(0));
	drv_data->pm_device_state = PM_DEVICE_STATE_OFF;

	if (drv_data->i2c == NULL) {
		LOG_INF("Failed to get pointer to %s device!",DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	#if DT_INST_NODE_HAS_PROP(0, supplyi2c_gpios)
	drv_data->supplyi2c_gpio = device_get_binding(DT_INST_GPIO_LABEL(0, supplyi2c_gpios));
	if (drv_data->supplyi2c_gpio == NULL) {
		LOG_INF("Failed to get pointer to Supply device: %s",
		DT_INST_GPIO_LABEL(0, supplyi2c_gpios));
		return -EINVAL;
	}

	err = gpio_pin_configure(drv_data->supplyi2c_gpio, SUPPLYI2C_PIN,
                GPIO_DS_DFLT_HIGH  |  GPIO_OUTPUT_INIT_LOW | DT_INST_GPIO_FLAGS(0, supplyi2c_gpios));

	if (err!= 0){
		LOG_INF("GPIO  Supply Config Error");
		return -EINVAL;
	}

	err = gpio_pin_set(drv_data->supplyi2c_gpio, SUPPLYI2C_PIN, enable);
	if (err!= 0){
		LOG_INF("GPIO Supply Set Error");
		return -EINVAL;
	}
	if (enable) {
		k_busy_wait(200000);        /* t_WAKE = 50 us */
		drv_data->pm_device_state = PM_DEVICE_STATE_ACTIVE;
		} else {
		drv_data->pm_device_state = PM_DEVICE_STATE_OFF;
		k_busy_wait(20);/* t_DWAKE = 20 us */
	}

	return 0;

	#else
	static int set_supply_i2c(const struct device *dev, bool enable)
	{
	LOG_INF(" NO GPIO Supply PIN! ");
	return 0;
	}
	#endif
}
#endif /* DT_INST_NODE_HAS_PROP */

/* all cmds read temp first: cmd[MEASURE_MODE][Clock_stretching_enabled] */
static const uint16_t measure_cmd[2][2] = {
	{ 0x7866, 0x7CA2 },
	{ 0x609C, 0x6458 },
};

/* measure_wait_us[shtc3_chip][MEASURE_MODE] */
static const uint16_t measure_wait_us[2][2] = {


	/* shtc3: 12.1ms, 0.8ms */
	{ 1210, 800 }, /* shtc3 */
	/* shtc1: 14.4ms, 0.94ms */
	{ 14400, 940 }, /* shtc1 */
};

/*
 * CRC algorithm parameters were taken from the
 * "Checksum Calculation" section of the datasheet.
 */
static uint8_t shtc3_compute_crc(uint16_t value)
{
	uint8_t buf[2];

	sys_put_be16(value, buf);
	return crc8(buf, 2, 0x31, 0xFF, false);
}

/* val = -45 + 175 * sample / (2^16) */
static void shtc3_temperature_from_raw(uint16_t raw, struct sensor_value *val)
{
	int32_t tmp;

	tmp = (int32_t)raw * 175U - (45 << 16);
	val->val1 = tmp / 0x10000;
	/* x * 1.000.000 / 65.536 == x * 15625 / 2^10 */
	val->val2 = ((tmp % 0x10000) * 15625U) / 1024;
}

/* val = 100 * sample / (2^16) */
static void shtc3_humidity_from_raw(uint16_t raw, struct sensor_value *val)
{
	uint32_t tmp;

	tmp = (uint32_t)raw * 100U;
	val->val1 = tmp / 0x10000;
	/* x * 1.000.000 / 65.536 == x * 15625 / 1024 */
	val->val2 = (tmp % 0x10000) * 15625U / 1024;
}


static int shtc3_write_command(const struct device *dev, uint16_t cmd)
{
	uint8_t tx_buf[2];

	sys_put_be16(cmd, tx_buf);
	return i2c_write(shtc3_i2c_bus(dev), tx_buf, sizeof(tx_buf), shtc3_i2c_address(dev));
}


static int shtc3_read_words(const struct device *dev, uint16_t cmd, uint16_t *data,
		     uint16_t num_words, uint16_t max_duration_us)
{
	const struct shtc3_config *cfg = dev->config;
	int status = 0;
	uint32_t raw_len = num_words * (SHTC3_WORD_LEN + SHTC3_CRC8_LEN);
	uint16_t temp16;
	uint8_t rx_buf[SHTC3_MAX_READ_LEN];
	int dst = 0;

	status = shtc3_write_command(dev, cmd);
	if (status != 0) {
		LOG_DBG("Failed to initiate read");
		return -EIO;
	}

	if (!cfg->clock_stretching) {
		k_sleep(K_USEC(max_duration_us));
	}
       
      k_busy_wait(100000); 
	status = i2c_read(shtc3_i2c_bus(dev), rx_buf, raw_len, shtc3_i2c_address(dev));
	if (status != 0) {
		LOG_DBG("Failed to read data");
		return -EIO;
	}

	for (int i = 0; i < raw_len; i += (SHTC3_WORD_LEN + SHTC3_CRC8_LEN)) {
		temp16 = sys_get_be16(&rx_buf[i]);
		if (shtc3_compute_crc(temp16) != rx_buf[i+2]) {
			LOG_DBG("invalid received invalid crc");
			return -EIO;
		}

		data[dst++] = temp16;
	}

	return 0;
}

static int shtc3_sleep(const struct device *dev)
{
	 k_busy_wait(100000);
        if (shtc3_write_command(dev, SHTC3_CMD_SLEEP) < 0) {
		return -EIO;
	}
        k_busy_wait(10000);  
	return 0;
}

static int shtc3_wakeup(const struct device *dev)
{
	 k_busy_wait(100000);
        if (shtc3_write_command(dev, SHTC3_CMD_WAKEUP)) {
		return -EIO;
	}

	k_sleep(K_USEC(30000));
	return 0;
}

static int shtc3_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct shtc3_data *data = dev->data;
	const struct shtc3_config *cfg = dev->config;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);
        k_busy_wait(100000);
	if (cfg->chip == SHTC3) {
		if (shtc3_wakeup(dev)) {
			return -EIO;
		}
	}
        k_busy_wait(100000);
	if (shtc3_read_words(dev,
			     measure_cmd[cfg->measure_mode][cfg->clock_stretching],
			     (uint16_t *)&data->sample, 2,
			     measure_wait_us[cfg->chip][cfg->measure_mode]) < 0) {
		LOG_DBG("Failed read measurements!");
		return -EIO;
	}
        k_busy_wait(100000);
	if (cfg->chip == SHTC3) {
		if (shtc3_sleep(dev)) {
			LOG_DBG("Failed to initiate sleep");
			return -EIO;
		}
	}

	return 0;
}

static int shtc3_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	const struct shtc3_data *data = dev->data;

	if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
		shtc3_temperature_from_raw(data->sample.temp, val);
	} else if (chan == SENSOR_CHAN_HUMIDITY) {
		shtc3_humidity_from_raw(data->sample.humidity, val);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api shtc3_driver_api = {
	.sample_fetch = shtc3_sample_fetch,
	.channel_get = shtc3_channel_get,
};

/***************************************************************/

static int shtc3_init(const struct device *dev)
{
	const struct shtc3_config *cfg = dev->config;
	struct shtc3_data *data = dev->data;
	uint16_t product_id;
	int err  =0 ;

	//pm_device_enable(dev);

	data->pm_device_state = PM_DEVICE_STATE_OFF;

#ifc DT_INST_NODE_HAS_PROP(0, supplyi2c_gpios)
	err = set_supply_i2c(dev,true);  //true = on, false = off
	if (err!= 0) {
		LOG_INF("GPIO Supply Set Error");
		return -EINVAL;
	}
        k_busy_wait(10000);  
#endif

	if (device_is_ready(cfg->bus) ==  0) {
		LOG_DBG("i2c bus is not ready");
		return -ENODEV;
	}
        k_busy_wait(10000);  
	//k_sleep(K_USEC(SHTC3_POWER_UP_TIME_US)); 
	if (cfg->chip == SHTC3) {
		if (shtc3_wakeup(dev)) {
			LOG_ERR("Wakeup failed");
			return -EIO;
		}
	}
            k_busy_wait(100000);   
      

	if (shtc3_write_command(dev, SHTC3_CMD_SOFT_RESET) < 0) {
		LOG_ERR("soft reset failed");
		return -EIO;
	}

	//k_sleep(K_USEC(SHTC3_SOFT_RESET_TIME_US+100 ));
       k_busy_wait(100000);
	if (shtc3_read_words(dev, SHTC3_CMD_READ_ID, &product_id, 1, 0) < 0) {
		LOG_ERR("Failed to read product id!");
		return -EIO;
	}
       
	if (cfg->chip == SHTC1) {
		if ((product_id & SHTC1_ID_MASK) != SHTC1_ID_VALUE) {
			LOG_ERR("Device is not a SHTC1");
			return -EINVAL;
		}
	}
	 
        if (cfg->chip == SHTC3) {
		if ((product_id & SHTC3_ID_MASK) != SHTC3_ID_VALUE) {
			LOG_ERR("Device is not a SHTC3");
			return -EINVAL;
		}
		
                shtc3_sleep(dev);
             
	}

	LOG_INF("Clock-stretching enabled: %d", cfg->clock_stretching);
	LOG_INF("Measurement mode: %d", cfg->measure_mode);
	LOG_INF("Init SHTC3");
	return 0;
}


/***************************************/
#ifdef CONFIG_PM_DEVICE
int shtc3_pm_ctrl(const struct device *dev, uint32_t ctrl_command,
		enum pm_device_state *state)
{
	struct shtc3_data *data = dev->data;
	int ret = 0;

	/* Set power state */
	if (ctrl_command == PM_DEVICE_STATE_SET) {
		if (*state != data->pm_device_state) {
			/* Switching from OFF to any */
				if (data->pm_device_state == PM_DEVICE_STATE_OFF) {
				/* Re-initialize the chip */
				ret = shtc3_init(dev);
			}
			/* Switching to OFF from any */
			else if (shtc3_pm_device_state == PM_DEVICE_STATE_ACTIVE) {
				/* Put the chip into sleep mode */
				ret = shtc3_sleep(dev);
				if (ret < 0)
					LOG_DBG("CTRL_MEAS write failed: %d",
						ret);
				}
			/* Store the new state */
			if (!ret)
				data->pm_device_state = shtc3_pm_device_state;
		}
	}
	/* Get power state */
	else {
		__ASSERT_NO_MSG(ctrl_command == PM_DEVICE_STATE_GET);
		*state = data->pm_device_state;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */


#define SHTC3_CONFIG(inst)						       \
	{								       \
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),		       \
		.base_address = DT_INST_REG_ADDR(inst),			       \
		.chip = DT_ENUM_IDX(DT_DRV_INST(inst), chip),		       \
		.measure_mode = DT_ENUM_IDX(DT_DRV_INST(inst), measure_mode),  \
		.clock_stretching = DT_INST_PROP(inst, clock_stretching)       \
	}

#define SHTC3_DEFINE(inst)						\
	static struct shtc3_data shtc3_data_##inst;			\
	static struct shtc3_config shtc3_config_##inst =		\
		SHTC3_CONFIG(inst);					\
                DEVICE_DT_INST_DEFINE(inst,					\
			      shtc3_init,				\
			      shtc3_pm_ctrl,					\
			      &shtc3_data_##inst,			\
			      &shtc3_config_##inst,			\
			      POST_KERNEL,				\
			      CONFIG_SENSOR_INIT_PRIORITY,		\
			      &shtc3_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SHTC3_DEFINE)
