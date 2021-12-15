/* tmp117.c - Driver for Texas TMP117 temperature  sensor */
/* 
 * 
 * SPDX-License-Identifier: Apache-2.0
 *
 */
 
 // LA 141221 07:00 

#define DT_DRV_COMPAT ti_tmp117

#include <init.h>
#include <kernel.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <sys/util.h>
#include <zephyr/types.h>

#include <stdint.h>
#include <string.h>

#include <device.h>
#include <drivers/i2c.h>

#include <logging/log.h>

#include <pm/device.h>
#include <pm/device_runtime.h>

#include "tmp117.h"
#include "tmp117_reg.h"

#ifdef CONFIG_PM_DEVICE
	#include <pm/device_runtime.h>
	enum pm_device_state tmp117_pm_device_state;
#endif

LOG_MODULE_REGISTER(tmp117, CONFIG_TMP117_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "TMP117 driver enabled without any devices"
#endif

#define SUPPLYI2C_PIN DT_INST_GPIO_PIN(0, supplyi2c_gpios)

#if DT_INST_NODE_HAS_PROP(0, supplyi2c_gpios)

#define SUPPLYI2C_PIN DT_INST_GPIO_PIN(0, supplyi2c_gpios)

static int set_supply_i2c(const struct device *dev, bool enable)
	{
	int err;
	struct tmp117_data *drv_data = dev->data;

	*drv_data = (struct tmp117_data){ 0 };
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

	/*
	 * Wakeup pin should be pulled low before initiating
	 * any I2C transfer.  If it has been tied to GND by
	 * default, skip this part.
	 */
	err = gpio_pin_configure(drv_data->supplyi2c_gpio, SUPPLYI2C_PIN,
                            GPIO_OUTPUT_INIT_LOW | DT_INST_GPIO_FLAGS(0, supplyi2c_gpios));

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
	k_busy_wait(50000);        /* t_WAKE = 50 us */
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

struct tmp117_data tmp117_data;

struct tmp117_config tmp117_config = {
	.bus_name = DT_INST_BUS_LABEL(0),
	.bus_init = tmp117_i2c_init,
	.bus_cfg = { .i2c_slv_addr = DT_INST_REG_ADDR(0) },
};

static inline struct tmp117_data *to_data(const struct device *dev)
{
	return dev->data;
}

/***************************************/
static int sample_read(const struct device *dev)
{
	struct tmp117_data *data = to_data(dev);
	uint16_t cfg_reg = 0;

	if( data->hw_tf->read_reg(dev, TMP117_REG_CONFIGURATION, &cfg_reg) < 0)
	{
		LOG_ERR("%s, Failed to read from CFGR register.",
			dev->name);
		return -EINVAL;
	}

	if ((cfg_reg & TMP117_CFGR_DATA_READY) == 0) {
		LOG_DBG("%s: no data ready", dev->name);
		return -EBUSY;
	}
	return 0;
}

/***************************************/
static int get_sample_tmp117(const struct device *dev, struct tmp117_sample *val)
{	
	struct tmp117_data *data = to_data(dev);
	uint16_t value;
	
	#ifdef CONFIG_PM_DEVICE
		/* Do not allow sample fetching from OFF state */
	if (data->pm_device_state == PM_DEVICE_STATE_OFF)
		return -EIO;
	#endif

	int rc = sample_read(dev);

	if (rc < 0) {
		return rc;
	}

	rc = data->hw_tf->read_reg(dev, TMP117_REG_TEMP_RESULT, &value);
	if(rc < 0)
		{
		LOG_ERR("%s, Failed to read temperature.",dev->name);
		return -EINVAL;
	}

	if (value > 0x7FFFU)
		{
		val->raw_sample = (int32_t)(((int32_t) value - 0xFFFF)* TMP117_RESOLUTION) ;
		}
	else
	{
	val->raw_sample = (int32_t)(value * TMP117_RESOLUTION);
	}

	return rc;
}

/**
 * @brief Check the Device ID
 */
static inline int tmp117_device_id_check(const struct device *dev)
{
	struct tmp117_data *data = to_data(dev);
	uint16_t id;

	if (data->hw_tf->read_reg(dev, TMP117_REG_DEVICE_ID, &id) != 0) {
		LOG_ERR("%s: Failed to get Device ID register!", dev->name);
		LOG_ERR("%d\n", id);
		return -EIO;
	}

	if ((id != TMP117_VALUE_ID) && (id != TMP117_VALUE_ID)) {
		LOG_ERR("%s: Failed to match the device IDs!", dev->name);
		return -EINVAL;
	}
	#ifdef CONFIG_PM_DEVICE
		/* Set power state to ACTIVE */
		data->pm_device_state = PM_DEVICE_STATE_ACTIVE;
		
	#endif

	LOG_INF("Got device ID: %x", id);
	return 0;
}

/***************************************/
static int soft_reset_tmp117(const struct device *dev)
{
	struct tmp117_data *data = to_data(dev);
	uint16_t err_code;
	uint16_t reset = TMP117_MASK_RESET & 0xFFFF;

	err_code = data->hw_tf->write_reg(dev, TMP117_REG_CONFIGURATION, reset);
	k_sleep(K_MSEC(TMP117_CC_RESET_DELAY_MS));
	return err_code;
}

/***************************************/
static int sleep_tmp117(const struct device *dev)
{
	struct tmp117_data *data = to_data(dev);
	uint16_t reg_val;
	uint16_t err_code;
	err_code = data->hw_tf->read_reg(dev, TMP117_REG_CONFIGURATION, &reg_val);
	reg_val &= ~TMP117_MASK_MODE;
	reg_val |= TMP117_VALUE_MODE_SLEEP;
	err_code |= data->hw_tf->write_reg(dev, TMP117_REG_CONFIGURATION, reg_val);

	#ifdef CONFIG_PM_DEVICE
		/* Set power state to ACTIVE */
		data->pm_device_state = PM_DEVICE_STATE_LOW_POWER;
	#endif

	return  err_code;
}

/***************************************/
static const struct tmp117_driver_api tmp117_driver_api = {
	.soft_reset		= soft_reset_tmp117,
	.sleep			= sleep_tmp117,
	.get_sample 	= get_sample_tmp117
};


/***************************************/
static int tmp117_init(const struct device *dev)
{
	const struct tmp117_config *cfg = dev->config;
	struct tmp117_data *data = to_data(dev);
	//pm_device_enable(dev);

	data->pm_device_state = PM_DEVICE_STATE_OFF;

	int rc ,err  =0 ;

	err = set_supply_i2c(dev,true);
        if (err!= 0) {
		LOG_INF("GPIO Supply Set Error");
		return -EINVAL;
	}

	/* Bind to the I2C bus that the sensor is connected */
	data->bus = device_get_binding(cfg->bus_name);
	if (!data->bus) {
		LOG_ERR("Cannot bind to %s device!",cfg->bus_name);
		return -EINVAL;
		}

	cfg->bus_init(dev);

	/* Check the Device ID */
	rc = tmp117_device_id_check(dev);
	return rc;

}

/***************************************/
#ifdef CONFIG_PM_DEVICE
int tmp117_pm_ctrl(const struct device *dev, uint32_t ctrl_command,
		enum pm_device_state *state)
{
	struct tmp117_data *data = to_data(dev);
	int ret = 0;

	/* Set power state */
	if (ctrl_command == PM_DEVICE_STATE_SET) {
		if (*state != data->pm_device_state) {
			/* Switching from OFF to any */
				if (data->pm_device_state == PM_DEVICE_STATE_OFF) {
				/* Re-initialize the chip */
				ret = tmp117_init(dev);
			}
			/* Switching to OFF from any */
			else if (tmp117_pm_device_state == PM_DEVICE_STATE_ACTIVE) {
				/* Put the chip into sleep mode */
				ret = sleep_tmp117(dev);
				if (ret < 0)
					LOG_DBG("CTRL_MEAS write failed: %d",
						ret);
				}
			/* Store the new state */
			if (!ret)
				data->pm_device_state = tmp117_pm_device_state;
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

/***************************************/
#ifdef CONFIG_USERSPACE

static inline void z_vrfy_tmp117_get_sample(const struct device *dev, struct tmp117_sample *val)
{
	Z_OOPS(Z_SYSCALL_DRIVER_TMP117(dev, tmp117_get_sample));
	Z_OOPS(Z_SYSCALL_MEMORY_WRITE(val, sizeof(struct tmp117_sample)));
	z_impl_tmp117_get_sample((const struct device *)dev, (struct tmp117_get_sample *)val);
}
#include <syscalls/tmp117_get_sample.c>

/***************************************/
static inline void z_vrfy_tmp117_soft_reset(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_DRIVER_TMP117(dev, tmp117_soft_reset));
	z_impl_tmp117_soft_reset((const struct device *)dev);
}
#include <syscalls/tmp117_soft_reset_mrsh.c>

/***************************************/
static inline void z_vrfy_tmp117_sleep(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_DRIVER_TMP117(dev, sleep));
	z_impl_tmp117_sleep((const struct device *)dev);
}
#include <syscalls/tmp117_sleep_mrsh.c>

#endif /* CONFIG_USERSPACE */

/***************************************/
#define TMP117_DEFINE(inst)
			DEVICE_DT_INST_DEFINE(0,
				tmp117_init,
				tmp117_pm_ctrl,
				&tmp117_data,
				&tmp117_config,
				POST_KERNEL,
				CONFIG_TMP117_INIT_PRIORITY,
				&tmp117_driver_api);
DT_INST_FOREACH_STATUS_OKAY(TMP117_DEFINE)