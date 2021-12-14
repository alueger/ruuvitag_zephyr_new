#ifndef ZEPHYR_DRIVERS_SENSOR_TMP117_TMP117_H_
#define ZEPHYR_DRIVERS_SENSOR_TMP117_TMP117_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#include <kernel.h>
#include <device.h>
#include <devicetree.h>

#include <sys/util.h>
#include <stdint.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <string.h>


struct tmp117_data {
	const struct device *bus;
	const struct device *i2c;
        const struct tmp117_transfer_function *hw_tf;
        uint8_t sample_rate;
	uint8_t resolution;
	uint8_t scale;
	uint8_t dsp;
	uint8_t mode;

        #if DT_INST_NODE_HAS_PROP(0, supplyi2c_gpios)
	const struct device *supplyi2c_gpio;
        #endif
	
        #ifdef CONFIG_PM_DEVICE
		enum pm_device_state pm_device_state; /* Current power state */
	#endif 
};


struct tmp117_sample {
	int32_t raw_sample;
};

union tmp117_bus_cfg {
	uint16_t i2c_slv_addr;
};

struct tmp117_config {
	const char *bus_name;
	int (*bus_init)(const struct device *dev);
	const union tmp117_bus_cfg bus_cfg;
};

struct tmp117_transfer_function {
	int (*read_reg)(const struct device *dev, uint8_t reg,
	uint16_t *val);
	int (*write_reg)(const struct device *dev, uint8_t reg,
	uint16_t val);
};


int tmp117_i2c_init(const struct device *dev);

struct tmp117_driver_api {
	int (*soft_reset)(const struct device *dev);
	int (*sleep)(const struct device *dev);
	int (*get_sample)(const struct device *dev, struct tmp117_sample *val);
};


static int tmp117_soft_reset(const struct device *dev)
{
	const struct tmp117_driver_api *api =
		(const struct tmp117_driver_api *)dev->api;

	return api->soft_reset(dev);
}


static inline int tmp117_sleep(const struct device *dev)
{
	const struct tmp117_driver_api *api =
		(const struct tmp117_driver_api *)dev->api;

	return api->sleep(dev);
}


static  int tmp117_get_sample(const struct device *dev,
					    						struct tmp117_sample *val)
{
	const struct tmp117_driver_api *api =
		(const struct tmp117_driver_api *)dev->api;

	return api->get_sample(dev, val);
}


#ifdef __cplusplus
}
#endif

//#include <syscalls/tmp117.h>

#endif /* ZEPHYR_DRIVERS_SENSOR_TMP117_TMP117_H_*/