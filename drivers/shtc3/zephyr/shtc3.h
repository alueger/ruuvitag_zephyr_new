/*
 * Copyright (c) 2021 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SHTC3_SHTC3_H_
#define ZEPHYR_DRIVERS_SENSOR_SHTC3_SHTC3_H_

#include <device.h>
#include <devicetree.h>
#include <kernel.h>
#include <drivers/gpio.h>

/* common cmds */
#define SHTC3_CMD_READ_ID		0xEFC8
#define SHTC3_CMD_SOFT_RESET		0x805D
/* shtc3 only: */
#define SHTC3_CMD_SLEEP			0xB098
#define SHTC3_CMD_WAKEUP		0x3517

#define SHTC3_POWER_UP_TIME_US		240U
/* Soft reset time is 230us for shtc1 and 240us for shtc3 */
#define SHTC3_SOFT_RESET_TIME_US	240U //

#define SHTC3_MAX_READ_LEN		6
#define SHTC3_WORD_LEN			2
#define SHTC3_CRC8_LEN			1

#define SHTC3_ID_MASK			0x083F
#define SHTC3_ID_VALUE			0x0807
#define SHTC1_ID_MASK			0x083F
#define SHTC1_ID_VALUE			0x0007

/* defines matching the related enums DT_ENUM_IDX: */
#define CHIP_SHTC1		0
#define CHIP_SHTC3		1
#define MEASURE_MODE_NORMAL	0
#define MEASURE_MODE_LOW_POWER	1

enum shtc3_chip {
	SHTC1 = CHIP_SHTC1,
	SHTC3 = CHIP_SHTC3,
};

enum shtc3_measure_mode {
	NORMAL = MEASURE_MODE_NORMAL,
	LOW_POWER = MEASURE_MODE_LOW_POWER,
};

struct shtc3_sample {
	uint16_t temp;
	uint16_t humidity;
} __packed __aligned(2);

struct shtc3_config {
	const struct device *bus;
	uint8_t base_address;
	enum shtc3_chip chip;
	enum shtc3_measure_mode measure_mode;
	bool clock_stretching;
};

struct shtc3_data {
	const struct device *i2c;
        struct shtc3_sample sample;
        
        #if DT_INST_NODE_HAS_PROP(0, supplyi2c_gpios)
	const struct device *supplyi2c_gpio;
        #endif
	
        #ifdef CONFIG_PM_DEVICE
		enum pm_device_state pm_device_state; /* Current power state */
	#endif 
};

static inline uint8_t shtc3_i2c_address(const struct device *dev)
{
	const struct shtc3_config *dcp = dev->config;

	return dcp->base_address;
}

static inline const struct device *shtc3_i2c_bus(const struct device *dev)
{
	const struct shtc3_config *dcp = dev->config;

	return dcp->bus;
}

#endif /* ZEPHYR_DRIVERS_SENSOR_SHTC3_SHTC3_H_ */
