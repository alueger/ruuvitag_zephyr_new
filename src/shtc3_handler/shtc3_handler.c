/*
 * Copyright (c) 2021 A.Lueger
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include "shtc3_handler.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(shtc3_handler, CONFIG_RUUVITAG_LOG_LEVEL);

#define SHTC3 DT_INST(0, sensirion_shtc3)

#if DT_NODE_HAS_STATUS(SHTC3, okay)
#define SHTC3_LABEL DT_LABEL(SHTC3)
#else
#error Your devicetree has no enabled nodes with compatible "bosch,shtc3"
#define SHTC3_LABEL "<none>"
#endif

// Required to access raw data without needing another driver.
struct shtc3_data {
	uint16_t temp;
	uint16_t humidity;
} __packed __aligned(2);


const struct device *shtc3;

void shtc3_fetch(void)
{
	sensor_sample_fetch(shtc3);
	return;
}

int32_t shtc3_get_temp(void){
	struct shtc3_data *data = shtc3->data;
	return data->temp;
}


uint32_t shtc3_get_humidity(void){
	struct shtc3_data *data = shtc3->data;
	return data->humidity;
}

bool init_shtc3(void){
	shtc3 = device_get_binding(DT_LABEL(DT_INST(0, sensirion_shtc3)));
	if (shtc3 == NULL) {
		return false;
	} else {
		return true;
	}
}
