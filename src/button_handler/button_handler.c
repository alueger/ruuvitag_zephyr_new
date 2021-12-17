/*
 * Copyright (c) 2020 theB@STI0N
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>

#include "ruuvi.h"
#include "button_handler.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(button_handler, CONFIG_RUUVITAG_LOG_LEVEL);

const struct device *button;

/* The devicetree node identifier for the "sw0" alias. */
#define SW0_NODE	DT_ALIAS(sw0)

#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
#define SW0_GPIO_LABEL	DT_GPIO_LABEL(SW0_NODE, gpios)
#define SW0_GPIO_PIN	DT_GPIO_PIN(SW0_NODE, gpios)
#define SW0_GPIO_FLAGS	(GPIO_INPUT | DT_GPIO_FLAGS(SW0_NODE, gpios))
#else
#error "Unsupported board: sw0 devicetree alias is not defined"
#define SW0_GPIO_LABEL	""
#define SW0_GPIO_PIN	0
#define SW0_GPIO_FLAGS	0
#endif

void button_int_setup(struct gpio_callback *handle, gpio_callback_handler_t cbh){
    int ret = gpio_pin_interrupt_configure(button,
					   SW0_GPIO_PIN,
					   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
		return;
	}

	gpio_init_callback(handle, cbh, BIT(SW0_GPIO_PIN));
	gpio_add_callback(button, handle);
    return;
}

bool button_pressed_state(void){
    int pressed = gpio_pin_get_raw(button, SW0_GPIO_PIN);
    if(pressed){
        return false;
    }
    else{
        return true;
    }
}

void button_init(void)
{
	button = device_get_binding(SW0_GPIO_LABEL);
	if (button == NULL) {
		LOG_ERR("Error: didn't find %s device\n", SW0_GPIO_LABEL);
		return;
	}

	int ret = gpio_pin_configure(button, SW0_GPIO_PIN, SW0_GPIO_FLAGS);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d\n",
		       ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
		return;
	}
}
