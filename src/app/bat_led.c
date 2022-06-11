/*
 * @Author: your name
 * @Date: 2021-08-13 10:35:53
 * @LastEditTime: 2021-08-25 10:43:33
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \v1.6.0\zephyr\samples\hello_world\src\blinky.c
 */
/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <logging/log.h>
// #include <sys/printk.h>
// #include "app_parm_conf.h"


LOG_MODULE_REGISTER(led);

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)



#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN_LED0	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS_LED0	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN_LED0	0
#define FLAGS_LED0	0
#endif

#if DT_NODE_HAS_STATUS(LED1_NODE, okay)
#define LED1	DT_GPIO_LABEL(LED1_NODE, gpios)
#define PIN_LED1	DT_GPIO_PIN(LED1_NODE, gpios)
#define FLAGS_LED1	DT_GPIO_FLAGS(LED1_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led1 devicetree alias is not defined"
#define LED1	""
#define PIN_LED1	0
#define FLAGS_LED1	0
#endif

void led_thread(void)
{
	const struct device *dev_led0;
	const struct device *dev_led1;
	bool led0_is_on = true;
	bool led1_is_on = false;
	int status;

	dev_led0 = device_get_binding(LED0);
	if (dev_led0 == NULL) 
	{
		LOG_ERR("Failed to get LED0-device binding");
		return;
	}

	dev_led1 = device_get_binding(LED1);
	if (dev_led1 == NULL) 
	{
		LOG_ERR("Failed to get LED1-device binding");
		return;
	}

	status = gpio_pin_configure(dev_led0, PIN_LED0, GPIO_OUTPUT_ACTIVE | FLAGS_LED0);
	if (status < 0) 
	{
		LOG_ERR("Failed to config LED0");
		return;
	}

	status = gpio_pin_configure(dev_led1, PIN_LED1, GPIO_OUTPUT_ACTIVE | FLAGS_LED1);
	if (status < 0) 
	{
		LOG_ERR("Failed to config LED1");
		return;
	}

	while (1) 
	{
		gpio_pin_set(dev_led0, PIN_LED0, (int)led0_is_on);
		gpio_pin_set(dev_led1, PIN_LED1, (int)led1_is_on);
		led0_is_on = !led0_is_on;
		led1_is_on = !led1_is_on;
		k_msleep(500);
	}
}

K_THREAD_DEFINE(led_thread_id, 1024, led_thread, NULL, NULL, NULL, 7, 0, 0);
