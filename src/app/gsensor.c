/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
// #include <drivers/sensor.h>
#include "sensor.h"
#include "gsensor.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(gsensor);


static const struct device *lis2dw12;

static void fetch_and_display(void)
{
	static unsigned int count;
	struct sensor_value accel[3];
	// struct sensor_value temperature;
	const char *overrun = "";
	int rc = sensor_sample_fetch(lis2dw12);

	++count;
	if (rc == -EBADMSG) {
		/* Sample overrun.  Ignore in polled mode. */
		if (IS_ENABLED(CONFIG_LIS2DW12XD_TRIGGER)) {
			overrun = "[OVERRUN] ";
		}
		rc = 0;
	}
	if (rc == 0) {
		rc = sensor_channel_get(lis2dw12,
					SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc < 0) {
		LOG_ERR("ERROR: Update failed: %d\n", rc);
	} else {
		LOG_DBG("#%u @ %u ms: %sx %f , y %f , z %f",
		       count, k_uptime_get_32(), overrun,
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]));
	}

	// if (IS_ENABLED(CONFIG_LIS2DH_MEASURE_TEMPERATURE)) {
	// 	if (rc == 0) {
	// 		rc = sensor_channel_get(sensor, SENSOR_CHAN_DIE_TEMP, &temperature);
	// 		if (rc < 0) {
	// 			printf("\nERROR: Unable to read temperature:%d\n", rc);
	// 		} else {
	// 			printf(", t %f\n", sensor_value_to_double(&temperature));
	// 		}
	// 	}

	// } else {
	// 	printf("\n");
	// }
}

#ifdef CONFIG_LIS2DH_TRIGGER
static void trigger_handler(const struct device *dev,
			    const struct sensor_trigger *trig)
{
	fetch_and_display(dev);
}
#endif

void gsensor_thread(void)
{
	// lis2dw12 = DEVICE_DT_GET_ANY(st_lis2dw12xd);
	lis2dw12 = device_get_binding(DT_LABEL(DT_INST(0, st_lis2dw12xd)));

	if (lis2dw12 == NULL) 
	{
		LOG_ERR("No device found\n");
		return;
	}
	if (!device_is_ready(lis2dw12)) 
	{
		LOG_ERR("Device %s is not ready\n", lis2dw12->name);
		return;
	}

// #if CONFIG_LIS2DH_TRIGGER
// 	{
// 		struct sensor_trigger trig;
// 		int rc;

// 		trig.type = SENSOR_TRIG_DATA_READY;
// 		trig.chan = SENSOR_CHAN_ACCEL_XYZ;

// 		if (IS_ENABLED(CONFIG_LIS2DH_ODR_RUNTIME)) {
// 			struct sensor_value odr = {
// 				.val1 = 1,
// 			};

// 			rc = sensor_attr_set(sensor, trig.chan,
// 					     SENSOR_ATTR_SAMPLING_FREQUENCY,
// 					     &odr);
// 			if (rc != 0) {
// 				printf("Failed to set odr: %d\n", rc);
// 				return;
// 			}
// 			printf("Sampling at %u Hz\n", odr.val1);
// 		}

// 		rc = sensor_trigger_set(sensor, &trig, trigger_handler);
// 		if (rc != 0) {
// 			printf("Failed to set trigger: %d\n", rc);
// 			return;
// 		}

// 		printf("Waiting for triggers\n");
// 		while (true) {
// 			k_sleep(K_MSEC(2000));
// 		}
// 	}
	// printf("Polling at 0.5 Hz\n");
	for (;;) 
	{
		fetch_and_display();
		k_sleep(K_MSEC(1000));
	}
}

K_THREAD_DEFINE(gsensor_thread_id, 1024, gsensor_thread, NULL, NULL, NULL, 7, 0, 0);

