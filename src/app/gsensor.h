#ifndef GSENSOR_H
#define GSENSOR_H

#include "sensor.h"

extern int gsensor_set_attr(enum sensor_channel chan, enum sensor_attribute attr, uint32_t value);
extern int gsensor_get_attr(enum sensor_channel chan, enum sensor_attribute attr, void *value);
extern int gsensor_chanl_get(struct sensor_axis_val *value);



#endif
