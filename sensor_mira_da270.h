

#ifndef SENSOR_MIRA_DA270_H__
#define SENSOR_MIRA_DA270_H__

#include "sensor.h"
#include "da270.h"

#define DA270_ADDR_DEFAULT UINT8_C(0x27)

int rt_hw_da270_init(const char *name, struct rt_sensor_config *cfg);

#endif
