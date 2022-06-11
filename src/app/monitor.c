
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include "sensor.h"

#include "monitor.h"

LOG_MODULE_REGISTER(monitor);

const struct device *dev;

static monitor_data_t g_tmonitor_data = {0};



int monitor_fet_control(uint8_t ctrl)
{
    struct sensor_trigger trig = {0};
    int status = 0;

    trig.type = SENSOR_TRIG_MONITOR_FET_CONTROL;
    trig.trig_data = ctrl;
    status = sensor_trigger_set(dev, &trig, NULL);
    if (status < 0)
    {
        LOG_ERR("Failed to control monitor fet");
    }

    return status;
}

int monitor_get_fet_status(uint8_t *p_data)
{
    int status = 0;
    uint8_t fet_s = 0;

    status = sensor_channel_get(dev, SENSOR_CHAN_MONITOR_FET_STATUS, (struct sensor_value *)&fet_s);
    if (status < 0)
    {
        LOG_ERR("Failed to get cells voltage");
    }
    *p_data = fet_s;
    LOG_DBG("monitor fet status is %d", fet_s);

    return 0;
}

void monitor_thread(void)
{
    int status = 0;

    dev = device_get_binding(DT_LABEL(DT_INST(0, ti_bq769x2)));
    if (!dev) 
    {
        LOG_ERR("Failed to get monitor binding");
        return;
    }

    while (1)
    {
        // for test
        // static uint8_t test = 0;
        // uint8_t fet_s = 0;

        // monitor_get_fet_status(&fet_s);

        // if (0 == test)
        // {
        //     LOG_DBG("turn on monitor dsg");
        //     monitor_fet_control(0x0e);
        //     test += 1;
        // }
        // else if (1 == test)
        // {
        //     LOG_DBG("turn on monitor chg");
        //     monitor_fet_control(11);
        //     test += 1;
        // }
        // else
        // {
        //     LOG_DBG("turn off monitor fet");
        //     monitor_fet_control(0x0f);
        //     test = 0;
        // }
        // =========================

        // get all voltage and cc1
        status = sensor_channel_get(dev, SENSOR_CHAN_MONITOR_VOLTAGE, (struct sensor_value *)g_tmonitor_data.temp_data);
        if (status < 0)
        {
            LOG_ERR("Failed to get cells voltage");
        }
        else
        {
            // memcpy(g_tmonitor_data.cell_vol, g_tmonitor_data.temp_data, CONFIG_BQ769x2_CONNECTING_CELLS * 2);
            // g_tmonitor_data.bat_vol = g_tmonitor_data.temp_data[CONFIG_BQ769x2_CONNECTING_CELLS * 2] 
            //                             + g_tmonitor_data.temp_data[CONFIG_BQ769x2_CONNECTING_CELLS * 2 + 1] << 8;
            // g_tmonitor_data.pack_vol = g_tmonitor_data.temp_data[CONFIG_BQ769x2_CONNECTING_CELLS * 2 + 2] 
            //                             + g_tmonitor_data.temp_data[CONFIG_BQ769x2_CONNECTING_CELLS * 2 + 3] << 8;

            g_tmonitor_data.cell_vol[0] = (uint16_t)g_tmonitor_data.temp_data[0] + ((uint16_t)g_tmonitor_data.temp_data[1] << 8);
            g_tmonitor_data.bat_vol = (uint16_t)g_tmonitor_data.temp_data[4] + (uint16_t)(g_tmonitor_data.temp_data[5] << 8);
            g_tmonitor_data.pack_vol = (uint16_t)g_tmonitor_data.temp_data[6] + (uint16_t)(g_tmonitor_data.temp_data[7] << 8);

            LOG_DBG("monitor cell voltages is %d", g_tmonitor_data.cell_vol[0]);
            LOG_DBG("monitor bat voltage is %d", g_tmonitor_data.bat_vol);
            LOG_DBG("monitor pack voltage is %d", g_tmonitor_data.pack_vol);

        }

        // get cc2
        status = sensor_channel_get(dev, SENSOR_CHAN_MONITOR_CC2, (struct sensor_value *)g_tmonitor_data.temp_data);
        if (status < 0)
        {
            LOG_ERR("Failed to get cells voltage");
        }
        else
        {
            g_tmonitor_data.cc2 = g_tmonitor_data.temp_data[0] + ((int16_t)g_tmonitor_data.temp_data[1] << 8);
            LOG_DBG("monitor cc2 is %d", g_tmonitor_data.cc1);
        }
        // get cc1
        status = sensor_channel_get(dev, SENSOR_CHAN_MONITOR_CC1, (struct sensor_value *)g_tmonitor_data.temp_data);
        if (status < 0)
        {
            LOG_ERR("Failed to get cells voltage");
        }
        else
        {
            g_tmonitor_data.cc1 = g_tmonitor_data.temp_data[0] + ((int16_t)g_tmonitor_data.temp_data[1] << 8);
            // LOG_DBG("monitor cc1 is %d", g_tmonitor_data.cc1);
        }
        // get inter temp
        status = sensor_channel_get(dev, SENSOR_CHAN_MONITOR_INTER_TS, (struct sensor_value *)g_tmonitor_data.temp_data);
        if (status < 0)
        {
            LOG_ERR("Failed to get inter temp");
        }
        else
        {
            g_tmonitor_data.inter_ts = g_tmonitor_data.temp_data[0] + ((int16_t)g_tmonitor_data.temp_data[1] << 8);
            g_tmonitor_data.inter_ts -= 2731;
            g_tmonitor_data.inter_ts /= 10;
            LOG_DBG("monitor inter temp is %f", g_tmonitor_data.inter_ts);
        }

        k_msleep(125);

    }
}


K_THREAD_DEFINE(monitor_thread_id, 1024, monitor_thread, NULL, NULL, NULL, 7, 0, 0);




