#ifndef MONITOR_H
#define MONITOR_H


typedef struct 
{
    uint8_t temp_data[50];
    uint16_t cell_vol[CONFIG_BQ769x2_CONNECTING_CELLS];
    uint16_t bat_vol;
    uint16_t pack_vol;
    int16_t cc1;
    int32_t cc2;
    float inter_ts;
}monitor_data_t;




#endif
