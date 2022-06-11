/*

//                                     /|\ /|\ 
//                   nrf52840           5k |
//                 -----------------    |  5k
//                |             PB8 |---+---|-- I2C Clock (SCL)
//                |                 |       |
//                |		        PB9 |-------+-- I2C Data (SDA)
//                |                 |
//     DFETOFF ---| PA8             |
//                |                 |
//   RST_SHUT  ---| PA9        		|--- Green LED
//                |                 |
//      ALERT  ---|	PA10            |
//                |                 |


*/



/*
 * Copyright (c) 2020 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_bq769x2

#include <drivers/i2c.h>
#include <init.h>
// #include <drivers/sensor.h>
#include "sensor.h"
#include <sys/__assert.h>
#include <string.h>
#include <sys/byteorder.h>
#include <logging/log.h>

#include "bq769x2.h"

LOG_MODULE_REGISTER(bq769x2);

static uint8_t bq769x2_checksum(unsigned char *ptr, unsigned char len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.	
{
	uint8_t i;
	uint8_t checksum = 0;

	for(i=0; i<len; i++)
		checksum += ptr[i];

	checksum = 0xff & ~checksum;

	return(checksum);
}

#ifdef BQ769x2_I2C_CRC_MODE
static uint8_t bq769x2_crc8(unsigned char *ptr, unsigned char len)
//Calculates CRC8 for passed bytes. Used in i2c read and write functions 
{
	uint8_t i;
	uint8_t crc=0;
	while(len--!=0)
	{
		for(i=0x80; i!=0; i/=2)
		{
			if((crc & 0x80) != 0)
			{
				crc *= 2;
				crc ^= 0x107;
			}
			else
				crc *= 2;

			if((*ptr & i)!=0)
				crc ^= 0x107;
		}
		ptr++;
	}
	return(crc);
}
#endif

// wtite one time
static int bq769x2_pack_write(const struct device *dev, uint8_t *p_data, uint8_t count)
{
#ifdef BQ769x2_I2C_CRC_MODE
	uint8_t s_data[CONFIG_BQ769x2_MAXBUF_SIZE] = {0};
	uint8_t crc_count = 0;
	crc_count = count * 2;
	uint8_t crc1stByteBuffer[3] = {0x10, p_data[0], p_data[1]};
	unsigned int j;
	unsigned int i;
	uint8_t temp_crc_buffer[3] = {0};

	s_data[0] = p_data[0];
	s_data[1] = bq769x2_crc8(crc1stByteBuffer,3);

	j = 2;
	for(i=1; i < count; i++)
	{
		s_data[j] = reg_data[i];
		j = j + 1;
		temp_crc_buffer[0] = reg_data[i];
		s_data[j] = bq769x2_crc8(temp_crc_buffer,1);
		j = j + 1;
	}

	return i2c_write(dev, p_data, count, DT_INST_REG_ADDR(0));
#else
	return i2c_write(dev, p_data, count, DT_INST_REG_ADDR(0));
#endif
}

static int bq769x2_pack_read(const struct device *dev, uint8_t start_adr, uint8_t *p_data, uint8_t count)
{	
#ifdef BQ769x2_I2C_CRC_MODE
	unsigned int RX_CRC_Fail = 0;  // reset to 0. If in CRC Mode and CRC fails, this will be incremented.
	uint8_t rr_buf[CONFIG_BQ769x2_MAXBUF_SIZE] = {0};
	uint8_t crc_count = 0;
	uint8_t r_data[CONFIG_BQ769x2_MAXBUF_SIZE] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	crc_count = count * 2;
	unsigned int j;
	unsigned int i;
	unsigned char CRCc = 0;
	uint8_t temp_crc_buffer[3];

	// HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, reg_addr, 1, ReceiveBuffer, crc_count, 1000);
	i2c_burst_read(dev, DT_INST_REG_ADDR(0), start_adr, r_data, crc_count);

	uint8_t crc1stByteBuffer [4] = {0x10, start_adr, 0x11, r_data[0]};
	CRCc = bq769x2_crc8(crc1stByteBuffer,4);
	if (CRCc != r_data[1])
	{
		// RX_CRC_Fail += 1;
		LOG_ERR(" bq769x2 crc error");
		return -EBUSY;
	}
	rr_buf[0] = r_data[0];

	j = 2;
	for (i=1; i<count; i++)
	{
		rr_buf[i] = r_data[j];
		temp_crc_buffer[0] = r_data[j];
		j = j + 1;
		CRCc = bq769x2_crc8(temp_crc_buffer,1);
		if (CRCc != r_data[j])
		{
			// RX_CRC_Fail += 1;
			LOG_ERR(" bq769x2 crc error");
			return -EBUSY;
		}
		j = j + 1;
	}
	memcpy(rr_buf, reg_data, count);
#else
	// int status;
	// uint8_t i2c_data[2];
	// uint16_t data = 0;

	return i2c_burst_read(dev, DT_INST_REG_ADDR(0), start_adr, p_data, count);
	// status = i2c_burst_read(dev, DT_INST_REG_ADDR(0), 0x14, i2c_data, count);
	// if (status < 0) 
	// {
	// 	LOG_ERR("Unable to read register");
	// 	return -EIO;
	// }
	// data = (i2c_data[1] << 8) | i2c_data[0];
	// LOG_DBG(" bq769x2 vol is %d", data);

	// return 0;

#endif
}

// exclude CGain and Capacity Gain;byte or word
static int bq769x2_set_register(struct bq769x2_data *bq769x2, uint16_t start_adr, uint16_t data, uint8_t count)
{
	uint8_t s_data[5] = {0};
	uint8_t c_data[5] = {0};
	int status = 0;

	s_data[0] = BQ769X2_COMM_CONTROL_LWO;
	s_data[1] = start_adr & 0x00ff;
	s_data[2] = (start_adr & 0xff00) >> 8;
	s_data[3] = data & 0x00ff;
	if (count > 1)
	{
		s_data[4] = (data & 0xff00) >> 8;
	}

	status = bq769x2_pack_write(bq769x2->i2c, s_data, 3 + count);
	if (status < 0)
	{
		return -EBUSY;
	}

	k_msleep(1);

	c_data[0] = 0x60;
	c_data[1] = bq769x2_checksum(&s_data[1], 2 + count);
	c_data[2] = 2 + 2 + count; //combined length of registers address and data
	status = bq769x2_pack_write(bq769x2->i2c, c_data, 3);
	if (status < 0)
	{
		return -EBUSY;
	}
	k_msleep(2);

	return 0;
}

//For Command only Subcommands
static int bq769x2_command_subcommands(struct bq769x2_data *bq769x2, uint16_t command) 
{
	uint8_t s_data[3] = {0};
	int status = 0;

	s_data[0] = BQ769X2_COMM_CONTROL_LWO;
	s_data[1] = command & 0x00ff;
	s_data[2] = (command & 0xff00) >> 8;
	status = bq769x2_pack_write(bq769x2->i2c, s_data, 3);
	if (status < 0)
	{
		return -EBUSY;
	}

	return 0;
}

static int bq769x2_subcommands_read(struct bq769x2_data *bq769x2, uint16_t command, uint8_t *p_data, uint8_t off_set, uint8_t count)
{
	uint8_t s_data[3] = {0};
	int status = 0;

	s_data[0] = BQ769X2_COMM_CONTROL_LWO;
	s_data[1] = command & 0x00ff;
	s_data[2] = (command & 0xff00) >> 8;
	status = bq769x2_pack_write(bq769x2->i2c, s_data, 3);
	if (status < 0)
	{
		return -EBUSY;
	}

	k_msleep(2);

	s_data[0] = 0x40;
	status = bq769x2_pack_read(bq769x2->i2c, 0x40 + off_set, p_data, count);
	if (status < 0)
	{
		return -EBUSY;
	}

	return 0;
}

// count <= 2;byte or word
static int bq769x2_subcommands_write(struct bq769x2_data *bq769x2, uint16_t command, uint16_t data, uint8_t count)
{
	uint8_t s_data[5] = {0};
	uint8_t c_data[5] = {0};
	int status = 0;

	s_data[0] = BQ769X2_COMM_CONTROL_LWO;
	s_data[1] = command & 0x00ff;
	s_data[2] = (command & 0xff00) >> 8;
	s_data[3] = data & 0x00ff;
	if (count > 1)
	{
		s_data[4] = (data & 0xff0) >> 8;
	}

	status = bq769x2_pack_write(bq769x2->i2c, s_data, 3 + count);
	if (status < 0)
	{
		return -EBUSY;
	}

	k_msleep(1);

	c_data[0] = 0x60;
	c_data[1] = bq769x2_checksum(&s_data[1], 2 + count);
	c_data[2] = 2 + 2 + count; //combined length of registers address and data
	status = bq769x2_pack_write(bq769x2->i2c, c_data, 3);
	if (status < 0)
	{
		return -EBUSY;
	}

	return 0;
}

static int bq769x2_direct_command_read(struct bq769x2_data *bq769x2, uint16_t start_adr, uint8_t *p_data, uint8_t count)
{
	// write-read
	// read 1 or 2 bytes
	// uint8_t i2c_data[2];
	// int status;

	if (NULL == p_data)
	{
		return -EINVAL;
	}

	return bq769x2_pack_read(bq769x2->i2c, start_adr, p_data, count);
}


// count <= 2;byte or word
static int bq769x2_direct_command_write(struct bq769x2_data *bq769x2, uint8_t start_adr, uint16_t data, uint8_t count)
{
	// write
	// write 2 bytes
	uint8_t s_data[3] = {0};
	// int status = 0;

	s_data[0] = start_adr;
	s_data[1] = data & 0x00ff;
	if (count > 1)
	{
		s_data[2] = (data & 0xff00) >> 8;
	}

	return bq769x2_pack_write(bq769x2->i2c, s_data, 2 + count);
}

/**
 * @brief sensor value get
 *
 * @return -ENOTSUP for unsupported channels
 */
static int bq769x2_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	struct bq769x2_data *bq769x2 = dev->data;
	// float int_temp;
	int status = 0;

	switch (chan) 
	{
		case SENSOR_CHAN_MONITOR_VOLTAGE:
			// status = bq769x2_direct_command_read(bq769x2, BQ76952_COMMAND_CELL1VOLTAGE, (uint8_t *)val, CONFIG_BQ769x2_CONNECTING_CELLS * 2);
			status = bq769x2_direct_command_read(bq769x2, BQ76952_COMMAND_CELL1VOLTAGE, (uint8_t *)val, 4);
			// LOG_DBG("cell 1 vol is %d", val->val1);
			status = bq769x2_direct_command_read(bq769x2, BQ76952_COMMAND_STACKVOLTAGE, (uint8_t *)(&(val->val2)), 4);
			return status;
		case SENSOR_CHAN_MONITOR_CC2:
			return bq769x2_direct_command_read(bq769x2, BQ76952_COMMAND_CC2CURRENT, (uint8_t *)val, 2);
		case SENSOR_CHAN_MONITOR_CC1:
			return bq769x2_subcommands_read(bq769x2, BQ76952_SUBCOMMAND_DASTATUS5, (uint8_t *)val, 0x16, 2);
		case SENSOR_CHAN_MONITOR_INTER_TS:
			return bq769x2_direct_command_read(bq769x2, BQ76952_COMMAND_INTTEMPERATURE, (uint8_t *)val, 2);
		case SENSOR_CHAN_MONITOR_FET_STATUS:
			status = bq769x2_direct_command_read(bq769x2, BQ76952_COMMAND_FETSTATUS, (uint8_t *)val, 1);
			if (status >= 0)
			{
				val->val1 &= 0x0f;
			}
			return status;
		default:
			return -ENOTSUP;
	}

	return 0;
}

static int  bq769x2_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler)
{
	struct bq769x2_data *bq769x2 = dev->data;
	// int status = 0;

	switch (trig->type)
	{
		case SENSOR_TRIG_MONITOR_FET_CONTROL:
			return bq769x2_subcommands_write(bq769x2, BQ76952_SUBCOMMAND_FET_CONTROL, trig->trig_data & 0x0000000f, 1);
		default:
			return -ENOTSUP;
	}

	return 0;
}

// static int bq769x2_sample_fetch(const struct device *dev, enum sensor_channel chan)
// {
// 	struct bq769x2_data *bq769x2 = dev->data;
// 	int status = 0;

// 	switch (chan) 
//     {
// 	case SENSOR_CHAN_MONITOR_VOLTAGE:
		
// 		break;

// 	default:
// 		return -ENOTSUP;
// 	}

// 	return 0;
// }

static int bq769x2_monitor_configure(const struct device *dev)
{
	struct bq769x2_data *bq769x2 = dev->data;
	// const struct bq769x2_config *const config = dev->config;
	// int status = 0;

	bq769x2_command_subcommands(bq769x2, BQ76952_SUBCOMMAND_SET_CFGUPDATE);
	// power config
	bq769x2_set_register(bq769x2, BQ76952_DMR_POWERCONFIG, 0x2D80, 2);

	// 'REG0 Config' - set REG0_EN bit to enable pre-regulator
	bq769x2_set_register(bq769x2, BQ76952_DMR_REG0CONFIG, 0x01, 1);

	// 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
	bq769x2_set_register(bq769x2, BQ76952_DMR_REG12CONFIG, 0x0D, 1);

	bq769x2_set_register(bq769x2, BQ76952_DMR_FETOPTIONS, 0x2f, 1);

	// Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
	// bq769x2_set_register(bq769x2, BQ76952_DMR_DFETOFFPINCONFIG, 0x42, 1);

	// Set up ALERT Pin - 0x92FC = 0x2A
	// This configures the ALERT pin to drive high (REG1 voltage) when enabled.
	// The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
	bq769x2_set_register(bq769x2, BQ76952_DMR_ALERTPINCONFIG, 0x2A, 1);

	// Set TS1 to measure Cell Temperature - 0x92FD = 0x07
	bq769x2_set_register(bq769x2, BQ76952_DMR_TS1CONFIG, 0x07, 1);

	// Set TS3 to measure FET Temperature - 0x92FF = 0x0F
	bq769x2_set_register(bq769x2, BQ76952_DMR_TS3CONFIG, 0x0F, 1);

	// // Set HDQ to measure Cell Temperature - 0x9300 = 0x07
	// BQ769x2_SetRegister(HDQPinConfig, 0x00, 1);  // No thermistor installed on EVM HDQ pin, so set to 0x00

	// // 'VCell Mode' - Enable 16 cells - 0x9304 = 0x0000; Writing 0x0000 sets the default of 16 cells
	// BQ769x2_SetRegister(VCellMode, 0x0000, 2);


	bq769x2_command_subcommands(bq769x2, BQ76952_SUBCOMMAND_EXIT_CFGUPDATE);

	return 0;
}

/**
 * @brief initialise the monitor
 *
 * @return 0 for success
 */
static int bq769x2_monitor_init(const struct device *dev)
{
	struct bq769x2_data *bq769x2 = dev->data;
	const struct bq769x2_config *const config = dev->config;
	int status = 0;
	uint16_t id;

	bq769x2->i2c = device_get_binding(config->bus_name);
	if (bq769x2->i2c == NULL) 
    {
		LOG_ERR("Could not get pointer to %s device.", config->bus_name);
		return -EINVAL;
	}

	// uint8_t test[2] = {0};
	// bq769x2_direct_command_read(bq769x2, BQ76952_COMMAND_CELL1VOLTAGE, test, 2);

	// bq769x2_direct_command_write(bq769x2, BQ76952_COMMAND_ALARMENABLE, 0xf082, 2);

	// bq769x2_subcommands_read(bq769x2, BQ76952_SUBCOMMAND_DEVICE_NUMBER, test, 2);
	// bq769x2_set_register(bq769x2, )
	// bq769x2_command_subcommands(bq769x2, BQ76952_SUBCOMMAND_FET_ENABLE);
	// k_msleep(500);
	// test[0] = 0x0e;
	// bq769x2_subcommands_write(bq769x2, BQ76952_SUBCOMMAND_FET_CONTROL, test, 1);
	// k_msleep(500);

	// bq769x2_direct_command_read(bq769x2, BQ76952_COMMAND_FETSTATUS, test, 1);
	
	// status = bq769x2_get_device_type(bq769x2, &id);
	status = bq769x2_subcommands_read(bq769x2, BQ76952_SUBCOMMAND_DEVICE_NUMBER, (uint8_t *)&id, 0, 2);
	if (status < 0) 
	{
		LOG_ERR("Unable to get bq769x2 ID");
		return -EIO;
	}
	if (id != BQ769X2_DEVICE_NUMBER) 
	{
		LOG_ERR("Invalid bq769x2 Number is %d", id);
		return -EINVAL;
	}
	// config gpio

	// config monitor 
	status = bq769x2_monitor_configure(dev);

	return status;
}

static const struct sensor_driver_api bq769x2_battery_driver_api = {
	// .sample_fetch = bq769x2_sample_fetch,
	.channel_get = bq769x2_channel_get,
	.trigger_set = bq769x2_trigger_set,
};

#define BQ769x2_INIT(index)                                                    \
	static struct bq769x2_data bq769x2_driver_##index;                     \
									       \
	static const struct bq769x2_config bq769x2_config_##index = {          \
		.bus_name = DT_INST_BUS_LABEL(index),                          \
	};                                                                     \
									       \
	DEVICE_DT_INST_DEFINE(index, &bq769x2_monitor_init, NULL,                \
			    &bq769x2_driver_##index,                           \
			    &bq769x2_config_##index, POST_KERNEL,              \
			    CONFIG_SENSOR_INIT_PRIORITY,                       \
			    &bq769x2_battery_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BQ769x2_INIT)
