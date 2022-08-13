/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
// #include "uart_async_adapter.h"

#include <zephyr/types.h>
#include <zephyr.h>
#include <drivers/uart.h>
// #include <usb/usb_device.h>

#include <device.h>
#include <soc.h>

// #include <bluetooth/bluetooth.h>
// #include <bluetooth/uuid.h>
// #include <bluetooth/gatt.h>
// #include <bluetooth/hci.h>

// #include <bluetooth/services/nus.h>

// #include <dk_buttons_and_leds.h>

// #include <settings/settings.h>

#include <stdio.h>

#include <logging/log.h>

#include "comm.h"
#include "nucleo_main.h"

// #define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(comm);

#define STACKSIZE 4096
#define PRIORITY 7

// #define DEVICE_NAME CONFIG_BT_DEVICE_NAME
// #define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

// #define RUN_STATUS_LED DK_LED1
// #define RUN_LED_BLINK_INTERVAL 1000

// #define CON_STATUS_LED DK_LED2

// #define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
// #define KEY_PASSKEY_REJECT DK_BTN2_MSK



// static K_SEM_DEFINE(ble_init_ok, 0, 1);

// static struct bt_conn *current_conn;
// static struct bt_conn *auth_conn;

static const struct device *uart;
static struct k_work_delayable uart_work;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
// static K_FIFO_DEFINE(fifo_uart_rx_data);

uint16_t uart_rx_offset = 0;
struct k_fifo fifo_uart_rx_data;

// static const struct bt_data ad[] = {
// 	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
// 	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
// };

// static const struct bt_data sd[] = {
// 	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
// };

// #if CONFIG_BT_NUS_UART_ASYNC_ADAPTER
// // UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
// // #else
// static const struct device *const async_adapter;
// #endif

void comm_uart_senddata(uint8_t *data, uint16_t len)
{
    uart_tx(uart, data, len, SYS_FOREVER_MS);
}

void comm_uart_sendmsg(TMsg *msg)
{
    uint16_t count_out;

    CHK_ComputeAndAdd(msg);
    /* MISRA C-2012 rule 11.8 violation for purpose */
    count_out = (uint16_t)ByteStuffCopy((uint8_t *)UartTxBuffer, msg);
	LOG_HEXDUMP_DBG(UartTxBuffer, count_out, "uart tx");
    uart_tx(uart, UartTxBuffer, count_out, SYS_FOREVER_MS);
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static uint8_t *current_buf;
	static size_t aborted_len;
	static bool buf_release;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	uint16_t len1 = 0;
    uint16_t len2 = 0;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("tx_done");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
					   data);
		}

		// k_free(buf);

		// buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		// if (!buf) {
		// 	return;
		// }

		// if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
		// 	LOG_WRN("Failed to send data over UART");
		// }

		break;

	case UART_RX_RDY:
		// struct uart_data_t *t_buf;
		LOG_DBG("rx_rdy");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		
		// buf->len += evt->data.rx.len;
		// t_buf->data = &buf[evt->data.rx.offset];
		// t_buf->len = evt->data.rx.len;
		// buf_release = false;

		// if (buf->len == UART_BUF_SIZE) 
		// {
		// 	// k_fifo_put(&fifo_uart_rx_data, buf);
		// 	LOG_HEXDUMP_DBG(buf->data, buf->len, "uart rx");
		// } 
		// else if (evt->data.rx.buf[buf->len - 1] == 0xf0)
		// {
		if ((uart_rx_offset + evt->data.rx.len) > UART_RxBufferSize)
		{
			len1 = UART_RxBufferSize - uart_rx_offset;
			memcpy(&UartRxBuffer[uart_rx_offset], buf->data[evt->data.rx.offset], len1);
			uart_rx_offset = 0;
			len2 = evt->data.rx.len - len1;
			memcpy(&UartRxBuffer[uart_rx_offset], &(buf->data[evt->data.rx.offset + len1]), len2);
			uart_rx_offset += len2;
		}
		else
		{
			memcpy(&UartRxBuffer[uart_rx_offset], buf->data[evt->data.rx.offset], evt->data.rx.len);
			uart_rx_offset += evt->data.rx.len;
		}

		LOG_DBG("addr of buf is %d, evt data len is %d", buf, evt->data.rx.len);
		LOG_HEXDUMP_DBG(buf->data, buf->len, "uart rx");

        // k_fifo_put(&fifo_uart_rx_data, t_buf);
        current_buf = evt->data.rx.buf;
        buf_release = true;
        uart_rx_disable(uart);
		// }

		

		break;

	case UART_RX_DISABLED:
		LOG_DBG("rx_disabled");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("rx_buf_request");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("rx_buf_released");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data);
		if (buf_release && (current_buf != evt->data.rx_buf.buf)) {
			k_free(buf);
			buf_release = false;
			current_buf = NULL;
		}

		break;

	case UART_TX_ABORTED:
			LOG_DBG("tx_aborted");
			if (!aborted_buf) {
				aborted_buf = (uint8_t *)evt->data.tx.buf;
			}

			aborted_len += evt->data.tx.len;
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data);

			uart_tx(uart, &buf->data[aborted_len],
				buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	// struct uart_data_t *tx;

	uart = device_get_binding(DT_LABEL(DT_NODELABEL(uart1)));
	if (!uart) {
		return -ENXIO;
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	return uart_rx_enable(uart, rx->data, sizeof(rx->data), 50);
}

void comm_thread(void)
{
	/* Don't go any further until BLE is initialized */
	// k_sem_take(&ble_init_ok, K_FOREVER);
    int status = 0;

	LOG_DBG("ok");
    status = uart_init();
    if (status)
    {
        return;
    }

    k_fifo_init(&fifo_uart_rx_data);

	for (;;) 
    {
        struct uart_data_t *buf;
        uint8_t len1 = 0;
        uint8_t len2 = 0;

        // if (!k_fifo_is_empty(&fifo_uart_rx_data))
        // {
        //     buf = k_fifo_get(&fifo_uart_rx_data, K_NO_WAIT);
        //     if (NULL != buf)
        //     {
        //         if ((uart_rx_offset + buf->len) > UART_RxBufferSize)
        //         {
        //             len1 = UART_RxBufferSize - uart_rx_offset;
        //             memcpy(&UartRxBuffer[uart_rx_offset], buf->data, len1);
        //             uart_rx_offset = 0;
        //             len2 = buf->len - len1;
        //             memcpy(&UartRxBuffer[uart_rx_offset], &(buf->data[len1]), len2);
        //             uart_rx_offset += len2;
        //         }
        //         else
        //         {
        //             memcpy(&UartRxBuffer[uart_rx_offset], buf->data, buf->len);
        //             uart_rx_offset += buf->len;
        //         }
        //     }
        //     k_free(buf);
        // }
        // nucleo_main();

        k_sleep(K_MSEC(50));
		
	}
}

K_THREAD_DEFINE(comm_id, STACKSIZE, comm_thread, NULL, NULL, NULL, PRIORITY, 0, 0);

