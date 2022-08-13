#ifndef COMM_H
#define COMM_H

#include "com.h"

#define UART_BUF_SIZE (TMsg_MaxLen + 10)
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX 50

extern uint16_t uart_rx_offset;

#endif