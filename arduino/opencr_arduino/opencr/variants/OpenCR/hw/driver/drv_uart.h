/*
 *  drv_uart.h
 *
 *  Created on: 2016. 7.13.
 *      Author: Baram, PBHP
 */

#ifndef DRV_UART_H
#define DRV_UART_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"





#define DRV_UART_NUM_1          0
#define DRV_UART_NUM_2          1
#define DRV_UART_NUM_3          2
#define DRV_UART_NUM_4          3
#define DRV_UART_NUM_MAX        4

#define DRV_UART_IRQ_MODE       0
#define DRV_UART_DMA_MODE       1

//#define DRV_UART_RX_DMA_ONLY	1  

int      drv_uart_init();
void     drv_uart_begin(uint8_t uart_num, uint8_t uart_mode, uint32_t baudrate);
uint32_t drv_uart_write(uint8_t uart_num, const uint8_t wr_data);
uint32_t drv_uart_write_dma_it(uint8_t uart_num, const uint8_t *wr_data, uint16_t Size);
void     drv_uart_flush(uint8_t uart_num);
void     drv_uart_rx_flush(uint8_t uart_num, uint32_t timeout_ms);
void     drv_uart_start_rx(uint8_t uart_num);
uint32_t drv_uart_read_buf(uint8_t uart_num, uint8_t *p_buf, uint32_t length);
uint8_t  drv_uart_get_mode(uint8_t uart_num);
uint32_t drv_uart_available(uint8_t uart_num);
int      drv_uart_read(uint8_t uart_num);
int      drv_uart_peek(uint8_t uart_num);

#ifdef __cplusplus
}
#endif


#endif
