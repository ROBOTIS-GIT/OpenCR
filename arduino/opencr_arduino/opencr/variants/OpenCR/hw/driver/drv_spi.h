/*
 *  drv_spi.h
 *
 *  Created on: 2016. 7.13.
 *      Author: Baram, PBHP
 */

#ifndef DRV_SPI_H
#define DRV_SPI_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"




typedef void (*DrvSPIDMACallback)(SPI_HandleTypeDef* hspi);



int     drv_spi_init();
void    drv_spi_enable_dma(SPI_HandleTypeDef* hspi);
bool    drv_spi_dma_enabled(SPI_HandleTypeDef* hspi);
uint8_t drv_spi_is_dma_tx_done(SPI_HandleTypeDef* hspi);
void    drv_spi_start_dma_tx(SPI_HandleTypeDef* hspi, uint8_t *p_buf, uint32_t length, DrvSPIDMACallback dma_callback);
void    drv_spi_start_dma_txrx(SPI_HandleTypeDef* hspi, uint8_t *p_buf, uint8_t *p_rxbuf, uint32_t length, 
			DrvSPIDMACallback dma_callback);

#ifdef __cplusplus
}
#endif


#endif
