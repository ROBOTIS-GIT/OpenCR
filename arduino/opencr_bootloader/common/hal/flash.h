/*
 *  flash.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#ifndef FLASH_H
#define FLASH_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"









void flash_init(void);

err_code_t flash_write(uint32_t addr, uint8_t *p_data, uint32_t length);
err_code_t flash_read(uint32_t addr, uint8_t *p_data, uint32_t length);
err_code_t flash_erase_whole_sectors(void);
err_code_t flash_erase_sector(uint32_t sector);
err_code_t flash_erase_fw_block(uint32_t length);


#ifdef __cplusplus
}
#endif


#endif

