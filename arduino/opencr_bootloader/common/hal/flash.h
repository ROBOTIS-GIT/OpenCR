#ifndef FLASH_H
#define FLASH_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"


typedef enum //PBHP_160514
{
  FLASH_OK       = 0x00,
  FLASH_ERROR    = 0x01,
  FLASH_BUSY     = 0x02,
  FLASH_ERR_TIMEOUT  = 0x03,
  FLASJ_NOT_EMPTY = 0x04
} FLASH_StatusTypeDef;

#define	TRUE	1
#define	FALSE	0

void flash_init(void);

uint8_t flash_erase_whole_sectors(void);
uint8_t flash_erase_sector(uint32_t sector);

uint8_t flash_write(uint32_t addr, uint8_t *p_data, uint32_t length );
uint8_t flash_read(uint32_t addr, uint8_t *p_data, uint32_t length );


#ifdef __cplusplus
}
#endif


#endif

