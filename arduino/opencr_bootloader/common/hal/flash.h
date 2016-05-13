#ifndef LED_H
#define LED_H

#include <stdint.h>



#define FLASH_OK              0
#define FLASH_ERR_TIMEOUT     1





void flash_init(void);

uint8_t flash_write(uint32_t addr, uint8_t *p_data, uint32_t length, uint32_t timeout );
uint8_t flash_read(uint32_t addr, uint8_t *p_data, uint32_t length, uint32_t timeout );


#endif

