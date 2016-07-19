/*
 *  main.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBPH
 */

#ifndef __OPENCR_LD_MAIN_H_
#define __OPENCR_LD_MAIN_H_

#include <unistd.h>
#include <stdint.h>
#include "type.h"
#include "./msg/msg.h"
//#include "serial.h"


#define GET_CALC_TIME(x)	( (int)(x / 1000) + ((float)(x % 1000))/1000 )

#define FLASH_TX_BLOCK_LENGTH	(8*1024)
#define FLASH_RX_BLOCK_LENGTH	(128)
#define FLASH_PACKET_LENGTH   	128


int opencr_ld_main( int argc, const char **argv );

int opencr_ld_down( int argc, const char **argv );
int opencr_ld_flash_write( uint32_t addr, uint8_t *p_data, uint32_t length  );
int opencr_ld_flash_read( uint32_t addr, uint8_t *p_data, uint32_t length  );
int opencr_ld_flash_erase( uint32_t length  );

uint32_t opencr_ld_file_read_data( uint8_t *dst, uint32_t len );


static long iclock();
int read_byte( void );
void delay_ms( int WaitTime );
uint32_t crc_calc( uint32_t crc_in, uint8_t data_in );


err_code_t cmd_read_version( uint32_t *p_version, uint32_t *p_revision );
err_code_t cmd_read_board_name( uint8_t *p_str, uint8_t *p_len );
err_code_t cmd_flash_fw_erase( uint32_t length );
err_code_t cmd_flash_fw_write_begin( void );
err_code_t cmd_flash_fw_write_end( void );
err_code_t cmd_flash_fw_write_packet( uint16_t addr, uint8_t *p_data, uint8_t length );
err_code_t cmd_flash_fw_write_block( uint32_t addr, uint32_t length  );
err_code_t cmd_flash_fw_send_block_multi( uint8_t block_count );
err_code_t cmd_flash_fw_read_block( uint32_t addr, uint8_t *p_data, uint16_t length );
err_code_t cmd_flash_fw_verify( uint32_t length, uint32_t crc, uint32_t *p_crc_ret );
err_code_t cmd_jump_to_fw(void);


#endif

