/*
 *  cmd.h
 *
 *  command process
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#ifndef CMD_H
#define CMD_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"
#include "hal.h"





void cmd_init(void);

void cmd_send_error( msg_t *p_msg, err_code_t err_code );
void cmd_read_version( msg_t *p_msg );
void cmd_read_board_name( msg_t *p_msg );
void cmd_read_tag( msg_t *p_msg );
void cmd_jump_to_fw( msg_t *p_msg );
void cmd_flash_fw_write_packet( msg_t *p_msg );
void cmd_flash_fw_write_begin( msg_t *p_msg );
void cmd_flash_fw_write_end( msg_t *p_msg );
void cmd_flash_fw_write_block( msg_t *p_msg );
void cmd_flash_fw_erase( msg_t *p_msg );
void cmd_flash_fw_verify( msg_t *p_msg );
void cmd_flash_fw_read_block( msg_t *p_msg );

void jump_to_fw(void);


#ifdef __cplusplus
}
#endif


#endif

