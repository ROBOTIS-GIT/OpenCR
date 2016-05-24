/*
 *  cmd.h
 *
 *  command process
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBPH
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

void cmd_read_version( msg_t *p_msg );
void cmd_jump_to_fw( msg_t *p_msg );
void cmd_flash_fw_send_block( msg_t *p_msg );
void cmd_flash_fw_write_begin( msg_t *p_msg );
void cmd_flash_fw_write_end( msg_t *p_msg );
void cmd_flash_fw_write_block( msg_t *p_msg );
void cmd_flash_fw_erase( msg_t *p_msg );
void cmd_flash_fw_verify( msg_t *p_msg );
void cmd_flash_fw_req_block( msg_t *p_msg );
void cmd_flash_fw_read_block( msg_t *p_msg );


#ifdef __cplusplus
}
#endif


#endif

