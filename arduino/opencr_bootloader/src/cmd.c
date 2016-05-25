/*
 *  cmd.c
 *
 *  message process
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#include "cmd.h"
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>


#define FLASH_FW_SIZE			(768*1024)	// 768KB
#define FLASH_FW_ADDR_START		0x08040000
#define FLASH_FW_ADDR_END		(FLASH_FW_ADDR_START + FLASH_FW_SIZE)

#define FLASH_CONFIG_SIZE		(32*1024)	// 32KB
#define FLASH_CONFIG_ADDR_START		0x08010000
#define FLASH_CONFIG_ADDR_END		(FLASH_CONFIG_ADDR_START + FLASH_CONFIG_SIZE)


#define FLASH_BLOCK_MAX_LENGTH		(16*1024)




uint32_t boot_version = 0x16052300;
uint32_t app_version  = 0x00000000;


typedef struct
{
  uint32_t length;
  uint32_t length_received;
  uint32_t length_total;

  uint16_t count;
  uint16_t count_total;

  uint8_t  data[FLASH_BLOCK_MAX_LENGTH];
} flash_block_t;



flash_block_t flash_block;





void cmd_init(void)
{

}

//-- resp_ack
//
void resp_ack( uint8_t ch, mavlink_ack_t *p_ack )
{
  mavlink_message_t mav_msg;


  mavlink_msg_ack_pack_chan(0, 0, ch, &mav_msg, p_ack->msg_id, p_ack->err_code, p_ack->param);

  msg_send( ch, &mav_msg);
}


//-- cmd_read_version
//
void cmd_read_version( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_read_version_t mav_data;


  mavlink_msg_read_version_decode(p_msg->p_msg, &mav_data);



  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;
    mav_ack.param[0] = boot_version;
    mav_ack.param[1] = boot_version>>8;
    mav_ack.param[2] = boot_version>>16;
    mav_ack.param[3] = boot_version>>24;
    mav_ack.param[4] = app_version;
    mav_ack.param[5] = app_version>>8;
    mav_ack.param[6] = app_version>>16;
    mav_ack.param[7] = app_version>>24;

    resp_ack(p_msg->ch, &mav_ack);
  }
}


//-- cmd_flash_fw_send_block
//
void cmd_flash_fw_send_block( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_flash_fw_send_block_t mav_data;


  mavlink_msg_flash_fw_send_block_decode(p_msg->p_msg, &mav_data);


  flash_block.count += 1;
  flash_block.length_received  += mav_data.length;

  flash_block.count_total += 1;
  flash_block.length_total  += mav_data.length;

  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;
    resp_ack(p_msg->ch, &mav_ack);
  }
}


//-- cmd_flash_fw_write_begin
//
void cmd_flash_fw_write_begin( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_flash_fw_write_begin_t mav_data;


  mavlink_msg_flash_fw_write_begin_decode(p_msg->p_msg, &mav_data);

  flash_block.count = 0;
  flash_block.count_total = 0;

  flash_block.length = 0;
  flash_block.length_total = 0;
  flash_block.length_received = 0;

  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;
    resp_ack(p_msg->ch, &mav_ack);
  }
}


//-- cmd_flash_fw_write_end
//
void cmd_flash_fw_write_end( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_flash_fw_write_end_t mav_data;


  mavlink_msg_flash_fw_write_end_decode(p_msg->p_msg, &mav_data);



  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;
    mav_ack.param[0] = flash_block.count_total;
    mav_ack.param[1] = flash_block.count_total>>8;
    mav_ack.param[2] = flash_block.length_total;
    mav_ack.param[3] = flash_block.length_total>>8;
    mav_ack.param[4] = flash_block.length_total>>16;
    mav_ack.param[5] = flash_block.length_total>>24;

    resp_ack(p_msg->ch, &mav_ack);
  }
}


//-- cmd_flash_fw_write_block
//
void cmd_flash_fw_write_block( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_flash_fw_write_block_t mav_data;


  mavlink_msg_flash_fw_write_block_decode(p_msg->p_msg, &mav_data);


  // TODO ------------------------------------------------
  // 플래시에 블럭 데이터를 Write한다.
  // mav_data.addr : 플래시 메모리 시작 주소
  // mav_data.length : 저장할 데이터 사이즈
  // flash_block.data[] : 저장할 데이터 버퍼
  //

	flash_write(mav_data.addr,flash_block.data, mav_data.length);

  //------------------------------------------------------

  flash_block.count = 0;
  flash_block.length_received = 0;

  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;
    resp_ack(p_msg->ch, &mav_ack);
  }
}


//-- cmd_flash_fw_erase
//
void cmd_flash_fw_erase( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_flash_fw_erase_t mav_data;


  mavlink_msg_flash_fw_erase_decode(p_msg->p_msg, &mav_data);


  // TODO ------------------------------------------------
  // 플래시에 펌웨어 영역 768KB의 영역을 지운다.


	flash_erase_fw_block();


  //------------------------------------------------------

  flash_block.count = 0;
  flash_block.length_received = 0;

  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;
    resp_ack(p_msg->ch, &mav_ack);
  }
}


//-- cmd_flash_fw_verify
//
void cmd_flash_fw_verify( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_flash_fw_verify_t mav_data;


  mavlink_msg_flash_fw_verify_decode(p_msg->p_msg, &mav_data);


  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;
    resp_ack(p_msg->ch, &mav_ack);
  }
}

//-- cmd_flash_fw_req_block
//
void cmd_flash_fw_req_block( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_flash_fw_req_block_t mav_data;


  mavlink_msg_flash_fw_req_block_decode(p_msg->p_msg, &mav_data);


  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;
    resp_ack(p_msg->ch, &mav_ack);
  }
}


//-- cmd_flash_fw_read_block
//
void cmd_flash_fw_read_block( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_flash_fw_read_block_t mav_data;


  mavlink_msg_flash_fw_read_block_decode(p_msg->p_msg, &mav_data);



  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;
    resp_ack(p_msg->ch, &mav_ack);
  }
}


//-- cmd_jump_to_fw
//
void cmd_jump_to_fw( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_jump_to_fw_t mav_data;


  mavlink_msg_jump_to_fw_decode(p_msg->p_msg, &mav_data);



  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;
    resp_ack(p_msg->ch, &mav_ack);
  }
}
