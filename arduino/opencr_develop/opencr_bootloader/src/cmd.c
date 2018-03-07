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

#include "crc.h"


#define FLASH_FW_SIZE             (768*1024)	// 768KB
#define FLASH_FW_ADDR_START       0x08040000
#define FLASH_FW_ADDR_END         (FLASH_FW_ADDR_START + FLASH_FW_SIZE)

#define FLASH_CONFIG_SIZE         (32*1024)	// 32KB
#define FLASH_CONFIG_ADDR_START   0x08010000
#define FLASH_CONFIG_ADDR_END     (FLASH_CONFIG_ADDR_START + FLASH_CONFIG_SIZE)

#define FLASH_BLOCK_PACKET_LENGTH	128
#define FLASH_BLOCK_MAX_LENGTH		(16*1024)



const uint8_t  *board_name   = "OpenCR R1.0";
uint32_t boot_version        = 0x17020800;
uint32_t boot_revision       = 0x00000000;



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



void jump_to_fw(void);



/*---------------------------------------------------------------------------
     TITLE   : cmd_init
     WORK    :
---------------------------------------------------------------------------*/
void cmd_init(void)
{

}


/*---------------------------------------------------------------------------
     TITLE   : resp_ack
     WORK    :
---------------------------------------------------------------------------*/
void resp_ack( uint8_t ch, mavlink_ack_t *p_ack )
{
  mavlink_message_t mav_msg;


  mavlink_msg_ack_pack_chan(0, 0, ch, &mav_msg, p_ack->msg_id, p_ack->err_code, p_ack->length, p_ack->data);

  msg_send( ch, &mav_msg);
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_read_version
     WORK    :
---------------------------------------------------------------------------*/
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
    mav_ack.data[0] = boot_version;
    mav_ack.data[1] = boot_version>>8;
    mav_ack.data[2] = boot_version>>16;
    mav_ack.data[3] = boot_version>>24;
    mav_ack.data[4] = boot_revision;
    mav_ack.data[5] = boot_revision>>8;
    mav_ack.data[6] = boot_revision>>16;
    mav_ack.data[7] = boot_revision>>24;
    mav_ack.length  = 8;
    resp_ack(p_msg->ch, &mav_ack);
  }
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_read_board_name
     WORK    :
---------------------------------------------------------------------------*/
void cmd_read_board_name( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_read_board_name_t mav_data;
  uint8_t i;

  mavlink_msg_read_board_name_decode(p_msg->p_msg, &mav_data);



  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;


    for( i=0; i<strlen(board_name); i++ )
    {
      mav_ack.data[i] = board_name[i];
    }
    mav_ack.data[i] = 0;
    mav_ack.length  = i;
    resp_ack(p_msg->ch, &mav_ack);
  }
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_read_tag
     WORK    :
---------------------------------------------------------------------------*/
void cmd_read_tag( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_read_tag_t mav_data;

  mavlink_msg_read_tag_decode(p_msg->p_msg, &mav_data);



  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;

    mav_ack.length  = 0;
    resp_ack(p_msg->ch, &mav_ack);
  }
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_packet
     WORK    :
---------------------------------------------------------------------------*/
void cmd_flash_fw_write_packet( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_flash_fw_write_packet_t mav_data;

  mavlink_msg_flash_fw_write_packet_decode(p_msg->p_msg, &mav_data);

  if((flash_block.length_received + mav_data.length) <= FLASH_BLOCK_MAX_LENGTH)
  {
    memcpy(&flash_block.data[flash_block.length_received], &mav_data.data[0], mav_data.length);
  }

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


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_begin
     WORK    :
---------------------------------------------------------------------------*/
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


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_end
     WORK    :
---------------------------------------------------------------------------*/
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
    mav_ack.data[0] = flash_block.count_total;
    mav_ack.data[1] = flash_block.count_total>>8;
    mav_ack.data[2] = flash_block.length_total;
    mav_ack.data[3] = flash_block.length_total>>8;
    mav_ack.data[4] = flash_block.length_total>>16;
    mav_ack.data[5] = flash_block.length_total>>24;
    mav_ack.length  = 6;

    resp_ack(p_msg->ch, &mav_ack);
  }
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_block
     WORK    :
---------------------------------------------------------------------------*/
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

  err_code = flash_write( FLASH_FW_ADDR_START + mav_data.addr,flash_block.data, mav_data.length);

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


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_erase
     WORK    :
---------------------------------------------------------------------------*/
void cmd_flash_fw_erase( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_flash_fw_erase_t mav_data;


  mavlink_msg_flash_fw_erase_decode(p_msg->p_msg, &mav_data);


  // TODO ------------------------------------------------
  // 플래시에 펌웨어 영역 768KB의 영역을 지운다.

  if( mav_data.length > FLASH_FW_SIZE )
  {
    err_code = ERR_FLASH_SIZE;
  }
  else
  {
    err_code = flash_erase_fw_block( mav_data.length );
  }


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


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_verify
     WORK    :
---------------------------------------------------------------------------*/
void cmd_flash_fw_verify( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_flash_fw_verify_t mav_data;
  uint32_t crc = 0;
  uint32_t i;
  uint8_t *p_fw = (uint8_t *)FLASH_FW_ADDR_START;


  mavlink_msg_flash_fw_verify_decode(p_msg->p_msg, &mav_data);

  crc = 0;
  for( i=0; i<mav_data.length; i++ )
  {
    crc = crc_calc( crc, p_fw[i] );
  }


  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;

    if( crc != mav_data.crc )
    {
      err_code = ERR_FLASH_CRC;
    }

    mav_ack.data[0] = crc >> 0;
    mav_ack.data[1] = crc >> 8;
    mav_ack.data[2] = crc >> 16;
    mav_ack.data[3] = crc >> 24;
    mav_ack.length = 4;

    mav_ack.err_code = err_code;
    resp_ack(p_msg->ch, &mav_ack);
  }
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_read_packet
     WORK    :
---------------------------------------------------------------------------*/
void cmd_flash_fw_read_packet( uint8_t ch, mavlink_flash_fw_read_packet_t *p_msg )
{
  mavlink_message_t mav_msg;


  mavlink_msg_flash_fw_read_packet_pack_chan(0, 0, ch, &mav_msg, p_msg->resp, p_msg->addr, p_msg->length, p_msg->data);
  msg_send( ch, &mav_msg);
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_read_block
     WORK    :
---------------------------------------------------------------------------*/
#if 0
void cmd_flash_fw_read_block( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_flash_fw_read_t       mav_resp;
  mavlink_flash_fw_read_block_t mav_data;
  uint16_t block_cnt = 0;
  uint16_t i;
  uint8_t *p_fw = (uint8_t *)FLASH_FW_ADDR_START;


  mavlink_msg_flash_fw_read_block_decode(p_msg->p_msg, &mav_data);


  flash_block.length_received = 0;
  flash_block.length_total    = mav_data.length;

  if( mav_data.resp == 1 )
  {
    if( mav_data.length > FLASH_BLOCK_MAX_LENGTH )
    {
      mav_data.length = FLASH_BLOCK_MAX_LENGTH;
    }

    memcpy(flash_block.data, &p_fw[mav_data.addr], mav_data.length );


    block_cnt = mav_data.length/FLASH_BLOCK_PACKET_LENGTH;

    if( (block_cnt%FLASH_BLOCK_PACKET_LENGTH) > 0 )
    {
      block_cnt += 1;
    }

    for(i=0; i<block_cnt; i++)
    {
      flash_block.length = flash_block.length_total - flash_block.length_received;
      if( flash_block.length > FLASH_BLOCK_PACKET_LENGTH )
      {
	flash_block.length = FLASH_BLOCK_PACKET_LENGTH;
      }

      mav_resp.addr   = flash_block.length_received;
      mav_resp.length = flash_block.length;

      memcpy(&mav_resp.data[0], &flash_block.data[0], flash_block.length);

      cmd_flash_fw_read(p_msg->ch, &mav_resp);
    }
  }
}
#else
void cmd_flash_fw_read_block( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_flash_fw_read_packet_t  mav_resp;
  mavlink_flash_fw_read_block_t   mav_data;
  uint16_t block_cnt = 0;
  uint16_t i;
  uint8_t *p_fw = (uint8_t *)FLASH_FW_ADDR_START;


  mavlink_msg_flash_fw_read_block_decode(p_msg->p_msg, &mav_data);


  flash_block.length_received = 0;
  flash_block.length_total    = mav_data.length;

  if( mav_data.resp == 1 )
  {
    mav_resp.addr   = mav_data.addr;
    mav_resp.length = mav_data.length;

    memcpy(mav_resp.data, &p_fw[mav_data.addr], mav_data.length );

    cmd_flash_fw_read_packet(p_msg->ch, &mav_resp);
  }
}
#endif


/*---------------------------------------------------------------------------
     TITLE   : cmd_jump_to_fw
     WORK    :
---------------------------------------------------------------------------*/
void cmd_jump_to_fw( msg_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_jump_to_fw_t mav_data;


  mavlink_msg_jump_to_fw_decode(p_msg->p_msg, &mav_data);



  jump_to_fw();


  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;
    resp_ack(p_msg->ch, &mav_ack);
  }
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_send_error
     WORK    :
---------------------------------------------------------------------------*/
void cmd_send_error( msg_t *p_msg, err_code_t err_code )
{

  mavlink_ack_t     mav_ack;
  mavlink_read_version_t mav_data;


  mavlink_msg_read_version_decode(p_msg->p_msg, &mav_data);

  mav_ack.msg_id   = p_msg->p_msg->msgid;
  mav_ack.err_code = err_code;
  resp_ack(p_msg->ch, &mav_ack);
}


/*---------------------------------------------------------------------------
     TITLE   : check_fw
     WORK    :
---------------------------------------------------------------------------*/
BOOL check_fw(void)
{
  BOOL ret = TRUE;
  uint32_t *p_addr = (uint32_t *)FLASH_FW_ADDR_START;


  if( p_addr[0] == 0xFFFFFFFF ) ret = FALSE;


  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : jump_to_fw
     WORK    :
---------------------------------------------------------------------------*/
void jump_to_fw(void)
{

  if( check_fw() == FALSE ) return;

  bsp_deinit();

  SCB->VTOR = FLASH_FW_ADDR_START;

  __asm volatile("ldr r0, =0x08040000 \n"
                 "ldr sp, [r0]        \n"
                 "ldr pc, [r0, #4]    \n");
}

