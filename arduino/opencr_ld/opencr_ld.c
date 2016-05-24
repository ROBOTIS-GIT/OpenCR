#include "opencr_ld.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include "serial.h"
#include "type.h"
#include "./msg/msg.h"
#include <sys/time.h>
#include <stdio.h>


ser_handler stm32_ser_id = ( ser_handler )-1;


int opencr_ld_init( const char *portname, u32 baud );

int read_byte( void );
err_code_t cmd_read_version( void );
err_code_t cmd_flash_fw_write_begin( void );
err_code_t cmd_flash_fw_write_end( void );
err_code_t cmd_flash_fw_send_block( void );
err_code_t cmd_flash_fw_write_block( void );
err_code_t cmd_flash_fw_send_block_multi( uint8_t block_count );



static long myclock()
{
	struct timeval tv;
	gettimeofday (&tv, NULL);
	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}


//-- opencr_ld_main
//
int opencr_ld_main( int argc, const char **argv )
{
  long baud;
  baud = strtol( argv[ 2 ], NULL, 10 );

  printf("opencr_ld_main \r\n");

  opencr_ld_init( argv[ 1 ], baud );

  return 0;
}


//-- delay_ms
//
void delay_ms( int WaitTime )
{
  int i;

  #ifdef WIN32_BUILD
  Sleep(WaitTime);
  #else
  for( i=0; i<WaitTime; i++ )
  {
    usleep(1000);
  }
  #endif
}


/*---------------------------------------------------------------------------
     TITLE   : read_byte
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
int read_byte( void )
{
  return ser_read_byte( stm32_ser_id );
}


/*---------------------------------------------------------------------------
     TITLE   : write_bytes
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
int write_bytes( char *p_data, int len )
{
  int written_len;

  written_len = ser_write( stm32_ser_id, (const u8 *)p_data, len );

  return written_len;
}


/*---------------------------------------------------------------------------
     TITLE   : OpenCM_Cmd_Init
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
int opencr_ld_init( const char *portname, u32 baud )
{
  int i;
  int j;
  err_code_t err_code = OK;


  // Open port
  if( ( stm32_ser_id = ser_open( portname ) ) == ( ser_handler )-1 )
  {
    printf("Fail to open port 1\n");
    return -1;
  }

  // Setup port
  ser_setupEx( stm32_ser_id, baud, SER_DATABITS_8, SER_PARITY_NONE, SER_STOPBITS_1, 1 );



  ser_set_timeout_ms( stm32_ser_id, SER_NO_TIMEOUT );
  while( read_byte() != -1 );
  ser_set_timeout_ms( stm32_ser_id, 2000000 );




  printf("send cmd\r\n");

  cmd_read_version();
  err_code = cmd_flash_fw_write_begin();
  if( err_code != OK ) printf("cmd_flash_fw_write_begin ERR : 0x%04X\r\n", err_code);

  long t, dt;
  float calc_time;
  t = myclock();

  for( j=0; j<96; j++ )
  {
    for( i=0; i<64; i++ )
    {
      cmd_flash_fw_send_block();
    }
    err_code = cmd_flash_fw_write_block();
    if( err_code != OK )
    {
      printf("ERR : 0x%04X\r\n", err_code);
      break;
    }
  }

  cmd_flash_fw_write_end();

  dt = myclock() - t;


  ser_close( stm32_ser_id );
  calc_time = (int)(dt / 1000) + ((float)(dt % 1000))/1000;
  printf("end %f sec, %f KB/s\n", calc_time, (768/calc_time) );



  return TRUE;
}


err_code_t cmd_read_version( void )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 1;


  mavlink_msg_read_version_pack(0, 0, &tx_msg, resp, param);
  msg_send(0, &tx_msg);

  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      printf("BootVersion : 0x%08X\r\n", ack_msg.param[3]<<24|ack_msg.param[2]<<16|ack_msg.param[1]<<8|ack_msg.param[0]);
      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }

  return OK;
}


err_code_t cmd_flash_fw_write_begin( void )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 1;


  mavlink_msg_flash_fw_write_begin_pack(0, 0, &tx_msg, resp, param);
  msg_send(0, &tx_msg);

  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }

  return err_code;
}


err_code_t cmd_flash_fw_write_end( void )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 1;


  mavlink_msg_flash_fw_write_end_pack(0, 0, &tx_msg, resp, param);
  msg_send(0, &tx_msg);

  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      printf("block_count  : %d\r\n", ack_msg.param[1]<<8|ack_msg.param[0]);
      printf("block_length : %d\r\n", ack_msg.param[5]<<24|ack_msg.param[4]<<16|ack_msg.param[3]<<8|ack_msg.param[2]);


      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }

  return err_code;
}


err_code_t cmd_flash_fw_send_block( void )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t buf[256];
  uint8_t resp = 0;


  mavlink_msg_flash_fw_send_block_pack(0, 0, &tx_msg, resp, 0, 128, buf);
  msg_send(0, &tx_msg);


  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }

  return err_code;
}


err_code_t cmd_flash_fw_send_block_multi( uint8_t block_count )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t buf[256];
  uint8_t tx_buf[16*1024];
  uint8_t resp = 0;
  uint8_t i;
  uint32_t len;


   len = 0;
  for( i=0; i<block_count; i++ )
  {
    mavlink_msg_flash_fw_send_block_pack(0, 0, &tx_msg, resp, 0, 128, buf);
    len += mavlink_msg_to_send_buffer(&tx_buf[len], &tx_msg);
  }
  write_bytes((char *)tx_buf, len);

  return err_code;
}


err_code_t cmd_flash_fw_write_block( void )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t buf[256];
  uint8_t resp = 1;

  mavlink_msg_flash_fw_write_block_pack(0, 0, &tx_msg, resp, 0, 128);
  msg_send(0, &tx_msg);


  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      if( tx_msg.msgid == ack_msg.msg_id ) err_code = ack_msg.err_code;
      else                                 err_code = ERR_MISMATCH_ID;
    }
    else
    {
      err_code = tx_msg.msgid<<8|ERR_TIMEOUT;
    }
  }

  return err_code;
}
