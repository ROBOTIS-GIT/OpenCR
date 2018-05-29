#include "opencr_ld.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdarg.h>

#include "serial.h"
#include "type.h"
#include "./msg/msg.h"
#include <sys/time.h>
#include <stdio.h>



static FILE      *opencr_fp;
static uint32_t   opencr_fpsize;


ser_handler stm32_ser_id = ( ser_handler )-1;


#define GET_CALC_TIME(x)	( (int)(x / 1000) + ((float)(x % 1000))/1000 )

#define FLASH_TX_BLOCK_LENGTH	(8*1024)
#define FLASH_RX_BLOCK_LENGTH	(128)
#define FLASH_PACKET_LENGTH   	128


uint32_t tx_buf[768*1024/4];
uint32_t rx_buf[768*1024/4];

char err_msg_str[512];


int opencr_ld_down( int argc, const char **argv );
int opencr_ld_jump_to_boot( char *portname );
int opencr_ld_flash_write( uint32_t addr, uint8_t *p_data, uint32_t length  );
int opencr_ld_flash_read( uint32_t addr, uint8_t *p_data, uint32_t length  );
int opencr_ld_flash_erase( uint32_t length  );

uint32_t opencr_ld_file_read_data( uint8_t *dst, uint32_t len );

void opencr_ld_write_err_msg( const char *fmt, ...);
void opencr_ld_print_err_msg(void);

static long iclock();
int read_byte( void );
int write_bytes( char *p_data, int len );
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




/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_main
     WORK    :
---------------------------------------------------------------------------*/
int opencr_ld_main( int argc, const char **argv )
{
  long baud;
  baud = strtol( argv[ 2 ], NULL, 10 );

  printf("opencr_ld_main \r\n");

  opencr_ld_down( argc, argv );

  return 0;
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_down
     WORK    :
---------------------------------------------------------------------------*/
int opencr_ld_down( int argc, const char **argv )
{
  int i;
  int j;
  int ret = 0;
  err_code_t err_code = OK;
  long t, dt;
  float calc_time;
  uint32_t fw_size = 256*1024*3;
  uint8_t  board_str[16];
  uint8_t  board_str_len;
  uint32_t board_version;
  uint32_t board_revision;
  uint32_t crc;
  uint32_t crc_ret = 0;
  uint8_t  *p_buf_crc;
  char *portname;
  uint32_t baud;
  uint8_t  block_buf[FLASH_TX_BLOCK_LENGTH];
  uint32_t addr;
  uint32_t len;
  uint8_t jump_to_fw = 0;
  uint8_t retry;


  baud     = strtol( argv[ 2 ], NULL, 10 );
  portname = (char *)argv[ 1 ];

  if( argc >= 5 && strlen(argv[ 4 ])==1 && strncmp(argv[ 4 ], "1", 1)==0 )
  {
    jump_to_fw = 1;
  }

  if( ( opencr_fp = fopen( argv[ 3 ], "rb" ) ) == NULL )
  {
    fprintf( stderr, "Unable to open %s\n", argv[ 3 ] );
    exit( 1 );
  }
  else
  {
    fseek( opencr_fp, 0, SEEK_END );
    opencr_fpsize = ftell( opencr_fp );
    fseek( opencr_fp, 0, SEEK_SET );
    printf(">>\r\n");
    printf("file name : %s \r\n", argv[3]);
    printf("file size : %d KB\r\n", opencr_fpsize/1024);
  }

  fw_size = opencr_fpsize;


  // Jump To Boot
  if( opencr_ld_jump_to_boot(portname ) < 0 )
  {
    printf("Fail to jump to boot\n");
    return -1;
  }


  // Open port
  if( ( stm32_ser_id = ser_open( portname ) ) == ( ser_handler )-1 )
  {
    printf("Fail to open port 1\n");
    return -1;
  }
  else
  {
    printf("Open port OK\n");
  }

  // Setup port
  ser_setupEx( stm32_ser_id, 115200, SER_DATABITS_8, SER_PARITY_NONE, SER_STOPBITS_1, 1 );

  printf("Clear Buffer Start\n");
  ser_set_timeout_ms( stm32_ser_id, SER_NO_TIMEOUT );
  while( read_byte() != -1 );
  ser_set_timeout_ms( stm32_ser_id, 1000 );
  printf("Clear Buffer End\n");


  err_code = cmd_read_board_name( board_str, &board_str_len );
  if( err_code == OK )
  {
    printf("Board Name : %s\r\n", board_str);
  }
  else
  {
    printf("cmd_read_board_name fail : 0x%X\n", err_code);
    ser_close( stm32_ser_id );
    fclose( opencr_fp );
    exit(1);
  }
  err_code = cmd_read_version( &board_version, &board_revision );
  if( err_code == OK )
  {
    printf("Board Ver  : 0x%08X\r\n", board_version);
    printf("Board Rev  : 0x%08X\r\n", board_revision);
  }
  printf(">>\r\n");

  t = iclock();
  ret = opencr_ld_flash_erase(fw_size);
  dt = iclock() - t;
  printf("flash_erase : %d : %f sec\r\n", ret, GET_CALC_TIME(dt));
  if( ret < 0 )
  {
    ser_close( stm32_ser_id );
    fclose( opencr_fp );
    exit(1);
  }

#if 1
  t = iclock();
  crc  = 0;
  addr = 0;
  while(1)
  {
    len = opencr_ld_file_read_data( block_buf, FLASH_TX_BLOCK_LENGTH);
    if( len == 0 ) break;

    for( i=0; i<len; i++ )
    {
      crc = crc_calc( crc,  block_buf[i] );
    }

    for( retry=0; retry<3; retry++ )
    {
      ret = opencr_ld_flash_write( addr, block_buf, len );
      if( ret >= 0 ) break;
    }
    if( ret < 0 ) break;

    addr += len;
  }
  dt = iclock() - t;

  printf("flash_write : %d : %f sec \r\n", ret,  GET_CALC_TIME(dt));
  if( ret < 0 )
  {
    ser_close( stm32_ser_id );
    fclose( opencr_fp );
    opencr_ld_print_err_msg();
    printf("[FAIL] Download \r\n");
    return -2;
  }


#else
  for( i=0; i<fw_size/4; i++ )
  {
    tx_buf[i] = i;
  }

  t = iclock();
  crc = 0;
  p_buf_crc = (uint8_t *)tx_buf;
  for( i=0; i<fw_size; i++ )
  {
    crc = crc_calc( crc,  p_buf_crc[i] );
  }
  dt = iclock() - t;
  printf("calc crc : %f sec \r\n", GET_CALC_TIME(dt));


  t = iclock();
  ret = opencr_ld_flash_write( 0, (uint8_t *)tx_buf, fw_size );
  dt = iclock() - t;


  calc_time = GET_CALC_TIME(dt);
  printf("opencr_ld_flash_write : %d : %f sec, %f KB/s\n", ret, calc_time, (fw_size/1024/calc_time) );

  memset(rx_buf, 0, 768*1024);
  t = iclock();
  ret = opencr_ld_flash_read( 0, (uint8_t *)rx_buf, fw_size );
  dt = iclock() - t;
  printf("opencr_ld_flash_read : %d : %f sec \r\n", ret,  GET_CALC_TIME(dt));

  ret = 0;
  for( i=0; i<fw_size/4; i++ )
  {
    if( tx_buf[i] != rx_buf[i] )
    {
      printf("Compare Error : 0x%X \r\n", i*4);
      ret = -1;
      break;
    }
  }
  if( ret == 0 )
  {
    printf("Compare OK \r\n");
  }
#endif


  for(i = 0; i < 3; i++)
  {
    if( i > 0)
    {
      printf("CRC Retry : %d\r\n", i);
    }

    t = iclock();
    err_code = cmd_flash_fw_verify( fw_size, crc, &crc_ret );
    dt = iclock() - t;
    
    if(err_code == OK)
    {
      break;
    }
  }

  if( err_code == OK )
  {
    printf("CRC OK %X %X %f sec\r\n", crc, crc_ret, GET_CALC_TIME(dt));
  }
  else
  {
    printf("CRC Fail : 0x%X : %X, %X %f sec\r\n", err_code, crc, crc_ret, GET_CALC_TIME(dt));
    printf("[FAIL] Download \r\n");
    ser_close( stm32_ser_id );
    fclose( opencr_fp );
    return -3;
  }

  printf("[OK] Download \r\n");

  if( jump_to_fw == 1 )
  {
    printf("jump_to_fw \r\n");
    cmd_jump_to_fw();
  }

  ser_close( stm32_ser_id );
  fclose( opencr_fp );


  for (int i=0; i<6; i++)
  {
    delay_ms(500);
    if (ser_port_is_ready(portname) > 0)
    {
      printf("jump finished\r\n");
      delay_ms(100);
      break;
    }
  }

  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_file_read_data
     WORK    :
---------------------------------------------------------------------------*/
int opencr_ld_jump_to_boot( char *portname )
{
  bool ret;


  // Open port
  if( ( stm32_ser_id = ser_open( portname ) ) == ( ser_handler )-1 )
  {
    printf("Fail to open port 1 : %s\n", portname);
    return -1;
  }

  // Setup port
  ser_setupEx( stm32_ser_id, 1200, SER_DATABITS_8, SER_PARITY_NONE, SER_STOPBITS_1, 1 );

  write_bytes("OpenCR 5555AAAA", 15);
  ser_close( stm32_ser_id );

  delay_ms(1500);

  return 0;
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_file_read_data
     WORK    :
---------------------------------------------------------------------------*/
uint32_t opencr_ld_file_read_data( uint8_t *dst, uint32_t len )
{
  size_t readbytes = 0;

  if( !feof( opencr_fp ) )
  {
    readbytes = fread( dst, 1, len, opencr_fp );
  }
  return ( uint32_t )readbytes;
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_flash_write
     WORK    :
---------------------------------------------------------------------------*/
int opencr_ld_flash_write( uint32_t addr, uint8_t *p_data, uint32_t length  )
{
  int ret = 0;
  err_code_t err_code = OK;
  uint32_t block_length;
  uint16_t block_cnt;
  uint32_t written_packet_length;
  uint32_t written_total_length;
  uint32_t packet_length = 128;
  uint32_t i;


  err_code = cmd_flash_fw_write_begin();
  if( err_code != OK )
  {
    opencr_ld_write_err_msg("cmd_flash_fw_write_begin ERR : 0x%04X\r\n", err_code);

    return -1;
  }

  written_total_length = 0;

  while(1)
  {
    block_length = length - written_total_length;

    if( block_length > FLASH_TX_BLOCK_LENGTH )
    {
      block_length = FLASH_TX_BLOCK_LENGTH;
    }

    block_cnt = block_length/FLASH_PACKET_LENGTH;
    if( block_length%FLASH_PACKET_LENGTH > 0 )
    {
      block_cnt += 1;
    }


    written_packet_length = 0;
    for( i=0; i<block_cnt; i++ )
    {
      packet_length = block_length - written_packet_length;
      if( packet_length > FLASH_PACKET_LENGTH )
      {
        packet_length = FLASH_PACKET_LENGTH;
      }

      err_code = cmd_flash_fw_write_packet(written_packet_length, &p_data[written_total_length+written_packet_length], packet_length);
      if( err_code != OK )
      {
        opencr_ld_write_err_msg("cmd_flash_fw_send_block ERR : 0x%04X\r\n", err_code);
        return -2;
      }

      written_packet_length += packet_length;
    }

    //printf("%d : %d, %d, %d \r\n", written_packet_length, block_length, block_cnt, packet_length);

    if( written_packet_length == block_length )
    {
      err_code = cmd_flash_fw_write_block(addr+written_total_length, block_length);
      if( err_code != OK )
      {
        opencr_ld_write_err_msg("cmd_flash_fw_write_block ERR : 0x%04X\r\n", err_code);
        return -3;
      }
    }
    else
    {
      opencr_ld_write_err_msg("written_packet_length : %d, %d 0x%04X\r\n", written_packet_length, block_length, err_code);
      return -4;
    }

    written_total_length += block_length;

    if( written_total_length == length )
    {
      break;
    }
    else if( written_total_length > length )
    {
      opencr_ld_write_err_msg("written_total_length over \r\n");
      return -5;
    }
  }


  cmd_flash_fw_write_end();

  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_flash_read
     WORK    :
---------------------------------------------------------------------------*/
int opencr_ld_flash_read( uint32_t addr, uint8_t *p_data, uint32_t length  )
{
  int ret = 0;
  err_code_t err_code = OK;
  uint32_t block_length;
  uint32_t read_packet_length;
  uint32_t read_total_length;
  int i;
  int err_count = 0;

  read_total_length = 0;

  while(1)
  {
    block_length = length - read_total_length;

    if( block_length > FLASH_PACKET_LENGTH )
    {
      block_length = FLASH_PACKET_LENGTH;
    }


    for( i=0; i<3; i++ )
    {
      err_code = cmd_flash_fw_read_block( addr+read_total_length, &p_data[read_total_length], block_length );
      if( err_code == OK ) break;
      err_count++;
    }


    if( err_code != OK )
    {
      printf("cmd_flash_fw_read_block : addr:%X, 0x%04X \r\n", addr+read_total_length, err_code);
      return -1;
    }

    read_total_length += block_length;

    if( read_total_length == length )
    {
      break;
    }
    else if( read_total_length > length )
    {
      printf("read_total_length over \r\n");
      return -2;
    }
  }

  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_flash_erase
     WORK    :
---------------------------------------------------------------------------*/
int opencr_ld_flash_erase( uint32_t length  )
{
  int ret = 0;
  err_code_t err_code = OK;

  err_code = cmd_flash_fw_erase( length );

  if( err_code != OK )
  {
    printf("cmd_flash_fw_erase_block : 0x%04X %d\r\n", err_code, length );
    return -1;
  }

  return ret;
}



static long iclock()
{
	struct timeval tv;
	gettimeofday (&tv, NULL);
	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}


/*---------------------------------------------------------------------------
     TITLE   : delay_ms
     WORK    :
---------------------------------------------------------------------------*/
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
---------------------------------------------------------------------------*/
int read_byte( void )
{
  return ser_read_byte( stm32_ser_id );
}



/*---------------------------------------------------------------------------
     TITLE   : read_bytes
     WORK    :
---------------------------------------------------------------------------*/
int read_bytes( uint8_t *pData, uint32_t size )
{
  return read( stm32_ser_id, pData, size ); //ser_read( stm32_ser_id, pData, size );
  //return ser_read( stm32_ser_id, pData, size );
}



/*---------------------------------------------------------------------------
     TITLE   : write_bytes
     WORK    :
---------------------------------------------------------------------------*/
int write_bytes( char *p_data, int len )
{
  int written_len;

  written_len = ser_write( stm32_ser_id, (const u8 *)p_data, len );

  return written_len;
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_read_version
     WORK    :
---------------------------------------------------------------------------*/
err_code_t cmd_read_version( uint32_t *p_version, uint32_t *p_revision )
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

      //printf("BootVersion : 0x%08X\r\n", ack_msg.data[3]<<24|ack_msg.data[2]<<16|ack_msg.data[1]<<8|ack_msg.data[0]);
      *p_version  = ack_msg.data[3]<<24|ack_msg.data[2]<<16|ack_msg.data[1]<<8|ack_msg.data[0];
      *p_revision = ack_msg.data[7]<<24|ack_msg.data[6]<<16|ack_msg.data[5]<<8|ack_msg.data[4];
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


/*---------------------------------------------------------------------------
     TITLE   : cmd_read_board_name
     WORK    :
---------------------------------------------------------------------------*/
err_code_t cmd_read_board_name( uint8_t *p_str, uint8_t *p_len )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 1;

  mavlink_msg_read_board_name_pack(0, 0, &tx_msg, resp, param);
  msg_send(0, &tx_msg);
  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      *p_len = ack_msg.length;
      memcpy(p_str, ack_msg.data, ack_msg.length);
      p_str[ack_msg.length] = 0;

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


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_erase
     WORK    :
---------------------------------------------------------------------------*/
err_code_t cmd_flash_fw_erase( uint32_t length )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 1;


  mavlink_msg_flash_fw_erase_pack(0, 0, &tx_msg, resp, length, param);
  msg_send(0, &tx_msg);

  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 3000) == TRUE )
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

/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_begin
     WORK    :
---------------------------------------------------------------------------*/
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


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_end
     WORK    :
---------------------------------------------------------------------------*/
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

      //printf("block_count  : %d\r\n", ack_msg.data[1]<<8|ack_msg.data[0]);
      //printf("block_length : %d\r\n", ack_msg.data[5]<<24|ack_msg.data[4]<<16|ack_msg.data[3]<<8|ack_msg.data[2]);


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


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_packet
     WORK    :
---------------------------------------------------------------------------*/
err_code_t cmd_flash_fw_write_packet( uint16_t addr, uint8_t *p_data, uint8_t length )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t resp = 0;



  mavlink_msg_flash_fw_write_packet_pack(0, 0, &tx_msg, resp, addr, length, p_data);
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


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_send_block_multi
     WORK    :
---------------------------------------------------------------------------*/
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
    mavlink_msg_flash_fw_write_packet_pack(0, 0, &tx_msg, resp, 0, 128, buf);
    len += mavlink_msg_to_send_buffer(&tx_buf[len], &tx_msg);
  }
  write_bytes((char *)tx_buf, len);

  return err_code;
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_write_block
     WORK    :
---------------------------------------------------------------------------*/
err_code_t cmd_flash_fw_write_block( uint32_t addr, uint32_t length  )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t buf[256];
  uint8_t resp = 1;


  mavlink_msg_flash_fw_write_block_pack(0, 0, &tx_msg, resp, addr, length);
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


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_read_block
     WORK    :
---------------------------------------------------------------------------*/
#if 0
err_code_t cmd_flash_fw_read_block( uint32_t addr, uint8_t *p_data, uint16_t length )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_flash_fw_read_t  resp_msg;
  uint8_t resp = 1;
  uint16_t received_length;


  mavlink_msg_flash_fw_read_block_pack(0, 0, &tx_msg, resp, addr, length);
  msg_send(0, &tx_msg);



  if( resp == 1 )
  {
    received_length = 0;

    while(1)
    {
      if( msg_get_resp(0, &rx_msg, 3000) == TRUE )
      {
	mavlink_msg_flash_fw_read_decode( &rx_msg, &resp_msg);

	memcpy(&p_data[received_length], resp_msg.data, resp_msg.length);
	received_length += resp_msg.length;

	//printf("recv %d \r\n", received_length);

	if( received_length == length )
	{
	  break;
	}
	else if( received_length > length )
	{
	  err_code = ERR_SIZE_OVER;
	  break;
	}
      }
      else
      {
	err_code = ERR_TIMEOUT;
	break;
      }
    }
  }

  return err_code;
}
#else
err_code_t cmd_flash_fw_read_block( uint32_t addr, uint8_t *p_data, uint16_t length )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_flash_fw_read_packet_t  resp_msg;
  uint8_t resp = 1;


  mavlink_msg_flash_fw_read_block_pack(0, 0, &tx_msg, resp, addr, length);
  msg_send(0, &tx_msg);



  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 100) == TRUE )
    {
      mavlink_msg_flash_fw_read_packet_decode( &rx_msg, &resp_msg);

      memcpy(p_data, resp_msg.data, resp_msg.length);

      if( resp_msg.length > length )
      {
	err_code = ERR_SIZE_OVER;
      }
    }
    else
    {
      err_code = ERR_TIMEOUT;
    }
  }

  return err_code;
}
#endif


/*---------------------------------------------------------------------------
     TITLE   : cmd_flash_fw_verify
     WORK    :
---------------------------------------------------------------------------*/
err_code_t cmd_flash_fw_verify( uint32_t length, uint32_t crc, uint32_t *p_crc_ret )
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 1;


  mavlink_msg_flash_fw_verify_pack(0, 0, &tx_msg, resp, length, crc, param);
  msg_send(0, &tx_msg);


  if( resp == 1 )
  {
    if( msg_get_resp(0, &rx_msg, 500) == TRUE )
    {
      mavlink_msg_ack_decode( &rx_msg, &ack_msg);

      *p_crc_ret = ack_msg.data[3]<<24|ack_msg.data[2]<<16|ack_msg.data[1]<<8|ack_msg.data[0];

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


/*---------------------------------------------------------------------------
     TITLE   : cmd_jump_to_fw
     WORK    :
---------------------------------------------------------------------------*/
err_code_t cmd_jump_to_fw(void)
{
  err_code_t err_code = OK;
  mavlink_message_t tx_msg;
  mavlink_message_t rx_msg;
  mavlink_ack_t     ack_msg;
  uint8_t param[8];
  uint8_t resp = 0;


  mavlink_msg_jump_to_fw_pack(0, 0, &tx_msg, resp, param);
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


/*---------------------------------------------------------------------------
     TITLE   : crc_calc
     WORK    :
---------------------------------------------------------------------------*/
uint32_t crc_calc( uint32_t crc_in, uint8_t data_in )
{

  crc_in  ^= data_in;
  crc_in  += data_in;

  return crc_in;
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_write_err_msg
     WORK    :
---------------------------------------------------------------------------*/
void opencr_ld_write_err_msg( const char *fmt, ...)
{
  int32_t ret = 0;
  va_list arg;
  va_start (arg, fmt);
  int32_t len;

  len = vsnprintf(err_msg_str, 255, fmt, arg);
  va_end (arg);
}


/*---------------------------------------------------------------------------
     TITLE   : opencr_ld_write_err_msg
     WORK    :
---------------------------------------------------------------------------*/
void opencr_ld_print_err_msg(void)
{
  uint32_t len;

  len = strlen(err_msg_str);

  if( len > 0 && len < 500 )
  {
    printf("%s", err_msg_str);
  }
}

