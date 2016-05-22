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


ser_handler stm32_ser_id = ( ser_handler )-1;


int opencr_ld_init( const char *portname, u32 baud );
void opencr_ld_msg_process(void);
void cmd_version( msg_t *p_msg );



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
  int  i;


  // Open port
  if( ( stm32_ser_id = ser_open( portname ) ) == ( ser_handler )-1 )
  {
    printf("Fail to open port 1\n");
    return -1;
  }

  // Setup port
  //ser_setupEx( stm32_ser_id, baud, SER_DATABITS_8, SER_PARITY_NONE, SER_STOPBITS_1, 0 );
  ser_setup( stm32_ser_id, baud, SER_DATABITS_8, SER_PARITY_NONE, SER_STOPBITS_1 );


  ser_set_timeout_ms( stm32_ser_id, SER_NO_TIMEOUT );
  while( read_byte() != -1 );
  ser_set_timeout_ms( stm32_ser_id, 2000000 );


  mavlink_message_t mav_msg;
  uint8_t buf[1024];

  mavlink_msg_version_pack(0, 0, &mav_msg, 1, 9, (const uint8_t*) "V160521R1");

  msg_send(0, &mav_msg);

  printf("send cmd\r\n");
  delay_ms(100);
  opencr_ld_msg_process();

  ser_close( stm32_ser_id );




  return TRUE;
}


void opencr_ld_msg_process(void)
{
  BOOL ret;
  int  ch_ret;
  uint8_t ch;
  msg_t	msg;
  int index = 0;
  int retry = 50;


  ser_set_timeout_ms( stm32_ser_id, 2000000 );

  while(1)
  {
    ch_ret = read_byte();

    if( ch_ret < 0 )
    {
      if( retry-- == 0 )
      {
	printf("no data \r\n");
	break;
      }
      else
      {
	continue;
      }
    }
    else
    {
      ch = (uint8_t)(ch_ret);
    }

    ret = msg_recv( 0, ch, &msg );

    if( ret == TRUE )
    {
      switch( msg.p_msg->msgid )
      {
	case MAVLINK_MSG_ID_VERSION:
	  cmd_version(&msg);
	  break;
      }
      break;
    }
  }
}


void cmd_version( msg_t *p_msg )
{
  mavlink_message_t mav_msg;
  uint8_t buf[1024];


  mavlink_version_t version_cmd;
  mavlink_msg_version_decode( p_msg->p_msg, &version_cmd);

  version_cmd.ver_stirng[version_cmd.ver_length] = 0;
  printf("Version : %s\r\n", version_cmd.ver_stirng);
}
