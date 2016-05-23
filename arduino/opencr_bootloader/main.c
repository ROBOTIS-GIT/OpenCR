/*
 * OpenCR BootLoader Firmware.
 *
 * by Baram
 * by PBPH
 * by http://oroca.org
 */

#include "main.h"



void main_init();
void msg_process_vcp(void);


extern void CDC_Write( uint8_t *p_buf, uint32_t length );

int main(void)
{
  uint32_t tTime;
  //uint8_t cnt = 0;
  //uint8_t ch;


  main_init();

  tTime = millis();
  while(1)
  {
    if( millis()-tTime > 500 )
    {
      tTime = millis();
      led_toggle(0);
    }


#if 0
    if( vcp_is_available() )
    {
      ch = vcp_getch();
      vcp_printf("pressed : 0x%02X \r\n", ch);
      //vcp_printf("float test %f\r\n",fvalue);
    }
#else
    msg_process_vcp();
#endif
  }
}


void main_init()
{
  bsp_init();
  hal_init();
}



void msg_process_vcp(void)
{
  BOOL ret;
  uint8_t ch;
  msg_t	msg;

  //if( vcp_is_available() )
  while(vcp_is_available())
  {
    ch = vcp_getch();
    ret = msg_recv( 0, ch, &msg );

    if( ret == TRUE )
    {
      switch( msg.p_msg->msgid )
      {
	case MAVLINK_MSG_ID_READ_VERSION:
	  cmd_read_version(&msg);
	  break;

	case MAVLINK_MSG_ID_FLASH_FW_SEND_BLOCK:
	  cmd_flash_fw_send_blcok(&msg);
	  break;

	case MAVLINK_MSG_ID_FLASH_FW_WRITE_BEGIN:
	  cmd_flash_fw_write_begin(&msg);
	  break;

	case MAVLINK_MSG_ID_FLASH_FW_WRITE_END:
	  cmd_flash_fw_write_end(&msg);
	  break;

	case MAVLINK_MSG_ID_FLASH_FW_WRITE_BLOCK:
	  cmd_flash_fw_write_block(&msg);
	  break;

	case MAVLINK_MSG_ID_FLASH_FW_ERASE:
	  cmd_flash_fw_erase(&msg);
	  break;
      }
    }
  }
}
