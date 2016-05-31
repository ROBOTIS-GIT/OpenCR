/*
 * OpenCR BootLoader Firmware.
 *
 * by Baram
 * by PBHP
 * by http://oroca.org
 */

#include "main.h"



void main_init();
void msg_process_vcp(void);


void stop()
{
  while(1);
}

int main(void)
{
  uint32_t tTime;

  main_init();


  tTime = millis();
  while(1)
  {
    if( millis()-tTime > 1000 )
    {
      tTime = millis();
      led_toggle(0);
    }


#if 1
    uint8_t ch;
    static uint32_t cnt = 0;


    if( vcp_is_available() )
    {
      ch = vcp_getch();
      vcp_printf("pressed : 0x%02X \r\n", ch);
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

	case MAVLINK_MSG_ID_READ_BOARD_NAME:
	  cmd_read_board_name(&msg);
	  break;

	case MAVLINK_MSG_ID_READ_TAG:
	  cmd_read_tag(&msg);
	  break;

	case MAVLINK_MSG_ID_FLASH_FW_WRITE_PACKET:
	  cmd_flash_fw_write_packet(&msg);
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

	case MAVLINK_MSG_ID_FLASH_FW_VERIFY:
	  cmd_flash_fw_verify(&msg);
	  break;

	case MAVLINK_MSG_ID_FLASH_FW_READ_BLOCK:
	  cmd_flash_fw_read_block(&msg);
	  break;

	case MAVLINK_MSG_ID_JUMP_TO_FW:
	  cmd_jump_to_fw(&msg);
	  break;

	default:
	  cmd_send_error(&msg, ERR_INVALID_CMD);
	  break;
      }
    }
  }
}
