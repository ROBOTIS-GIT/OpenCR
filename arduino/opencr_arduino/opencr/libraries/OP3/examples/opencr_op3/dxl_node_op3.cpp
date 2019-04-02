/*
 *  dxl_node_op3.cpp
 *
 *  dynamixel node op3
 *
 *  Created on: 2016. 10. 21.
 *      Author: Baram
 */

#include "dxl.h"
#include "dxl_hw.h"
#include "dxl_hw_op3.h"
#include "dxl_node_op3.h"
#include "dxl_debug.h"
#include <EEPROM.h>



#define RANGE_CHECK(addr,x)            dxl_node_check_range(addr, (uint32_t)&(x), sizeof(x))



static dxl_t dxl_sp;


dxl_mem_op3_t *p_dxl_mem;
dxl_mem_t      mem;



void dxl_node_op3_reset(void);
void dxl_node_op3_factory_reset(void);
void dxl_node_op3_btn_loop(void);


//-- dxl sp driver function
//
dxl_error_t ping(dxl_t *p_dxl);
dxl_error_t read(dxl_t *p_dxl);
dxl_error_t write(dxl_t *p_dxl);
dxl_error_t sync_read(dxl_t *p_dxl);
dxl_error_t sync_write(dxl_t *p_dxl);
dxl_error_t bulk_read(dxl_t *p_dxl);
dxl_error_t bulk_write(dxl_t *p_dxl);


void dxl_process_packet();


static uint8_t dxl_node_read_byte(uint16_t addr);
static void    dxl_node_write_byte(uint16_t addr, uint8_t data);

static BOOL dxl_node_check_range(uint16_t addr, uint32_t addr_ptr, uint8_t length);
void dxl_node_op3_change_baud(void);

static void dxl_node_update_tx_rx_led();

/*---------------------------------------------------------------------------
     TITLE   : dxl_node_op3_init
     WORK    :
---------------------------------------------------------------------------*/
void dxl_node_op3_init(void)
{
  p_dxl_mem = (dxl_mem_op3_t *)&mem.data;


  dxlInit(&dxl_sp, DXL_PACKET_VER_2_0);


  dxl_hw_op3_init();
  dxl_node_op3_reset();

  if( p_dxl_mem->Model_Number != DXL_NODE_OP3_MODLE_NUMBER )
  {
    dxl_node_op3_factory_reset();
    dxl_node_op3_reset();
  }

  if( p_dxl_mem->Firmware_Version != DXL_NODE_OP3_FW_VER )
  {
    p_dxl_mem->Firmware_Version = DXL_NODE_OP3_FW_VER;
    EEPROM[2] = mem.data[2];
  }


  p_dxl_mem->IMU_Control = 0;

  dxl_node_write_byte(26, (0x1F<<0));
  dxl_node_write_byte(27, (0x00<<0));



  dxlSetId(&dxl_sp, p_dxl_mem->ID);
  dxlOpenPort(&dxl_sp, 0, p_dxl_mem->Baud);


  dxlAddInstFunc(&dxl_sp, INST_PING,  ping);
  dxlAddInstFunc(&dxl_sp, INST_READ,  read);
  dxlAddInstFunc(&dxl_sp, INST_WRITE, write);
  dxlAddInstFunc(&dxl_sp, INST_SYNC_READ, sync_read);
  dxlAddInstFunc(&dxl_sp, INST_SYNC_WRITE, sync_write);
  dxlAddInstFunc(&dxl_sp, INST_BULK_READ, bulk_read);
  dxlAddInstFunc(&dxl_sp, INST_BULK_WRITE, bulk_write);

  dxl_debug_init();
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_node_op3_loop
     WORK    :
---------------------------------------------------------------------------*/
void dxl_node_op3_loop(void)
{
  static uint8_t  gyro_cali_state = 0;
  uint8_t i;

  dxl_process_packet();
  dxl_node_update_tx_rx_led();


  dxl_hw_op3_update();


  p_dxl_mem->Acc_X  = dxl_hw_op3_acc_get_x();
  p_dxl_mem->Acc_Y  = dxl_hw_op3_acc_get_y();
  p_dxl_mem->Acc_Z  = dxl_hw_op3_acc_get_z();

  p_dxl_mem->Gyro_X = dxl_hw_op3_gyro_get_x();
  p_dxl_mem->Gyro_Y = dxl_hw_op3_gyro_get_y();
  p_dxl_mem->Gyro_Z = dxl_hw_op3_gyro_get_z();

  p_dxl_mem->Roll   = dxl_hw_op3_get_rpy(0);
  p_dxl_mem->Pitch  = dxl_hw_op3_get_rpy(1);
  p_dxl_mem->Yaw    = dxl_hw_op3_get_rpy(2);


  for(i=0; i<3; i++)
  {
    if(p_dxl_mem->IMU_Control & (1<<i))
    {
      if(dxl_hw_op3_get_cali(i) == 0)
      {
        dxl_hw_op3_start_cali(i);
      }
      if(dxl_hw_op3_get_cali(i) < 0)
      {
        p_dxl_mem->IMU_Control &= ~(1<<i);
        dxl_hw_op3_clear_cali(i);

        p_dxl_mem->Roll_Offset  = dxl_hw_op3_get_offset(0) * 10.;
        p_dxl_mem->Pitch_Offset = dxl_hw_op3_get_offset(1) * 10.;
        p_dxl_mem->Yaw_Offset   = dxl_hw_op3_get_offset(2) * 10.;


        EEPROM[18] = mem.data[18];
        EEPROM[19] = mem.data[19];
        EEPROM[20] = mem.data[20];
        EEPROM[21] = mem.data[21];
      }
    }
  }

  if(p_dxl_mem->IMU_Control & (1<<3))
  {
    if(gyro_cali_state == 0)
    {
      dxl_hw_op3_start_gyro_cali();
      gyro_cali_state = 1;
    }
    else
    {
      if(dxl_hw_op3_get_gyro_cali_done() == true)
      {
        p_dxl_mem->IMU_Control &= ~(1<<3);
        gyro_cali_state = 0;
      }
    }

  }


  dxl_node_op3_btn_loop();

  dxl_debug_loop();
}


void dxl_process_packet()
{
  static uint8_t process_state = 0;
  dxl_error_t dxl_ret;
  static uint32_t pre_time;


  switch (process_state)
  {
    //-- INST
    //
    case DXL_PROCESS_INST:
      dxl_ret = dxlRxPacket(&dxl_sp);

      if (dxl_ret == DXL_RET_RX_INST)
      {
        dxl_ret = dxlProcessInst(&dxl_sp);

        if (dxl_ret == DXL_RET_PROCESS_BROAD_PING)
        {
          dxl_sp.current_id = 1;
          pre_time = micros();
          process_state = DXL_PROCESS_BROAD_PING;
        }

        if (dxl_ret == DXL_RET_PROCESS_BROAD_READ)
        {
          pre_time = micros();
          process_state = DXL_PROCESS_BROAD_READ;
        }
      }
      break;


    //-- BROAD_PING
    //
    case DXL_PROCESS_BROAD_PING:
      dxl_ret = dxlRxPacket(&dxl_sp);

      if (dxl_ret == DXL_RET_RX_STATUS)
      {
        dxl_sp.current_id = dxl_sp.rx.id + 1;
      }
      else
      {
        if (micros()-pre_time >= 3000)
        {
          pre_time = micros();
          dxl_sp.current_id++;
        }
      }

      if (dxl_sp.current_id == dxl_sp.id)
      {
        dxlTxPacket(&dxl_sp);
        process_state = DXL_PROCESS_INST;
      }
      break;

    //-- BROAD_READ
    //
    case DXL_PROCESS_BROAD_READ:
      dxl_ret = dxlRxPacket(&dxl_sp);

      if (dxl_ret == DXL_RET_RX_STATUS)
      {
        pre_time = micros();
        if (dxl_sp.pre_id == dxl_sp.rx.id)
        {
          dxlTxPacket(&dxl_sp);
          process_state = DXL_PROCESS_INST;
          //Serial.println(" Bulk Read out");
        }
        else
        {
          //Serial.print(" in ");
          //Serial.println(dxl_sp.rx.id, HEX);
        }
      }
      else
      {
        if (micros()-pre_time >= 50000)
        {
          process_state = DXL_PROCESS_INST;
          //Serial.println(" Bulk Read timeout");
        }
      }
      break;


    default:
      process_state = DXL_PROCESS_INST;
      break;
  }

}



/*---------------------------------------------------------------------------
     TITLE   : dxl_node_op3_btn_loop
     WORK    :
---------------------------------------------------------------------------*/
void dxl_node_op3_btn_loop(void)
{
  static uint8_t  btn_state = 0;
  static uint32_t btn_time  = 0;


  switch( btn_state )
  {
    case 0:
      if( dxl_hw_op3_button_read(PIN_BUTTON_S4) )
      {
        btn_time  = millis();
        btn_state = 1;
      }
      break;

    case 1:
      if( !dxl_hw_op3_button_read(PIN_BUTTON_S4) ) btn_state = 0;
      if( (millis()-btn_time) > 100 )
      {
        dxl_node_write_byte(24, 0);
        btn_time  = millis();
        btn_state = 2;
      }
      break;

    case 2:
      if( !dxl_hw_op3_button_read(PIN_BUTTON_S4) ) btn_state = 0;
      break;

    default:
      break;
  }
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_node_op3_reset
     WORK    :
---------------------------------------------------------------------------*/
void dxl_node_op3_reset(void)
{
  uint16_t i;


  memset(&mem, 0x00, sizeof(dxl_mem_t));

  mem.attr[0]   = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RO;
  mem.attr[1]   = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RO;
  mem.attr[2]   = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RO;
  mem.attr[3]   = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  mem.attr[4]   = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  mem.attr[5]   = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  mem.attr[16]  = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  mem.attr[18]  = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  mem.attr[19]  = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  mem.attr[20]  = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  mem.attr[21]  = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  mem.attr[22]  = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  mem.attr[23]  = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;

  mem.attr[24]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RW;
  mem.attr[25]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RW;
  mem.attr[26]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RW;
  mem.attr[27]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RW;
  mem.attr[28]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RW;
  mem.attr[29]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RW;
  mem.attr[30]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[31]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[32]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[33]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[34]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[35]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[36]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[37]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[38]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[39]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[40]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[41]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[42]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[43]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[44]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[45]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[46]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[47]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[48]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[49]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  mem.attr[50]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RW;


  // EEPROM Load
  for( i=0; i<sizeof(dxl_mem_op3_t); i++ )
  {
    if( mem.attr[i]&DXL_MEM_ATTR_EEPROM )
    {
      mem.data[i] = EEPROM[i];
    }
  }

  dxl_hw_op3_set_offset(0, (float)p_dxl_mem->Roll_Offset/10.);
  dxl_hw_op3_set_offset(1, (float)p_dxl_mem->Pitch_Offset/10.);
  //dxl_hw_op3_set_offset(2, (float)p_dxl_mem->Yaw_Offset/10.);
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_node_op3_factory_reset
     WORK    :
---------------------------------------------------------------------------*/
void dxl_node_op3_factory_reset(void)
{
  uint16_t i;


  p_dxl_mem->Model_Number         = DXL_NODE_OP3_MODLE_NUMBER;
  p_dxl_mem->Firmware_Version     = DXL_NODE_OP3_FW_VER;
  p_dxl_mem->ID                   = DXL_NODE_OP3_ID;
  p_dxl_mem->Baud                 = DXL_NODE_OP3_BAUD;
  p_dxl_mem->Return_Delay_Time    = 0;
  p_dxl_mem->Status_Return_Level  = 2;
  p_dxl_mem->Roll_Offset          = 0;
  p_dxl_mem->Pitch_Offset         = 0;
  p_dxl_mem->Yaw_Offset           = 0;

  // EEPROM Save
  for( i=0; i<sizeof(dxl_mem_op3_t); i++ )
  {
    if( mem.attr[i]&DXL_MEM_ATTR_EEPROM )
    {
      EEPROM[i] = mem.data[i];
    }
  }

  dxl_node_op3_reset();
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_node_op3_change_baud
     WORK    :
---------------------------------------------------------------------------*/
void dxl_node_op3_change_baud(void)
{
  dxlOpenPort(&dxl_sp, 0, p_dxl_mem->Baud);
}

/*---------------------------------------------------------------------------
     TITLE   : dxl_node_read_byte
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_node_read_byte(uint16_t addr)
{
  if( RANGE_CHECK(addr, p_dxl_mem->Button) )
  {
    p_dxl_mem->Button  = dxl_hw_op3_button_read(PIN_BUTTON_S1)<<0;
    p_dxl_mem->Button |= dxl_hw_op3_button_read(PIN_BUTTON_S2)<<1;
    p_dxl_mem->Button |= dxl_hw_op3_button_read(PIN_BUTTON_S3)<<2;
    p_dxl_mem->Button |= dxl_hw_op3_button_read(PIN_BUTTON_S4)<<3;
  }

  if( RANGE_CHECK(addr, p_dxl_mem->Voltage) )
  {
    p_dxl_mem->Voltage  = dxl_hw_op3_voltage_read();
  }


  return mem.data[addr];
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_node_write_byte
     WORK    :
---------------------------------------------------------------------------*/
void dxl_node_write_byte(uint16_t addr, uint8_t data)
{
  uint8_t pwm_value[3];


  mem.data[addr] = data;



  if( RANGE_CHECK(addr, p_dxl_mem->Dynamixel_Power) )
  {
    if( p_dxl_mem->Dynamixel_Power == 1 ) dxl_hw_power_enable();
    else                                  dxl_hw_power_disable();
  }

  if( RANGE_CHECK(addr, p_dxl_mem->LED) )
  {
    if( data & (1<<0) ) dxl_hw_op3_led_set(PIN_LED_1, 0);
    else                dxl_hw_op3_led_set(PIN_LED_1, 1);
    if( data & (1<<1) ) dxl_hw_op3_led_set(PIN_LED_2, 0);
    else                dxl_hw_op3_led_set(PIN_LED_2, 1);
    if( data & (1<<2) ) dxl_hw_op3_led_set(PIN_LED_3, 0);
    else                dxl_hw_op3_led_set(PIN_LED_3, 1);
  }

  if( RANGE_CHECK(addr, p_dxl_mem->LED_RGB) )
  {
    pwm_value[0] = (p_dxl_mem->LED_RGB>> 0) & 0x1F;
    pwm_value[1] = (p_dxl_mem->LED_RGB>> 5) & 0x1F;
    pwm_value[2] = (p_dxl_mem->LED_RGB>>10) & 0x1F;

    dxl_hw_op3_led_pwm(PIN_LED_R, pwm_value[0]);
    dxl_hw_op3_led_pwm(PIN_LED_G, pwm_value[1]);
    dxl_hw_op3_led_pwm(PIN_LED_B, pwm_value[2]);
  }

  if( RANGE_CHECK(addr, p_dxl_mem->Baud) )
  {
    dxl_node_op3_change_baud();
  }

  if( RANGE_CHECK(addr, p_dxl_mem->Buzzer) )
  {
    if( p_dxl_mem->Buzzer > 0 ) tone(BDPIN_BUZZER, p_dxl_mem->Buzzer);
    else                        noTone(BDPIN_BUZZER);
  }
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_node_check_range
     WORK    :
---------------------------------------------------------------------------*/
BOOL dxl_node_check_range(uint16_t addr, uint32_t addr_ptr, uint8_t length)
{
  BOOL ret = FALSE;
  uint32_t addr_offset;

  addr_offset = addr_ptr - (uint32_t)p_dxl_mem;

  if( addr >= (addr_offset+length-1) && addr < (addr_offset+length) )
  {
    ret = TRUE;
  }


  return ret;
}


/*---------------------------------------------------------------------------
     dxl sp driver
---------------------------------------------------------------------------*/
void processRead(uint16_t addr, uint8_t *p_data, uint16_t length)
{
  uint32_t i;


  for( i=0; i<length; i++ )
  {
    p_data[i] = dxl_node_read_byte(addr);
    addr++;
  }
}

void processWrite(uint16_t addr, uint8_t *p_data, uint16_t length)
{
  uint32_t i;


  for( i=0; i<length; i++ )
  {
    if( mem.attr[addr]&DXL_MEM_ATTR_WO || mem.attr[addr]&DXL_MEM_ATTR_RW )
    {
      dxl_node_write_byte(addr, p_data[i]);
      if( mem.attr[addr]&DXL_MEM_ATTR_EEPROM )
      {
        EEPROM[addr] = mem.data[addr];
      }
    }
    addr++;
  }
}




/*---------------------------------------------------------------------------
     TITLE   : ping
     WORK    :
---------------------------------------------------------------------------*/
dxl_error_t ping(dxl_t *p_dxl)
{
  dxl_error_t ret = DXL_RET_OK;
  uint8_t data[3];



  data[0] = (p_dxl_mem->Model_Number>>0) & 0xFF;
  data[1] = (p_dxl_mem->Model_Number>>8) & 0xFF;
  data[2] = p_dxl_mem->Firmware_Version;

  if (p_dxl->rx.id == DXL_GLOBAL_ID)
  {
    ret = dxlMakePacketStatus(p_dxl, p_dxl->id, 0, data, 3);

    if (ret == DXL_RET_OK)
    {
      ret = DXL_RET_PROCESS_BROAD_PING;
    }
  }
  else
  {
    ret = dxlTxPacketStatus(p_dxl, p_dxl->id, 0, data, 3);
  }


  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : read
     WORK    :
---------------------------------------------------------------------------*/
dxl_error_t read(dxl_t *p_dxl)
{
  dxl_error_t ret = DXL_RET_OK;
  uint16_t addr;
  uint16_t length;
  uint8_t data[DXL_MAX_BUFFER];


  if (p_dxl->rx.id == DXL_GLOBAL_ID || p_dxl->rx.param_length != 4)
  {
    return DXL_RET_EMPTY;
  }


  addr   = (p_dxl->rx.p_param[1]<<8) | p_dxl->rx.p_param[0];
  length = (p_dxl->rx.p_param[3]<<8) | p_dxl->rx.p_param[2];


  if (addr >= sizeof(dxl_mem_op3_t))
  {
    dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, NULL, 0);
    return DXL_RET_ERROR_LENGTH;
  }
  if( length > DXL_MAX_BUFFER - 10 )
  {
    dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, NULL, 0);
    return DXL_RET_ERROR_LENGTH;
  }

  processRead(addr, data, length);


  ret = dxlTxPacketStatus(p_dxl, p_dxl->id, 0, data, length);

  //Serial.println(" Read");

  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : write
     WORK    :
---------------------------------------------------------------------------*/
dxl_error_t write(dxl_t *p_dxl)
{
  dxl_error_t ret = DXL_RET_OK;
  uint16_t addr;
  uint16_t length;
  uint8_t  *p_data;


  if (p_dxl->rx.id == DXL_GLOBAL_ID)
  {
    return DXL_RET_EMPTY;
  }

  addr   = (p_dxl->rx.p_param[1]<<8) | p_dxl->rx.p_param[0];
  p_data = &p_dxl->rx.p_param[2];

  if (p_dxl->rx.param_length > 2 )
  {
    length = p_dxl->rx.param_length - 2;
  }
  else
  {
    dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, NULL, 0);
    return DXL_RET_ERROR_LENGTH;
  }

  if (addr >= sizeof(dxl_mem_op3_t))
  {
    dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, NULL, 0);
    return DXL_RET_ERROR_LENGTH;
  }
  if( length > DXL_MAX_BUFFER - 10 )
  {
    dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, NULL, 0);
    return DXL_RET_ERROR_LENGTH;
  }


  dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_NONE, NULL, 0);


  processWrite(addr, p_data, length);

  //Serial.println(" write");
  return ret;
}


dxl_error_t sync_read(dxl_t *p_dxl)
{
  dxl_error_t ret = DXL_RET_OK;
  uint16_t addr;
  uint16_t length;
  uint8_t  *p_data;
  uint16_t i;
  uint16_t rx_id_cnt;
  uint8_t data[DXL_MAX_BUFFER];


  if (p_dxl->rx.id != DXL_GLOBAL_ID)
  {
    return DXL_RET_EMPTY;
  }

  addr      = (p_dxl->rx.p_param[1]<<8) | p_dxl->rx.p_param[0];
  length    = (p_dxl->rx.p_param[3]<<8) | p_dxl->rx.p_param[2];
  p_data    = &p_dxl->rx.p_param[4];
  rx_id_cnt = p_dxl->rx.param_length - 4;


  if (p_dxl->rx.param_length < (5) || rx_id_cnt > 255)
  {
    //dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, NULL, 0);
    return DXL_RET_ERROR_LENGTH;
  }
  if (addr >= sizeof(dxl_mem_op3_t) || (addr+length) > sizeof(dxl_mem_op3_t))
  {
    //dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, NULL, 0);
    return DXL_RET_ERROR_LENGTH;
  }




  p_dxl->pre_id     = 0xFF;
  p_dxl->current_id = 0xFF;

  for (i=0; i<rx_id_cnt; i++)
  {
    if (p_data[i] == p_dxl->id)
    {
      p_dxl->current_id = p_dxl->id;
      break;
    }

    p_dxl->pre_id = p_data[i];
  }


  if (p_dxl->current_id == p_dxl->id)
  {
    processRead(addr, data, length);


    if (p_dxl->pre_id == 0xFF)
    {
      ret = dxlTxPacketStatus(p_dxl, p_dxl->id, 0, data, length);
    }
    else
    {
      ret = dxlMakePacketStatus(p_dxl, p_dxl->id, 0, data, length);
      if (ret == DXL_RET_OK)
      {
        ret = DXL_RET_PROCESS_BROAD_READ;
      }
    }
  }

  //Serial.println(" Sync Read");

  return ret;
}


dxl_error_t sync_write(dxl_t *p_dxl)
{
  dxl_error_t ret = DXL_RET_OK;
  uint16_t addr;
  uint16_t length;
  uint8_t  *p_data;
  uint16_t remain_length;
  uint16_t index;

  if (p_dxl->rx.id != DXL_GLOBAL_ID)
  {
    //Serial.println(" Sync Write Err 0");
    return DXL_RET_EMPTY;
  }

  addr   = (p_dxl->rx.p_param[1]<<8) | p_dxl->rx.p_param[0];
  length = (p_dxl->rx.p_param[3]<<8) | p_dxl->rx.p_param[2];



  //Serial.print(" Sync Write in : ");
  //Serial.print(addr);
  //Serial.print(" ");
  //Serial.println(length);

  if (p_dxl->rx.param_length < (4+length+1))
  {
    //dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, NULL, 0);
    //Serial.println(" Sync Write Err 1");
    return DXL_RET_ERROR_LENGTH;
  }
  if (addr >= sizeof(dxl_mem_op3_t) || (addr+length) > sizeof(dxl_mem_op3_t))
  {
    //dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, NULL, 0);
    //Serial.println(" Sync Write Err 2");
    return DXL_RET_ERROR_LENGTH;
  }




  index = 4;
  while(1)
  {
    p_data = &p_dxl->rx.p_param[index];
    remain_length = p_dxl->rx.param_length - index;


    if (remain_length < (length+1))
    {
      break;
    }
    else
    {
      if (p_data[0] == p_dxl->id)
      {
        processWrite(addr, &p_data[1], length);
        //Serial.println(" Sync Write out");
        break;
      }

      index += length + 1;
    }
  }

  return ret;
}


dxl_error_t bulk_read(dxl_t *p_dxl)
{
  dxl_error_t ret = DXL_RET_OK;
  uint16_t addr;
  uint16_t length;
  uint8_t  *p_data;
  uint16_t i;
  uint16_t rx_id_cnt;
  uint8_t data[DXL_MAX_BUFFER];



  if (p_dxl->rx.id != DXL_GLOBAL_ID)
  {
    return DXL_RET_EMPTY;
  }


  rx_id_cnt = p_dxl->rx.param_length / 5;


  if (p_dxl->rx.param_length < 5 || (p_dxl->rx.param_length%5) != 0)
  {
    //Serial.print(" DXL_RET_ERROR_LENGTH ");
    return DXL_RET_ERROR_LENGTH;
  }


  p_dxl->pre_id     = 0xFF;
  p_dxl->current_id = 0xFF;

  for (i=0; i<rx_id_cnt; i++)
  {
    p_data = &p_dxl->rx.p_param[i*5];
    addr   = (p_data[2]<<8) | p_data[1];
    length = (p_data[4]<<8) | p_data[3];


    //Serial.print(" bulk in id ");
    //Serial.println(p_data[0], HEX);

    if (p_data[0] == p_dxl->id)
    {
      p_dxl->current_id = p_dxl->id;
      break;
    }
    p_dxl->pre_id = p_data[0];
  }


  if (p_dxl->current_id == p_dxl->id)
  {
    if (addr >= sizeof(dxl_mem_op3_t) || (addr+length) > sizeof(dxl_mem_op3_t))
    {
      return DXL_RET_ERROR_LENGTH;
    }


    processRead(addr, data, length);


    if (p_dxl->pre_id == 0xFF)
    {
      ret = dxlTxPacketStatus(p_dxl, p_dxl->id, 0, data, length);
    }
    else
    {
      ret = dxlMakePacketStatus(p_dxl, p_dxl->id, 0, data, length);
      if (ret == DXL_RET_OK)
      {
        ret = DXL_RET_PROCESS_BROAD_READ;
      }
    }
  }

  return ret;
}


dxl_error_t bulk_write(dxl_t *p_dxl)
{
  dxl_error_t ret = DXL_RET_OK;
  uint16_t addr;
  uint16_t length;
  uint8_t  *p_data;
  uint16_t index;

  if (p_dxl->rx.id != DXL_GLOBAL_ID)
  {
    return DXL_RET_EMPTY;
  }


  index = 0;
  while(1)
  {
    p_data = &p_dxl->rx.p_param[index];
    addr   = (p_data[2]<<8) | p_data[1];
    length = (p_data[4]<<8) | p_data[3];

    index += 5;

    if (p_dxl->rx.param_length < (index + length))
    {
      break;
    }

    if (p_data[0] == p_dxl->id)
    {
      if (addr >= sizeof(dxl_mem_op3_t) || (addr+length) > sizeof(dxl_mem_op3_t))
      {
        return DXL_RET_ERROR_LENGTH;
      }
      processWrite(addr, &p_dxl->rx.p_param[index], length);

      //Serial.print(addr);
      //Serial.print(" ");
      //Serial.print(length);
      //Serial.print(" ");
      //Serial.println(" bulk write ");
      break;
    }
    index += length;
  }

  return ret;
}


extern uint32_t tx_led_count, rx_led_count;

static void dxl_node_update_tx_rx_led()
{
  static uint32_t tx_led_update_time = millis();
  static uint32_t rx_led_update_time = millis();

  if( (millis()-tx_led_update_time) > 50 )
  {
    tx_led_update_time = millis();

    if( tx_led_count )
    {
      digitalWriteFast(DXL_LED_TX, !digitalReadFast(DXL_LED_TX));
      tx_led_count--;
    }
    else
    {
      digitalWriteFast(DXL_LED_TX, HIGH);
    }
  }

  if( (millis()-rx_led_update_time) > 50 )
  {
    rx_led_update_time = millis();

    if( rx_led_count )
    {
      digitalWriteFast(DXL_LED_RX, !digitalReadFast(DXL_LED_RX));
      rx_led_count--;
    }
    else
    {
      digitalWriteFast(DXL_LED_RX, HIGH);
    }
  }
}