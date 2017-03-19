/*
 *  dxl_node_op3.cpp
 *
 *  dynamixel node op3
 *
 *  Created on: 2016. 10. 21.
 *      Author: Baram
 */

#include "dxl_hw.h"
#include "dxl_hw_op3.h"
#include "dxl_node_op3.h"
#include "dxl_debug.h"
#include <EEPROM.h>



#define RANGE_CHECK(addr,x)            dxl_node_check_range(addr, (uint32_t)&(x), sizeof(x))


static dxl_sp_t  dxl_sp;

dxl_mem_op3_t *p_dxl_mem;




void dxl_node_op3_reset(void);
void dxl_node_op3_factory_reset(void);
void dxl_node_op3_btn_loop(void);



//-- dxl sp driver function
//
static void make_basic_status_packet(uint8_t error, uint32_t send_time);
static void ping(dxl_inst_packet_t *p_inst_packet);
static void read(dxl_inst_packet_t *p_inst_packet);
static void write(dxl_inst_packet_t *p_inst_packet);
static void reg_write(dxl_inst_packet_t *p_inst_packet);
static void action(dxl_inst_packet_t *p_inst_packet);
static void factory_reset(dxl_inst_packet_t *p_inst_packet);
static void reboot(dxl_inst_packet_t *p_inst_packet);
static void sync_read(dxl_inst_packet_t *p_inst_packet);
static void sync_write(dxl_inst_packet_t *p_inst_packet);
static void bulk_read(dxl_inst_packet_t *p_inst_packet);
static void bulk_write(dxl_inst_packet_t *p_inst_packet);


static uint8_t dxl_node_read_byte(uint16_t addr);
static void    dxl_node_write_byte(uint16_t addr, uint8_t data);

static BOOL dxl_node_check_range(uint16_t addr, uint32_t addr_ptr, uint8_t length);
void dxl_node_op3_change_baud(void);


/*---------------------------------------------------------------------------
     TITLE   : dxl_node_op3_init
     WORK    :
---------------------------------------------------------------------------*/
void dxl_node_op3_init(void)
{
  uint16_t i;


  p_dxl_mem = (dxl_mem_op3_t *)&dxl_sp.mem.data;

  dxl_sp.inst_func.ping           = ping;
  dxl_sp.inst_func.read           = read;
  dxl_sp.inst_func.write          = write;
  dxl_sp.inst_func.reg_write      = reg_write;
  dxl_sp.inst_func.action         = action;
  dxl_sp.inst_func.factory_reset  = factory_reset;
  dxl_sp.inst_func.reboot         = reboot;
  dxl_sp.inst_func.sync_read      = sync_read;
  dxl_sp.inst_func.sync_write     = sync_write;
  dxl_sp.inst_func.bulk_read      = bulk_read;
  dxl_sp.inst_func.bulk_write     = bulk_write;

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
    EEPROM[2] = dxl_sp.mem.data[2];
  }

  dxl_sp.tx_status_packet.run_flag = 0;
  dxl_sp.tx_status_packet.run_func = NULL;

  p_dxl_mem->IMU_Control = 0;

  dxl_node_write_byte(26, (0x1F<<0));
  dxl_node_write_byte(27, (0x00<<0));

  dxl_sp_init(&dxl_sp);
  dxl_sp_begin(p_dxl_mem->Baud);


  dxl_debug_init();
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_node_op3_loop
     WORK    :
---------------------------------------------------------------------------*/
void dxl_node_op3_loop(void)
{
  static uint32_t tTime[16];
  static uint8_t  gyro_cali_state = 0;
  uint8_t ret;
  uint8_t i;
  uint8_t ch;


  dxl_sp_rx_update(&dxl_sp);
  dxl_sp_tx_update(&dxl_sp);

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


        EEPROM[18] = dxl_sp.mem.data[18];
        EEPROM[19] = dxl_sp.mem.data[19];
        EEPROM[20] = dxl_sp.mem.data[20];
        EEPROM[21] = dxl_sp.mem.data[21];
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


  memset(&dxl_sp.mem, 0x00, sizeof(dxl_mem_t));

  dxl_sp.mem.attr[0]   = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[1]   = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[2]   = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[3]   = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[4]   = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[5]   = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[16]  = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[18]  = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[19]  = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[20]  = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[21]  = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[22]  = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[23]  = DXL_MEM_ATTR_EEPROM | DXL_MEM_ATTR_RW;

  dxl_sp.mem.attr[24]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[25]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[26]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[27]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[28]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[29]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RW;
  dxl_sp.mem.attr[30]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[31]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[32]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[33]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[34]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[35]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[36]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[37]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[38]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[39]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[40]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[41]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[42]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[43]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[44]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[45]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[46]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[47]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[48]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[49]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RO;
  dxl_sp.mem.attr[50]  = DXL_MEM_ATTR_RAM    | DXL_MEM_ATTR_RW;


  // EEPROM Load
  for( i=0; i<sizeof(dxl_mem_op3_t); i++ )
  {
    if( dxl_sp.mem.attr[i]&DXL_MEM_ATTR_EEPROM )
    {
      dxl_sp.mem.data[i] = EEPROM[i];
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
    if( dxl_sp.mem.attr[i]&DXL_MEM_ATTR_EEPROM )
    {
      EEPROM[i] = dxl_sp.mem.data[i];
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
  dxl_sp_init(&dxl_sp);
  dxl_sp_begin(p_dxl_mem->Baud);
}

/*---------------------------------------------------------------------------
     TITLE   : dxl_node_read_byte
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_node_read_byte(uint16_t addr)
{
  uint8_t data;


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


  return dxl_sp.mem.data[addr];
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_node_write_byte
     WORK    :
---------------------------------------------------------------------------*/
void dxl_node_write_byte(uint16_t addr, uint8_t data)
{
  uint8_t pwm_value[3];


  dxl_sp.mem.data[addr] = data;



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
    dxl_sp.tx_status_packet.run_flag = 1;
    dxl_sp.tx_status_packet.run_func = dxl_node_op3_change_baud;
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
  uint16_t i;

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



/*---------------------------------------------------------------------------
     TITLE   : make_basic_status_packet
     WORK    :
---------------------------------------------------------------------------*/
void make_basic_status_packet(uint8_t error, uint32_t send_time)
{
  dxl_tx_status_packet_t *p_tx_status_packet;


  p_tx_status_packet = &dxl_sp.tx_status_packet;


  p_tx_status_packet->packet.id             = p_dxl_mem->ID;
  p_tx_status_packet->packet.inst           = DXL_INST_STATUS;
  p_tx_status_packet->packet.error          = error;
  p_tx_status_packet->packet.param_length   = 0;
  p_tx_status_packet->packet.packet_length  = p_tx_status_packet->packet.param_length + 4;

  p_tx_status_packet->state     = TX_STATUS_STATE_SEND;
  p_tx_status_packet->prev_time = micros();
  p_tx_status_packet->send_time = send_time;
}


/*---------------------------------------------------------------------------
     TITLE   : ping
     WORK    :
---------------------------------------------------------------------------*/
void ping(dxl_inst_packet_t *p_inst_packet)
{
  uint8_t  error = 0x00;
  uint16_t index = 0;
  uint32_t send_time;
  dxl_tx_status_packet_t *p_tx_status_packet;


  #if _USE_DEBUG_LOG_INST_FUNC == 1
  Serial.print("ping ");
  Serial.print(p_inst_packet->id, HEX);
  Serial.print(" ");
  Serial.println(p_dxl_mem->ID, HEX);
  #endif


  if     ( p_inst_packet->id == DXL_ID_BROADCAST_ID ) send_time = p_dxl_mem->ID*100;
  else if( p_inst_packet->id == p_dxl_mem->ID       ) send_time = 0;
  else                                                return;


  p_tx_status_packet = &dxl_sp.tx_status_packet;


  p_tx_status_packet->packet.id             = p_dxl_mem->ID;
  p_tx_status_packet->packet.inst           = DXL_INST_STATUS;
  p_tx_status_packet->packet.error          = error;
  index = 0;
  p_tx_status_packet->packet.param[index++] = (p_dxl_mem->Model_Number>>0) & 0xFF;
  p_tx_status_packet->packet.param[index++] = (p_dxl_mem->Model_Number>>8) & 0xFF;
  p_tx_status_packet->packet.param[index++] = p_dxl_mem->Firmware_Version;
  p_tx_status_packet->packet.param_length   = index;
  p_tx_status_packet->packet.packet_length  = p_tx_status_packet->packet.param_length + 4;

  p_tx_status_packet->state     = TX_STATUS_STATE_SEND;
  p_tx_status_packet->prev_time = micros();
  p_tx_status_packet->send_time = send_time;
}


/*---------------------------------------------------------------------------
     TITLE   : read
     WORK    :
---------------------------------------------------------------------------*/
void read(dxl_inst_packet_t *p_inst_packet)
{
  uint8_t  error = 0x00;
  uint16_t index = 0;
  uint32_t send_time;
  dxl_tx_status_packet_t *p_tx_status_packet;

  uint16_t i;
  uint16_t addr;
  uint16_t length = 0;

  #if _USE_DEBUG_LOG_INST_FUNC == 1
  Serial.print("read ");
  Serial.println(p_inst_packet->id, HEX);
  #endif

  p_tx_status_packet = &dxl_sp.tx_status_packet;


  if     ( p_inst_packet->id == DXL_ID_BROADCAST_ID ) send_time = p_dxl_mem->ID*1000;
  else if( p_inst_packet->id == p_dxl_mem->ID       ) send_time = 0;
  else                                                return;


  addr   = (p_inst_packet->param[1]<<8) | (p_inst_packet->param[0]<<0);
  length = (p_inst_packet->param[3]<<8) | (p_inst_packet->param[2]<<0);

  if( addr >= sizeof(dxl_mem_op3_t) )
  {
    error = DXL_ERR_DATA_LENGTH;
  }
  if( length > DXL_BUF_LENGTH )
  {
    error = DXL_ERR_DATA_LENGTH;
  }
  dxl_sp_set_step(1, 1);


  if( error == 0x00 )
  {
    dxl_sp_set_step(1, 2);
    index = 0;
    for( i=0; i<length; i++ )
    {
      p_tx_status_packet->packet.param[index++] = dxl_node_read_byte(addr);
      addr++;
    }
  }
  dxl_sp_set_step(1, 3);
  p_tx_status_packet->packet.param_length  = index;
  p_tx_status_packet->packet.packet_length = p_tx_status_packet->packet.param_length + 4;

  // make status packet

  p_tx_status_packet->packet.id             = p_dxl_mem->ID;
  p_tx_status_packet->packet.inst           = DXL_INST_STATUS;
  p_tx_status_packet->packet.error          = error;

  p_tx_status_packet->state     = TX_STATUS_STATE_SEND;
  p_tx_status_packet->prev_time = micros();
  p_tx_status_packet->send_time = send_time;
}


/*---------------------------------------------------------------------------
     TITLE   : write
     WORK    :
---------------------------------------------------------------------------*/
void write(dxl_inst_packet_t *p_inst_packet)
{
  uint8_t  error = 0x00;
  uint16_t index = 0;
  uint32_t send_time;
  dxl_tx_status_packet_t *p_tx_status_packet;

  uint16_t i;
  uint16_t addr;
  uint16_t offset;
  uint16_t length = 0;

  #if _USE_DEBUG_LOG_INST_FUNC == 1
  Serial.print("write ");
  Serial.println(p_inst_packet->id, HEX);
  #endif


  if     ( p_inst_packet->id == DXL_ID_BROADCAST_ID ) send_time = p_dxl_mem->ID*1000;
  else if( p_inst_packet->id == p_dxl_mem->ID       ) send_time = 0;
  else                                                return;


  addr   = (p_inst_packet->param[1]<<8) | (p_inst_packet->param[0]<<0);

  if( p_inst_packet->param_length > 2 )
  {
      length = p_inst_packet->param_length-2;
  }
  else
  {
    error = DXL_ERR_DATA_LENGTH;
  }

  if( addr >= sizeof(dxl_mem_op3_t) )
  {
    error = DXL_ERR_DATA_LENGTH;
  }
  if( length > DXL_BUF_LENGTH )
  {
    error = DXL_ERR_DATA_LENGTH;
  }

  dxl_sp_set_step(1, 1);
  if( error == 0x00 )
  {
    dxl_sp_set_step(1, 2);
    offset = 2;
    for( i=0; i<length; i++ )
    {
      if( dxl_sp.mem.attr[addr]&DXL_MEM_ATTR_WO || dxl_sp.mem.attr[addr]&DXL_MEM_ATTR_RW )
      {
        dxl_node_write_byte(addr, p_inst_packet->param[offset+i]);
        if( dxl_sp.mem.attr[addr]&DXL_MEM_ATTR_EEPROM )
        {
          EEPROM[addr] = dxl_sp.mem.data[addr];
        }
      }
      addr++;
    }
    dxl_sp_set_step(1, 3);
  }

  dxl_sp_set_step(1, 4);
  make_basic_status_packet(error, send_time);
}


/*---------------------------------------------------------------------------
     TITLE   : reg_write
     WORK    :
---------------------------------------------------------------------------*/
void reg_write(dxl_inst_packet_t *p_inst_packet)
{
  #if _USE_DEBUG_LOG_INST_FUNC == 1
  Serial.print("reg_write ");
  Serial.println(p_inst_packet->id, HEX);
  #endif

}


/*---------------------------------------------------------------------------
     TITLE   : action
     WORK    :
---------------------------------------------------------------------------*/
void action(dxl_inst_packet_t *p_inst_packet)
{
  #if _USE_DEBUG_LOG_INST_FUNC == 1
  Serial.print("action ");
  Serial.println(p_inst_packet->id, HEX);
  #endif

}


/*---------------------------------------------------------------------------
     TITLE   : factory_reset
     WORK    :
---------------------------------------------------------------------------*/
void factory_reset(dxl_inst_packet_t *p_inst_packet)
{
  #if _USE_DEBUG_LOG_INST_FUNC == 1
  Serial.print("factory_reset ");
  Serial.println(p_inst_packet->id, HEX);
  #endif

}


void reboot(dxl_inst_packet_t *p_inst_packet)
{
  #if _USE_DEBUG_LOG_INST_FUNC == 1
  Serial.print("reboot ");
  Serial.println(p_inst_packet->id, HEX);
  #endif

}


void sync_read(dxl_inst_packet_t *p_inst_packet)
{
  uint8_t  error = 0x00;
  uint16_t index = 0;
  uint32_t send_time = 0;

  uint16_t i;
  uint16_t addr;
  uint16_t offset;
  uint16_t length = 0;
  uint8_t  pre_id = 0;
  uint8_t  dev_id = 0;
  uint8_t  dev_cnt = 0;

  dxl_tx_status_packet_t *p_tx_status_packet;


  p_tx_status_packet = &dxl_sp.tx_status_packet;

  #if _USE_DEBUG_LOG_INST_FUNC == 1
  Serial.print("sync_read : ");
  Serial.println(p_inst_packet->id, HEX);
  #endif


  if( p_inst_packet->id != DXL_ID_BROADCAST_ID )
    return;


  addr   = (p_inst_packet->param[1]<<8) | (p_inst_packet->param[0]<<0);
  length = (p_inst_packet->param[3]<<8) | (p_inst_packet->param[2]<<0);

  if( addr >= sizeof(dxl_mem_op3_t) )
  {
    error = DXL_ERR_DATA_LENGTH;
  }
  if( length > DXL_BUF_LENGTH )
  {
    error = DXL_ERR_DATA_LENGTH;
  }

  dxl_sp_set_step(1, 1);
  if( error == 0x00 )
  {
    dev_cnt = 0;
    offset  = 4;
    dxl_sp_set_step(1, 2);
    while(1)
    {
      dev_id = p_inst_packet->param[offset++];

      dev_cnt++;

      if( dev_id == p_dxl_mem->ID )
      {
        for( i=0; i<length; i++ )
        {
          if( addr < sizeof(dxl_mem_op3_t))
          {
            if( dxl_sp.mem.attr[addr]&DXL_MEM_ATTR_RO || dxl_sp.mem.attr[addr]&DXL_MEM_ATTR_RW )
            {
              p_tx_status_packet->packet.param[i] = dxl_node_read_byte(addr);
            }
          }
          else
          {
            p_tx_status_packet->packet.param[i] = 0x00;
          }
          addr++;
        }
        break;
      }
      else
      {
        pre_id  = dev_id;
      }

      if( offset >= p_inst_packet->param_length ) break;
    }

    dxl_sp_set_step(1, 3);
    #if _USE_DEBUG_LOG_INST_FUNC == 1
    Serial.print("    dev_cnt ");
    Serial.println(dev_cnt);
    #endif

    if( dev_cnt > 1 )
    {
      dxl_sp.rx_status_wait_id      = pre_id;
      dxl_sp.rx_status_wait_dev_cnt = dev_cnt - 1;
      dxl_sp.rx_status_state        = RX_STATUS_STATE_SYNC_READ;
      p_tx_status_packet->state     = TX_STATUS_STATE_SYNC_READ;
    }
    else
    {
      p_tx_status_packet->state = TX_STATUS_STATE_SEND;
    }

    p_tx_status_packet->packet.param_length  = length;
    p_tx_status_packet->packet.packet_length = p_tx_status_packet->packet.param_length + 4;

    // make status packet

    p_tx_status_packet->packet.id             = p_dxl_mem->ID;
    p_tx_status_packet->packet.inst           = DXL_INST_STATUS;
    p_tx_status_packet->packet.error          = error;

    p_tx_status_packet->prev_time = micros();
    p_tx_status_packet->send_time = send_time;
  }
}


void sync_write(dxl_inst_packet_t *p_inst_packet)
{
  uint8_t  error = 0x00;
  uint16_t index = 0;
  uint32_t send_time = 0;

  uint16_t i;
  uint16_t addr;
  uint16_t offset;
  uint16_t length = 0;
  uint8_t  dev_id = 0;

  #if _USE_DEBUG_LOG_INST_FUNC == 1
  Serial.print("sync_write ");
  Serial.println(p_inst_packet->id, HEX);
  #endif


  if( p_inst_packet->id != DXL_ID_BROADCAST_ID )
    return;


  addr   = (p_inst_packet->param[1]<<8) | (p_inst_packet->param[0]<<0);
  length = (p_inst_packet->param[3]<<8) | (p_inst_packet->param[2]<<0);

  if( p_inst_packet->param_length < (4+1+length) )
  {
    error = DXL_ERR_DATA_LENGTH;
  }
  if( length > DXL_BUF_LENGTH )
  {
    error = DXL_ERR_DATA_LENGTH;
  }

  dxl_sp_set_step(1, 1);
  if( error == 0x00 )
  {
    offset = 4;
    dxl_sp_set_step(1, 2);
    while(1)
    {
      dev_id = p_inst_packet->param[offset++];

      if( dev_id == p_dxl_mem->ID )
      {
        for( i=0; i<length; i++ )
        {
          if( addr >= sizeof(dxl_mem_op3_t) )
          {
            break;
          }
          if( dxl_sp.mem.attr[addr]&DXL_MEM_ATTR_WO || dxl_sp.mem.attr[addr]&DXL_MEM_ATTR_RW )
          {
            dxl_node_write_byte(addr, p_inst_packet->param[offset+i]);
            if( dxl_sp.mem.attr[addr]&DXL_MEM_ATTR_EEPROM )
            {
              EEPROM[addr] = dxl_sp.mem.data[addr];
            }
          }
          addr++;
        }
        offset += length;
        break;
      }
      else
      {
        offset += length;
      }

      if( offset >= p_inst_packet->param_length ) break;
    }
    dxl_sp_set_step(1, 3);
  }
}


void bulk_read(dxl_inst_packet_t *p_inst_packet)
{
  uint8_t  error = 0x00;
  uint16_t index = 0;
  uint32_t send_time = 0;
  dxl_tx_status_packet_t *p_tx_status_packet;

  uint16_t i;
  uint16_t addr;
  uint16_t length = 0;
  uint8_t  pre_id = 0;
  uint8_t  dev_id = 0;
  uint16_t dev_cnt = 0;


  #if _USE_DEBUG_LOG_INST_FUNC == 1
  Serial.print("inst bulk_read ");
  Serial.println(p_inst_packet->id, HEX);
  #endif

  p_tx_status_packet = &dxl_sp.tx_status_packet;


  if( p_inst_packet->id != DXL_ID_BROADCAST_ID )
    return;

  if( p_inst_packet->param_length > DXL_BUF_LENGTH )
    return;

  dxl_sp_set_step(1, 1);
  for( i=0; i<p_inst_packet->param_length; i+=5 )
  {
    dev_id = p_inst_packet->param[i];
    addr   = (p_inst_packet->param[i+2]<<8) | (p_inst_packet->param[i+1]<<0);
    length = (p_inst_packet->param[i+4]<<8) | (p_inst_packet->param[i+3]<<0);

    dev_cnt++;

    if( dev_id == p_dxl_mem->ID && addr < sizeof(dxl_mem_op3_t) )
    {
      break;
    }

    pre_id = dev_id;

    #if _USE_DEBUG_LOG_INST_FUNC == 1
    Serial.print("     bulk_read rxid: ");
    Serial.println(dev_id, HEX);
    #endif
  }

  dxl_sp_set_step(1, 2);
  if( dev_id == p_dxl_mem->ID )
  {
    #if _USE_DEBUG_LOG_INST_FUNC == 1
    Serial.print("     bulk_read rxid: ");
    Serial.print(dev_id, HEX);
    Serial.print(" devcnt: ");
    Serial.print(dev_cnt);
    Serial.println();
    #endif

    dxl_sp_set_step(1, 3);
    index = 0;
    for( i=0; i<length; i++ )
    {
      p_tx_status_packet->packet.param[index++] = dxl_node_read_byte(addr);
      addr++;
    }

    dxl_sp_set_step(1, 4);
    if( dev_cnt > 1 )
    {
      dxl_sp.rx_status_wait_id      = pre_id;
      dxl_sp.rx_status_wait_dev_cnt = dev_cnt - 1;
      dxl_sp.rx_status_state        = RX_STATUS_STATE_BULK_READ;
      p_tx_status_packet->state     = TX_STATUS_STATE_BULK_READ;
    }
    else
    {
      p_tx_status_packet->state = TX_STATUS_STATE_SEND;
    }

    p_tx_status_packet->packet.param_length  = index;
    p_tx_status_packet->packet.packet_length = p_tx_status_packet->packet.param_length + 4;

    // make status packet

    p_tx_status_packet->packet.id             = p_dxl_mem->ID;
    p_tx_status_packet->packet.inst           = DXL_INST_STATUS;
    p_tx_status_packet->packet.error          = error;

    p_tx_status_packet->prev_time = micros();
    p_tx_status_packet->send_time = send_time;
    dxl_sp_set_step(1, 5);
  }

}


void bulk_write(dxl_inst_packet_t *p_inst_packet)
{
  uint8_t  error = 0x00;
  uint16_t index = 0;
  uint32_t send_time = 0;

  uint16_t i;
  uint16_t addr;
  uint16_t offset;
  uint16_t length = 0;
  uint8_t  pre_id = 0;
  uint8_t  dev_id = 0;


  #if _USE_DEBUG_LOG_INST_FUNC == 1
  Serial.print("bulk_write ");
  Serial.println(p_inst_packet->id, HEX);
  #endif



  if( p_inst_packet->id != DXL_ID_BROADCAST_ID )
    return;


  offset = 0;

  dxl_sp_set_step(1, 1);
  while(1)
  {
    dev_id = p_inst_packet->param[offset];
    addr   = (p_inst_packet->param[offset+2]<<8) | (p_inst_packet->param[offset+1]<<0);
    length = (p_inst_packet->param[offset+4]<<8) | (p_inst_packet->param[offset+3]<<0);

    offset += 5;

    #if _USE_DEBUG_LOG_INST_FUNC == 1
    Serial.print("    dev_id :");
    Serial.print(dev_id);
    Serial.print(" ");
    Serial.print(length);
    Serial.print(" ");
    Serial.println(p_inst_packet->param_length);

    #endif

    dxl_sp_set_step(1, 2);
    if( dev_id == p_dxl_mem->ID )
    {
      dxl_sp_set_step(1, 3);
      #if _USE_DEBUG_LOG_INST_FUNC == 1
      Serial.print("    recv data ");
      Serial.print(addr);
      Serial.print(" ");
      Serial.println(length);
      #endif

      for( i=0; i<length; i++ )
      {
        if( addr >= sizeof(dxl_mem_op3_t))
        {
          #if _USE_DEBUG_LOG_INST_FUNC == 1
          Serial.print("    addr over");
          Serial.println(p_inst_packet->id, HEX);
          #endif
          break;
        }
        if( dxl_sp.mem.attr[addr]&DXL_MEM_ATTR_WO || dxl_sp.mem.attr[addr]&DXL_MEM_ATTR_RW )
        {
          dxl_node_write_byte(addr, p_inst_packet->param[offset+i]);
          if( dxl_sp.mem.attr[addr]&DXL_MEM_ATTR_EEPROM )
          {
            EEPROM[addr] = dxl_sp.mem.data[addr];
          }
        }
        addr++;
      }
      break;
    }

    dxl_sp_set_step(1, 4);
    offset += length;

    if( offset >= p_inst_packet->param_length ) break;
  }

  #if _USE_DEBUG_LOG_INST_FUNC == 1
  Serial.println("bulk_write_end");
  #endif
}
