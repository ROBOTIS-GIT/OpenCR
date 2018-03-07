/*
 *  dxl_sp.c
 *
 *  dynamixel serial protocol
 *
 *  Created on: 2016. 10. 21.
 *      Author: Baram
 */

#include "dxl_hw.h"
#include "dxl_sp.h"





#define INST_STATE_IDLE           0
#define INST_STATE_RESERVED       1
#define INST_STATE_ID             2
#define INST_STATE_LENGTH_L       3
#define INST_STATE_LENGTH_H       4
#define INST_STATE_INST           5
#define INST_STATE_PARAM          6
#define INST_STATE_CRC_L          7
#define INST_STATE_CRC_H          8


#define STATUS_STATE_IDLE         0
#define STATUS_STATE_RESERVED     1
#define STATUS_STATE_ID           2
#define STATUS_STATE_LENGTH_L     3
#define STATUS_STATE_LENGTH_H     4
#define STATUS_STATE_INST         5
#define STATUS_STATE_ERROR        6
#define STATUS_STATE_PARAM        7
#define STATUS_STATE_CRC_L        8
#define STATUS_STATE_CRC_H        9







uint8_t dxl_sp_rx_inst_state(dxl_rx_inst_packet_t *p_rx_packet, uint8_t data_in);
uint8_t dxl_sp_rx_status_state(dxl_rx_status_packet_t *p_rx_packet, uint8_t data_in);
uint8_t dxl_sp_rx_inst_update(dxl_sp_t *p_dxl_sp, uint8_t data_in);
uint8_t dxl_sp_rx_status_update(dxl_sp_t *p_dxl_sp, uint8_t data_in);

uint8_t dxl_sp_tx_inst_state(dxl_tx_inst_packet_t *p_tx_packet);
uint8_t dxl_sp_tx_status_state(dxl_tx_status_packet_t *p_tx_packet);

uint8_t dxl_sp_send_inst_packet(dxl_tx_inst_packet_t *p_packet);
uint8_t dxl_sp_send_status_packet(dxl_tx_status_packet_t *p_packet);

void dxl_sp_inst_add_stuffing(dxl_inst_packet_t *p_packet);
void dxl_sp_inst_remove_stuffing(dxl_inst_packet_t *p_packet);
void dxl_sp_status_add_stuffing(dxl_status_packet_t *p_packet);
void dxl_sp_status_remove_stuffing(dxl_status_packet_t *p_packet);

void dxl_sp_update_crc(uint16_t *p_crc_cur, uint8_t data_in);





/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_init
     WORK    :
---------------------------------------------------------------------------*/
void dxl_sp_init(dxl_sp_t *p_dxl_sp)
{
  p_dxl_sp->rx_inst_state          = RX_INST_STATE_IDLE;
  p_dxl_sp->rx_status_state        = RX_STATUS_STATE_IDLE;

  p_dxl_sp->rx_inst_packet.state   = INST_STATE_IDLE;
  p_dxl_sp->rx_status_packet.state = STATUS_STATE_IDLE;

  p_dxl_sp->tx_inst_packet.state   = TX_INST_STATE_IDLE;
  p_dxl_sp->tx_status_packet.state = TX_STATUS_STATE_IDLE;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_begin
     WORK    :
---------------------------------------------------------------------------*/
void dxl_sp_begin(uint8_t baud)
{
  dxl_hw_begin(baud);
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_set_step
     WORK    :
---------------------------------------------------------------------------*/
void dxl_sp_set_step(uint8_t index, uint8_t data)
{
  uint32_t reg;
  uint32_t offset_bit;

  if(index  < 4)
  {
    offset_bit = index*8;
    reg  = drv_rtc_read_step();
    reg &= ~(0xFF<<offset_bit);

    reg |= (data<<offset_bit);
    drv_rtc_write_step(reg);
  }
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_get_step
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_sp_get_step(uint8_t index)
{
  uint8_t  ret = 0;
  uint32_t reg;
  uint32_t offset_bit;

  if(index  < 4)
  {
    offset_bit = index*8;
    reg  = drv_rtc_read_step();

    ret  = (reg>>offset_bit);
  }

  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_rx_update
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_sp_rx_update(dxl_sp_t *p_dxl_sp)
{
  uint8_t ret = TRUE;
  uint8_t ret_inst;
  uint8_t ret_status;
  uint8_t data_in;
  uint8_t inst_code;

  uint32_t i;
  uint32_t received_length;

  //-- receive byte
  //
  received_length = dxl_hw_available();
  if(received_length > 1024)
  {
    received_length = 1024;
  }
  if( received_length > 0 )
  {
    for( i=0; i<received_length; i++ )
    {
      data_in = dxl_hw_read();

      ret_inst   = dxl_sp_rx_inst_update(p_dxl_sp, data_in);
      ret_status = dxl_sp_rx_status_update(p_dxl_sp, data_in);
    }
  }
  else
  {
    return FALSE;
  }

  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_rx_inst_update
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_sp_rx_inst_update(dxl_sp_t *p_dxl_sp, uint8_t data_in)
{
  uint8_t ret = FALSE;
  uint8_t inst_code;
  dxl_inst_packet_t *p_packet;


  p_packet = &p_dxl_sp->rx_inst_packet.packet;

  ret   = dxl_sp_rx_inst_state(&p_dxl_sp->rx_inst_packet, data_in);
  if( ret == TRUE )
  {
    inst_code = p_dxl_sp->rx_inst_packet.packet.inst;
    dxl_sp_set_step(0, inst_code);
    switch(inst_code)
    {
      case DXL_INST_PING:
        if( p_dxl_sp->inst_func.ping != NULL ) p_dxl_sp->inst_func.ping(p_packet);
        break;

      case DXL_INST_READ:
        if( p_dxl_sp->inst_func.read != NULL ) p_dxl_sp->inst_func.read(p_packet);
        break;

      case DXL_INST_WRITE:
        if( p_dxl_sp->inst_func.write != NULL ) p_dxl_sp->inst_func.write(p_packet);
        break;

      case DXL_INST_REG_WRITE:
        if( p_dxl_sp->inst_func.reg_write != NULL ) p_dxl_sp->inst_func.reg_write(p_packet);
        break;

      case DXL_INST_ACTION:
        if( p_dxl_sp->inst_func.action != NULL ) p_dxl_sp->inst_func.action(p_packet);
        break;

      case DXL_INST_FACTORY_RESET:
        if( p_dxl_sp->inst_func.factory_reset != NULL ) p_dxl_sp->inst_func.factory_reset(p_packet);
        break;

      case DXL_INST_REBOOT:
        if( p_dxl_sp->inst_func.reboot != NULL ) p_dxl_sp->inst_func.reboot(p_packet);
        break;

      case DXL_INST_STATUS:
        break;

      case DXL_INST_SYNC_READ:
        if( p_dxl_sp->inst_func.sync_read != NULL ) p_dxl_sp->inst_func.sync_read(p_packet);
        break;

      case DXL_INST_SYNC_WRITE:
        if( p_dxl_sp->inst_func.sync_write != NULL ) p_dxl_sp->inst_func.sync_write(p_packet);
        break;

      case DXL_INST_BULK_READ:
        if( p_dxl_sp->inst_func.bulk_read != NULL ) p_dxl_sp->inst_func.bulk_read(p_packet);
        break;

      case DXL_INST_BULK_WRITE:
        if( p_dxl_sp->inst_func.bulk_write != NULL ) p_dxl_sp->inst_func.bulk_write(p_packet);
        break;
    }
    dxl_sp_set_step(0, 0);
  }

  return ret;
}


uint8_t dxl_sp_inst_ping(dxl_sp_t *p_dxl_sp)
{

}




/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_rx_status_update
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_sp_rx_status_update(dxl_sp_t *p_dxl_sp, uint8_t data_in)
{
  uint8_t ret = FALSE;
  static uint32_t tTime;
  uint8_t wait_id;
  uint8_t rx_id;

  ret = dxl_sp_rx_status_state(&p_dxl_sp->rx_status_packet, data_in);

  if( (millis()-tTime) > p_dxl_sp->rx_status_wait_dev_cnt*10 )
  {
    tTime = millis();
    if( p_dxl_sp->rx_status_wait_dev_cnt > 0 )
    {
        p_dxl_sp->rx_status_wait_dev_cnt = 0;
        p_dxl_sp->rx_status_state        = RX_STATUS_STATE_IDLE;
    }
    else
    {
      p_dxl_sp->rx_status_state = RX_STATUS_STATE_IDLE;
    }
  }

  if( ret == TRUE )
  {
    tTime = millis();

    switch( p_dxl_sp->rx_status_state )
    {
      case RX_STATUS_STATE_IDLE:
        p_dxl_sp->rx_status_wait_dev_cnt = 0;
        break;

      case RX_STATUS_STATE_SYNC_READ:
      case RX_STATUS_STATE_BULK_READ:
        wait_id = p_dxl_sp->rx_status_wait_id;
        rx_id   = p_dxl_sp->rx_status_packet.packet.id;

        #if _USE_DEBUG_LOG_TX_STATUS == 1
        Serial.print("wait_id ");
        Serial.print(wait_id, HEX);
        Serial.print(" ");
        Serial.print(p_dxl_sp->tx_status_packet.state);
        Serial.print(" ");
        Serial.println(rx_id, HEX);
        #endif

        if( p_dxl_sp->rx_status_wait_dev_cnt > 0 )
        {
          p_dxl_sp->rx_status_wait_dev_cnt--;
        }
        if( wait_id == rx_id || p_dxl_sp->rx_status_wait_dev_cnt == 0 )
        {
          p_dxl_sp->tx_status_packet.state = TX_STATUS_STATE_SEND;
          p_dxl_sp->rx_status_state        = RX_STATUS_STATE_IDLE;
        }
        break;

      default:
        p_dxl_sp->rx_status_state = RX_STATUS_STATE_IDLE;
        break;
    }
  }

  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_tx_update
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_sp_tx_update(dxl_sp_t *p_dxl_sp)
{
  uint8_t ret = FALSE;
  uint8_t ret_inst;
  uint8_t ret_status;


  ret_inst   = dxl_sp_tx_inst_state(&p_dxl_sp->tx_inst_packet);
  ret_status = dxl_sp_tx_status_state(&p_dxl_sp->tx_status_packet);

  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_tx_inst_state
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_sp_tx_inst_state(dxl_tx_inst_packet_t *p_tx_packet)
{
  uint8_t ret = FALSE;


  switch(p_tx_packet->state)
  {
    case TX_INST_STATE_IDLE:
      break;

    case TX_INST_STATE_SEND:
      break;

    default:
      p_tx_packet->state = TX_INST_STATE_IDLE;
      break;
  }

  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_tx_status_state
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_sp_tx_status_state(dxl_tx_status_packet_t *p_tx_packet)
{
  uint8_t ret = FALSE;


  switch(p_tx_packet->state)
  {
    case TX_STATUS_STATE_IDLE:
      break;

    case TX_STATUS_STATE_SEND:
      if( micros()-p_tx_packet->prev_time > p_tx_packet->send_time )
      {
        #if _USE_DEBUG_LOG_TX_STATUS == 1
        Serial.println("tx_status_send");
        #endif
        dxl_sp_status_add_stuffing(&p_tx_packet->packet);
        dxl_sp_send_status_packet(p_tx_packet);
        p_tx_packet->state = TX_STATUS_STATE_IDLE;

        if(p_tx_packet->run_flag)
        {
          if( p_tx_packet->run_func != NULL )
          {
            p_tx_packet->run_func();
          }
        }
        p_tx_packet->run_flag = 0;
      }
      break;

    default:
      p_tx_packet->state = TX_STATUS_STATE_IDLE;
      break;
  }

  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_rx_inst_state
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_sp_rx_inst_state(dxl_rx_inst_packet_t *p_rx_packet, uint8_t data_in)
{
  uint8_t ret = FALSE;

  static uint8_t header_cnt = 0;
  static uint8_t header_tbl[3];


  //-- time out(100ms)
	//
  if( (micros() - p_rx_packet->prev_time) > 100000 )
  {
    p_rx_packet->state     = STATUS_STATE_IDLE;
    p_rx_packet->prev_time = micros();
    header_cnt = 0;
  }

  #if _USE_DEBUG_LOG_RX_INST == 1
  Serial.print(header_cnt);
  Serial.print(" ");
  Serial.print(p_rx_packet->state);
  Serial.print(" ");
  Serial.print(p_rx_packet->crc, HEX);
  Serial.print(" ");
  Serial.println(data_in, HEX);
  #endif

  switch(p_rx_packet->state)
  {
    case INST_STATE_IDLE:
      p_rx_packet->prev_time = micros();

      if( header_cnt >= 2 )
      {
        header_tbl[2] = data_in;

        if(    header_tbl[0] == 0xFF
            && header_tbl[1] == 0xFF
            && header_tbl[2] == 0xFD )
        {
          header_cnt = 0;
          p_rx_packet->crc = 0;
          dxl_sp_update_crc(&p_rx_packet->crc, 0xFF);
          dxl_sp_update_crc(&p_rx_packet->crc, 0xFF);
          dxl_sp_update_crc(&p_rx_packet->crc, 0xFD);
          p_rx_packet->state = INST_STATE_RESERVED;
        }
        else
        {
          header_tbl[0] = header_tbl[1];
          header_tbl[1] = header_tbl[2];
          header_tbl[2] = 0;
        }
      }
      else
      {
          header_tbl[header_cnt] = data_in;
          header_cnt++;
      }
      break;

    case INST_STATE_RESERVED:
      if( data_in == 0xFD )
      {
        p_rx_packet->state = INST_STATE_IDLE;
      }
      else
      {
        p_rx_packet->packet.reserved = data_in;
        p_rx_packet->state = INST_STATE_ID;
      }
      dxl_sp_update_crc(&p_rx_packet->crc, data_in);
      break;

    case INST_STATE_ID:
      p_rx_packet->packet.id = data_in;
      p_rx_packet->state = INST_STATE_LENGTH_L;
      dxl_sp_update_crc(&p_rx_packet->crc, data_in);
      break;

    case INST_STATE_LENGTH_L:
      p_rx_packet->packet.packet_length = data_in;
      p_rx_packet->state = INST_STATE_LENGTH_H;
      dxl_sp_update_crc(&p_rx_packet->crc, data_in);
      break;

    case INST_STATE_LENGTH_H:
      p_rx_packet->packet.packet_length |= data_in<<8;
      p_rx_packet->state = INST_STATE_INST;

      if( p_rx_packet->packet.packet_length > 3 )
      {
        p_rx_packet->packet.param_length = p_rx_packet->packet.packet_length - 3;
      }
      else
      {
        p_rx_packet->packet.param_length = 0;
        p_rx_packet->state = INST_STATE_INST;
      }
      if( p_rx_packet->packet.param_length >= DXL_BUF_LENGTH )
      {
        p_rx_packet->packet.param_length = 0;
        p_rx_packet->state = STATUS_STATE_IDLE;
      }
      dxl_sp_update_crc(&p_rx_packet->crc, data_in);
      break;

    case INST_STATE_INST:
      p_rx_packet->packet.inst = data_in;
      if( p_rx_packet->packet.param_length > 0 )
      {
        p_rx_packet->packet.received_length = 0;
        p_rx_packet->state = INST_STATE_PARAM;
      }
      else
      {
        p_rx_packet->state = INST_STATE_CRC_L;
      }

      if( data_in == DXL_INST_STATUS )
      {
        p_rx_packet->state = INST_STATE_IDLE;
      }
      dxl_sp_update_crc(&p_rx_packet->crc, data_in);
      break;

    case INST_STATE_PARAM:

      p_rx_packet->packet.param[p_rx_packet->packet.received_length++] = data_in;
      dxl_sp_update_crc(&p_rx_packet->crc, data_in);

      if( p_rx_packet->packet.received_length >= p_rx_packet->packet.param_length )
      {
        p_rx_packet->state = INST_STATE_CRC_L;
      }
      break;

    case INST_STATE_CRC_L:
      p_rx_packet->packet.crc = data_in;
      p_rx_packet->state = INST_STATE_CRC_H;
      break;

    case INST_STATE_CRC_H:
      p_rx_packet->packet.crc |= data_in<<8;
      p_rx_packet->state = INST_STATE_IDLE;

      if( p_rx_packet->packet.crc == p_rx_packet->crc )
      {
        #if _USE_DEBUG_LOG_RX_INST == 1
        Serial.println("RXD Inst");
        #endif
        ret = TRUE;
        dxl_sp_inst_remove_stuffing(&p_rx_packet->packet);
      }
      break;

    default:
      p_rx_packet->state = INST_STATE_IDLE;
      break;
  }


  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_rx_status_state
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_sp_rx_status_state(dxl_rx_status_packet_t *p_rx_packet, uint8_t data_in)
{
  uint8_t ret = FALSE;


  static uint8_t header_cnt = 0;
  static uint8_t header_tbl[3];
  uint32_t calc_timeout;

  //-- time out(100ms)
	//
  calc_timeout = micros() - p_rx_packet->prev_time;
  if( calc_timeout > 100000 )
  {
    p_rx_packet->state     = STATUS_STATE_IDLE;
    p_rx_packet->prev_time = micros();
    header_cnt = 0;
  }

  #if _USE_DEBUG_LOG_RX_RAW == 1
  Serial.print(header_cnt);
  Serial.print(" ");
  Serial.print(p_rx_packet->state);
  Serial.print(" ");
  Serial.print(p_rx_packet->crc, HEX);
  Serial.print(" ");
  Serial.println(data_in, HEX);
  #endif

  switch(p_rx_packet->state)
  {
    case STATUS_STATE_IDLE:
      p_rx_packet->prev_time = micros();

      if( header_cnt >= 2 )
      {
        header_tbl[2] = data_in;

        if(    header_tbl[0] == 0xFF
            && header_tbl[1] == 0xFF
            && header_tbl[2] == 0xFD )
        {
          header_cnt = 0;
          p_rx_packet->crc = 0;
          dxl_sp_update_crc(&p_rx_packet->crc, 0xFF);
          dxl_sp_update_crc(&p_rx_packet->crc, 0xFF);
          dxl_sp_update_crc(&p_rx_packet->crc, 0xFD);
          p_rx_packet->state = STATUS_STATE_RESERVED;
        }
        else
        {
          header_tbl[0] = header_tbl[1];
          header_tbl[1] = header_tbl[2];
          header_tbl[2] = 0;
        }
      }
      else
      {
          header_tbl[header_cnt] = data_in;
          header_cnt++;
      }
      break;

    case STATUS_STATE_RESERVED:
      if( data_in == 0xFD )
      {
        p_rx_packet->state = STATUS_STATE_IDLE;
      }
      else
      {
        p_rx_packet->packet.reserved = data_in;
        p_rx_packet->state = STATUS_STATE_ID;
      }
      dxl_sp_update_crc(&p_rx_packet->crc, data_in);
      break;

    case STATUS_STATE_ID:
      p_rx_packet->packet.id = data_in;
      p_rx_packet->state = STATUS_STATE_LENGTH_L;
      dxl_sp_update_crc(&p_rx_packet->crc, data_in);
      break;

    case STATUS_STATE_LENGTH_L:
      p_rx_packet->packet.packet_length = data_in;
      p_rx_packet->state = STATUS_STATE_LENGTH_H;
      dxl_sp_update_crc(&p_rx_packet->crc, data_in);
      break;

    case STATUS_STATE_LENGTH_H:
      p_rx_packet->packet.packet_length |= data_in<<8;
      p_rx_packet->state = STATUS_STATE_INST;

      if( p_rx_packet->packet.packet_length > 4 )
      {
        p_rx_packet->packet.param_length = p_rx_packet->packet.packet_length - 4;
      }
      else
      {
        p_rx_packet->packet.param_length = 0;
        p_rx_packet->state = STATUS_STATE_INST;
      }
      if( p_rx_packet->packet.param_length >= DXL_BUF_LENGTH )
      {
        p_rx_packet->packet.param_length = 0;
        p_rx_packet->state = STATUS_STATE_IDLE;
      }
      dxl_sp_update_crc(&p_rx_packet->crc, data_in);
      break;

    case STATUS_STATE_INST:
      p_rx_packet->packet.inst = data_in;
      p_rx_packet->state = STATUS_STATE_ERROR;
      if( data_in != DXL_INST_STATUS )
      {
        p_rx_packet->state = STATUS_STATE_IDLE;
      }
      dxl_sp_update_crc(&p_rx_packet->crc, data_in);
      break;

    case STATUS_STATE_ERROR:
      p_rx_packet->packet.error = data_in;
      if( p_rx_packet->packet.param_length > 0 )
      {
        p_rx_packet->packet.received_length = 0;
        p_rx_packet->state = STATUS_STATE_PARAM;
      }
      else
      {
        p_rx_packet->state = STATUS_STATE_CRC_L;
      }
      dxl_sp_update_crc(&p_rx_packet->crc, data_in);
      break;


    case STATUS_STATE_PARAM:

      p_rx_packet->packet.param[p_rx_packet->packet.received_length++] = data_in;
      dxl_sp_update_crc(&p_rx_packet->crc, data_in);

      if( p_rx_packet->packet.received_length >= p_rx_packet->packet.param_length )
      {
        p_rx_packet->state = STATUS_STATE_CRC_L;
      }
      break;

    case STATUS_STATE_CRC_L:
      p_rx_packet->packet.crc = data_in;
      p_rx_packet->state = STATUS_STATE_CRC_H;
      break;

    case STATUS_STATE_CRC_H:
      p_rx_packet->packet.crc |= data_in<<8;
      p_rx_packet->state = STATUS_STATE_IDLE;

      if( p_rx_packet->packet.crc == p_rx_packet->crc )
      {
        #if _USE_DEBUG_LOG_RX_STATUS == 1
        Serial.print("rx_status : ");
        Serial.println(p_rx_packet->packet.id, HEX);
        #endif
        ret = TRUE;
        dxl_sp_status_remove_stuffing(&p_rx_packet->packet);
      }
      break;

    default:
      p_rx_packet->state = STATUS_STATE_IDLE;
      break;
  }


  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_send_inst_packet
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_sp_send_inst_packet(dxl_tx_inst_packet_t *p_tx_packet)
{
  uint8_t ret = FALSE;

  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_send_status_packet
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_sp_send_status_packet(dxl_tx_status_packet_t *p_tx_packet)
{
  uint8_t  ret = TRUE;
  uint16_t i;


  dxl_hw_tx_enable();

  p_tx_packet->crc = 0;

  dxl_hw_write(0xFF); // Header
  dxl_hw_write(0xFF); // Header
  dxl_hw_write(0xFD); // Header
  dxl_hw_write(0x00); // Reserved
  dxl_sp_update_crc(&p_tx_packet->crc, 0xFF);
  dxl_sp_update_crc(&p_tx_packet->crc, 0xFF);
  dxl_sp_update_crc(&p_tx_packet->crc, 0xFD);
  dxl_sp_update_crc(&p_tx_packet->crc, 0x00);

  dxl_hw_write(p_tx_packet->packet.id); // ID
  dxl_sp_update_crc(&p_tx_packet->crc, p_tx_packet->packet.id);

  dxl_hw_write((p_tx_packet->packet.packet_length>>0) & 0xFF); // Packet Length
  dxl_hw_write((p_tx_packet->packet.packet_length>>8) & 0xFF); // Packet Length

  dxl_sp_update_crc(&p_tx_packet->crc, (p_tx_packet->packet.packet_length>>0) & 0xFF);
  dxl_sp_update_crc(&p_tx_packet->crc, (p_tx_packet->packet.packet_length>>8) & 0xFF);



  dxl_hw_write(DXL_INST_STATUS);                            // Instruction
  dxl_sp_update_crc(&p_tx_packet->crc, DXL_INST_STATUS);

  dxl_hw_write(p_tx_packet->packet.error);                     // Error
  dxl_sp_update_crc(&p_tx_packet->crc, p_tx_packet->packet.error);

  for( i=0; i<p_tx_packet->packet.param_length; i++ )
  {
    dxl_hw_write(p_tx_packet->packet.param[i]);                // Param
    dxl_sp_update_crc(&p_tx_packet->crc, p_tx_packet->packet.param[i]);
  }

  p_tx_packet->packet.crc = p_tx_packet->crc;

  dxl_hw_write((p_tx_packet->packet.crc>>0) & 0xFF); // Packet Length
  dxl_hw_write((p_tx_packet->packet.crc>>8) & 0xFF); // Packet Length


  dxl_hw_tx_disable();

  #if _USE_DEBUG_LOG_TX_STATUS == 1
  Serial.println("send_status_packet");
  #endif
  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_inst_add_stuffing
     WORK    :
---------------------------------------------------------------------------*/
void dxl_sp_inst_add_stuffing(dxl_inst_packet_t *p_packet)
{
  uint8_t stuff_buf[DXL_BUF_LENGTH];
  uint8_t *p_data;
  uint16_t i;
  uint16_t index;
  uint16_t stuff_length;

  p_data = &p_packet->inst;

  index = 0;
  stuff_length = 0;
  for( i=0; i<p_packet->packet_length-2; i++ )
  {
    stuff_buf[index++] = p_data[i];

    if( i >= 2 )
    {
      if( p_data[i-2] == 0xFF && p_data[i-1] == 0xFF  && p_data[i] == 0xFD )
      {
        stuff_buf[index++] = 0xFD;
        stuff_length++;
      }
    }
  }

  if( stuff_length > 0 )
  {
    for( i=0; i<index; i++ )
    {
      p_data[i] = stuff_buf[i];
    }
    p_packet->packet_length += stuff_length;
    p_packet->param_length   = p_packet->packet_length - 3;
  }
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_inst_remove_stuffing
     WORK    :
---------------------------------------------------------------------------*/
void dxl_sp_inst_remove_stuffing(dxl_inst_packet_t *p_packet)
{
  uint8_t *p_data;
  uint16_t i;
  uint16_t index;
  uint16_t stuff_length;

  p_data = &p_packet->inst;

  index = 0;
  stuff_length = 0;
  for( i=0; i<p_packet->packet_length-2; i++ )
  {
    if( i >= 2 )
    {
      if( p_data[i-2] == 0xFF && p_data[i-1] == 0xFF  && p_data[i] == 0xFD )
      {
        i++;
        p_data[i] = p_data[i+1];
        stuff_length++;
      }
    }
  }

  if( stuff_length > 0 )
  {
    p_packet->packet_length -= stuff_length;
    p_packet->param_length   = p_packet->packet_length - 3;
  }
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_status_add_stuffing
     WORK    :
---------------------------------------------------------------------------*/
void dxl_sp_status_add_stuffing(dxl_status_packet_t *p_packet)
{
  uint8_t stuff_buf[DXL_BUF_LENGTH];
  uint8_t *p_data;
  uint16_t i;
  uint16_t index;
  uint16_t stuff_length;

  p_data = &p_packet->inst;

  index = 0;
  stuff_length = 0;
  for( i=0; i<p_packet->packet_length-2; i++ )
  {
    stuff_buf[index++] = p_data[i];

    if( i >= 2 )
    {
      if( p_data[i-2] == 0xFF && p_data[i-1] == 0xFF  && p_data[i] == 0xFD )
      {
        stuff_buf[index++] = 0xFD;
        stuff_length++;
      }
    }
  }

  if( stuff_length > 0 )
  {
    for( i=0; i<index; i++ )
    {
      p_data[i] = stuff_buf[i];
    }
    p_packet->packet_length += stuff_length;
    p_packet->param_length   = p_packet->packet_length - 4;
  }
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_status_remove_stuffing
     WORK    :
---------------------------------------------------------------------------*/
void dxl_sp_status_remove_stuffing(dxl_status_packet_t *p_packet)
{
  uint8_t *p_data;
  uint16_t i;
  uint16_t index;
  uint16_t stuff_length;

  p_data = &p_packet->inst;

  index = 0;
  stuff_length = 0;
  for( i=0; i<p_packet->packet_length-2; i++ )
  {
    if( i >= 2 )
    {
      if( p_data[i-2] == 0xFF && p_data[i-1] == 0xFF  && p_data[i] == 0xFD )
      {
        i++;
        p_data[i] = p_data[i+1];
        stuff_length++;
      }
    }
  }

  if( stuff_length > 0 )
  {
    p_packet->packet_length -= stuff_length;
    p_packet->param_length   = p_packet->packet_length - 4;
  }
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_sp_update_crc
     WORK    :
---------------------------------------------------------------------------*/
void dxl_sp_update_crc(uint16_t *p_crc_cur, uint8_t data_in)
{
  uint16_t crc;
  uint16_t i;
  unsigned short crc_table[256] = {0x0000,
                                  0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                                  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
                                  0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
                                  0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
                                  0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
                                  0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
                                  0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
                                  0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
                                  0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                                  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
                                  0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
                                  0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
                                  0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
                                  0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
                                  0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
                                  0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
                                  0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                                  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
                                  0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
                                  0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
                                  0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
                                  0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
                                  0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
                                  0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
                                  0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                                  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
                                  0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
                                  0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
                                  0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
                                  0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
                                  0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
                                  0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
                                  0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                                  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
                                  0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
                                  0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
                                  0x820D, 0x8207, 0x0202 };

  crc = *p_crc_cur;

  i = ((unsigned short)(crc >> 8) ^ data_in) & 0xFF;
  *p_crc_cur = (crc << 8) ^ crc_table[i];
}
