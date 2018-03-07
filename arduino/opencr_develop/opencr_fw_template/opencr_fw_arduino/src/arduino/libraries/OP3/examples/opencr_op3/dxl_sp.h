/*
 *  dxl_sp.h
 *
 *  dynamixel serial protocol
 *
 *  Created on: 2016. 10. 21.
 *      Author: Baram
 */

#ifndef DXL_SP_H
#define DXL_SP_H


#include "dxl_def.h"


#ifdef __cplusplus
 extern "C" {
#endif


#ifdef __cplusplus
}
#endif


#define TX_INST_STATE_IDLE              0
#define TX_INST_STATE_SEND              1

#define TX_STATUS_STATE_IDLE            0
#define TX_STATUS_STATE_SEND            1
#define TX_STATUS_STATE_BULK_READ       2
#define TX_STATUS_STATE_SYNC_READ       3

#define RX_INST_STATE_IDLE              0

#define RX_STATUS_STATE_IDLE            0
#define RX_STATUS_STATE_BULK_READ       1
#define RX_STATUS_STATE_SYNC_READ       2


typedef struct
{
    uint8_t  header[3];
    uint8_t  reserved;
    uint8_t  id;
    uint16_t packet_length;
    uint16_t param_length;
    uint16_t received_length;
    uint8_t  inst;
    uint8_t  param[DXL_BUF_LENGTH];
    uint16_t crc;
} dxl_inst_packet_t;


typedef struct
{
  uint8_t  header[3];
  uint8_t  reserved;
  uint8_t  id;
  uint16_t packet_length;
  uint16_t param_length;
  uint16_t received_length;
  uint8_t  inst;
  uint8_t  error;
  uint8_t  param[DXL_BUF_LENGTH];
  uint16_t crc;
} dxl_status_packet_t;


typedef struct
{
  uint8_t             state;
  uint16_t            length;
  uint16_t            crc;
  uint32_t            prev_time;
  dxl_inst_packet_t   packet;
} dxl_rx_inst_packet_t;


typedef struct
{
  uint8_t             state;
  uint16_t            length;
  uint16_t            crc;
  uint32_t            prev_time;
  dxl_status_packet_t packet;
} dxl_rx_status_packet_t;


typedef struct
{
  uint8_t             state;
  uint16_t            length;
  uint16_t            crc;
  uint32_t            prev_time;
  uint32_t            send_time;
  dxl_inst_packet_t   packet;
} dxl_tx_inst_packet_t;


typedef struct
{
  uint8_t             state;
  uint16_t            length;
  uint16_t            crc;
  uint32_t            prev_time;
  uint32_t            send_time;
  dxl_status_packet_t packet;
  uint8_t             run_flag;
  void                (*run_func)();
} dxl_tx_status_packet_t;


typedef struct
{
  void (*ping         )(dxl_inst_packet_t *p_inst_packet);
  void (*read         )(dxl_inst_packet_t *p_inst_packet);
  void (*write        )(dxl_inst_packet_t *p_inst_packet);
  void (*reg_write    )(dxl_inst_packet_t *p_inst_packet);
  void (*action       )(dxl_inst_packet_t *p_inst_packet);
  void (*factory_reset)(dxl_inst_packet_t *p_inst_packet);
  void (*reboot       )(dxl_inst_packet_t *p_inst_packet);
  void (*sync_read    )(dxl_inst_packet_t *p_inst_packet);
  void (*sync_write   )(dxl_inst_packet_t *p_inst_packet);
  void (*bulk_read    )(dxl_inst_packet_t *p_inst_packet);
  void (*bulk_write   )(dxl_inst_packet_t *p_inst_packet);
} dxl_inst_func_t;


typedef struct
{
  dxl_mem_t               mem;
  dxl_inst_func_t         inst_func;

  uint8_t                 rx_inst_state;
  uint8_t                 rx_status_state;
  uint8_t                 rx_status_wait_id;
  uint8_t                 rx_status_wait_dev_cnt;

  dxl_rx_inst_packet_t    rx_inst_packet;
  dxl_rx_status_packet_t  rx_status_packet;

  dxl_tx_inst_packet_t    tx_inst_packet;
  dxl_tx_status_packet_t  tx_status_packet;
} dxl_sp_t;




void dxl_sp_init(dxl_sp_t *p_dxl_sp);
void dxl_sp_begin(uint8_t baud);

uint8_t dxl_sp_rx_update(dxl_sp_t *p_dxl_sp);
uint8_t dxl_sp_tx_update(dxl_sp_t *p_dxl_sp);

void    dxl_sp_set_step(uint8_t index, uint8_t data);
uint8_t dxl_sp_get_step(uint8_t index);

#endif
