/*
 *  dxl_def.h
 *
 *  dynamixel define
 *
 *  Created on: 2016. 10. 21.
 *      Author: Baram
 */

#ifndef DXL_DEF_H
#define DXL_DEF_H

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>



#define DXL_PORT                  Serial3


#define DXL_ID_BROADCAST_ID       0xFE

#define DXL_BUF_LENGTH            1024


#define DXL_INST_PING             0x01
#define DXL_INST_READ             0x02
#define DXL_INST_WRITE            0x03
#define DXL_INST_REG_WRITE        0x04
#define DXL_INST_ACTION           0x05
#define DXL_INST_FACTORY_RESET    0x06
#define DXL_INST_REBOOT           0x08
#define DXL_INST_STATUS           0x55
#define DXL_INST_SYNC_READ        0x82
#define DXL_INST_SYNC_WRITE       0x83
#define DXL_INST_BULK_READ        0x92
#define DXL_INST_BULK_WRITE       0x93


#define DXL_ERR_RESULT_FAIL       0x01
#define DXL_ERR_INST              0x02
#define DXL_ERR_CRC               0x03
#define DXL_ERR_DATA_RANGE        0x04
#define DXL_ERR_DATA_LENGTH       0x05
#define DXL_ERR_DATA_LIMIT        0x06
#define DXL_ERR_ACCESS            0x07



#define _USE_DEBUG_LOG_RX_INST    (0)
#define _USE_DEBUG_LOG_RX_STATUS  (0)
#define _USE_DEBUG_LOG_RX_RAW     (0)
#define _USE_DEBUG_LOG_TX_INST    (0)
#define _USE_DEBUG_LOG_TX_STATUS  (0)
#define _USE_DEBUG_LOG_INST_FUNC  (0)



#define DXL_MEM_ATTR_NONE         0
#define DXL_MEM_ATTR_EEPROM       (1<<1)
#define DXL_MEM_ATTR_RAM          (1<<2)
#define DXL_MEM_ATTR_RO           (1<<3)
#define DXL_MEM_ATTR_WO           (1<<4)
#define DXL_MEM_ATTR_RW           (1<<5)
#define DXL_MEM_ATTR_REG          (1<<6)



typedef struct
{
  uint8_t data    [DXL_BUF_LENGTH];
  uint8_t data_reg[DXL_BUF_LENGTH];
  uint8_t attr    [DXL_BUF_LENGTH];
} dxl_mem_t;


typedef struct
{
  uint8_t addr;
  uint8_t length;
  uint8_t attr;
  uint8_t init_data;
} dxl_mem_attr_t;



#endif
