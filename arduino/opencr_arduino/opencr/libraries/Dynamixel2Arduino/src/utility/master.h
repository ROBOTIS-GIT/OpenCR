/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef DYNAMIXEL_MASTER_H_
#define DYNAMIXEL_MASTER_H_


#include "packet_handler.h"
#include "protocol.h"

typedef struct InfoFromPing {
  uint8_t id;
  uint8_t firmware_version;
  uint16_t model_number;
} XelInfoFromPing_t;

typedef struct XelsInfoFromPing {
  uint8_t id_count;
  XelInfoFromPing_t xel[DXL_MAX_NODE];
} RecvInfoFromPing_t;

typedef struct XelInfoForStatusInst{
  uint8_t id;
  uint16_t length;
  uint8_t error;
  uint8_t data[DXL_MAX_NODE_BUFFER_SIZE];
} XelInfoForStatusInst_t;

typedef struct RecvInfoFromStatusInst{
  uint8_t id_count;
  XelInfoForStatusInst_t xel[DXL_MAX_NODE];
} RecvInfoFromStatusInst_t;

typedef struct InfoForSyncReadParam{
  uint8_t id;
} InfoForSyncReadParam_t;

typedef struct ParamForSyncReadInst{
  uint16_t addr;
  uint16_t length;
  uint8_t id_count;
  InfoForSyncReadParam_t xel[DXL_MAX_NODE];
} ParamForSyncReadInst_t;

typedef struct XelInfoForSyncWriteParam{
  uint8_t id;
  uint8_t data[DXL_MAX_NODE_BUFFER_SIZE];
} XelInfoForSyncWriteParam_t;

typedef struct ParamForSyncWriteInst{
  uint16_t addr;
  uint16_t length;
  uint8_t id_count;
  XelInfoForSyncWriteParam_t xel[DXL_MAX_NODE];
} ParamForSyncWriteInst_t;

typedef struct XelInfoForBulkReadParam{
  uint8_t id;
  uint16_t addr;
  uint16_t length;
} XelInfoForBulkReadParam_t;

typedef struct ParamForBulkReadInst{
  uint8_t id_count;
  XelInfoForBulkReadParam_t xel[DXL_MAX_NODE];
} ParamForBulkReadInst_t;

typedef struct XelInfoForBulkWriteParam{
  uint8_t id;
  uint16_t addr;
  uint16_t length;
  uint8_t data[DXL_MAX_NODE_BUFFER_SIZE];
} XelInfoForBulkWriteParam_t;

typedef struct ParamForBulkWriteInst{
  uint8_t id_count;
  XelInfoForBulkWriteParam_t xel[DXL_MAX_NODE];
} ParamForBulkWriteInst_t;

typedef union
{
  ParamForSyncReadInst_t sync_read;
  ParamForSyncWriteInst_t sync_write;
  ParamForBulkReadInst_t bulk_read;
  ParamForBulkWriteInst_t bulk_write;
} send_param_t;

typedef union
{
  RecvInfoFromPing_t ping;
  RecvInfoFromStatusInst_t read;
  RecvInfoFromStatusInst_t sync_read;
  RecvInfoFromStatusInst_t bulk_read;
} recv_info_t;


namespace DYNAMIXEL {

class Master
{
  public:
    /**
     * @brief The constructor.
     * @code
     * const int DXL_DIR_PIN = 2;
     * const float PROTOCOL_VER = 2.0;
     * DYNAMIXEL::SerialPortHandler dxl_port(Serial1, DXL_DIR_PIN);
     * DYNAMIXEL::Master dxl_master(dxl_port, PROTOCOL_VER);
     * @endcode
     * @param port The PortHandler instance you want to use on the board to communicate with DYNAMIXELs.
     *             It can be used not only for Serial but also for other communication port handlers like SerialPortHandler class.
     * @param protocol_ver DYNAMIXEL protocol version used for communications. (default : 2.0)
     */
    Master(PortHandler &port, float protocol_ver = DXL_PACKET_VER_2_0);

    /**
     * @brief The constructor.
     *        This constructor must be added to the PortHanlder instance via the @setPort () function after creation.
     * @code
     * const float PROTOCOL_VER = 2.0;
     * DYNAMIXEL::Master dxl_master(PROTOCOL_VER);
     * @endcode
     * @param protocol_ver DYNAMIXEL protocol version used for communications. (default : 2.0)        
     */    
    Master(float protocol_ver = DXL_PACKET_VER_2_0);

    bool setPortProtocolVersion(float version);
    float getPortProtocolVersion();

    bool setPort(PortHandler &port);

    uint8_t ping(uint8_t id, 
      XelInfoFromPing_t *recv_info_array, uint8_t recv_array_cnt, uint32_t timeout = 3);
    bool ping(uint8_t id,
      RecvInfoFromPing_t &recv_info, uint32_t timeout = 3);
          
    int32_t read(uint8_t id, uint16_t addr, uint16_t addr_length,
      uint8_t *p_recv_buf, uint16_t recv_buf_length, uint32_t timeout = 3);

    bool write(uint8_t id, uint16_t addr, 
      const uint8_t *p_data, uint16_t data_length, uint32_t timeout = 3);

    bool writeNoResp(uint8_t id, uint16_t addr, 
      const uint8_t *p_data, uint16_t data_length);

    //TODO: bool regWrite();
    //TODO: bool action();
    bool factoryReset(uint8_t id, uint8_t option, uint32_t timeout = 3);
    bool reboot(uint8_t id, uint32_t timeout);

    //TODO: bool clear();

    bool syncRead(const ParamForSyncReadInst_t &param_info, RecvInfoFromStatusInst_t &recv_info, uint32_t timeout = 3);
    bool syncWrite(const ParamForSyncWriteInst_t &param_info);

    bool bulkRead(const ParamForBulkReadInst_t &param_info, RecvInfoFromStatusInst_t &recv_info, uint32_t timeout = 3);
    bool bulkWrite(const ParamForBulkWriteInst_t &param_info);

    uint8_t getLastStatusPacketError() const;
    lib_err_code_t getLastLibErrCode() const;

    void setLastLibErrCode(lib_err_code_t err_code);

  private:
    PortHandler *p_port_;
    dxl_t packet_;
    uint8_t last_status_packet_error_; 
    lib_err_code_t last_lib_err_code_;
  };
}


#endif /* DYNAMIXEL_MASTER_H_ */