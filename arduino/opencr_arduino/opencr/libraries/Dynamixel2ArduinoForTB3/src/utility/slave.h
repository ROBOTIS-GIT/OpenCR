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

#ifndef DYNAMIXEL_SLAVE_H_
#define DYNAMIXEL_SLAVE_H_

#include "protocol.h"
#include "port_handler.h"

namespace DYNAMIXEL{

#define TABLE_SIZE 255

class Slave;

typedef void (*userCallbackFunc)(Slave *slave, uint16_t addr, uint16_t length);

typedef struct ControlTableAddr{
  uint8_t data;
  bool is_protected;
} ControlTableAddr_t;

class Slave
{
  public:
    Slave(PortHandler &port, const uint16_t model_num, float protocol_ver = DXL_PACKET_VER_2_0);
    Slave(const uint16_t model_num, float protocol_ver = DXL_PACKET_VER_2_0);      

    void setWriteCallbackFunc(userCallbackFunc callback_func);
    void setReadCallbackFunc(userCallbackFunc callback_func);
    uint16_t getModelNumber() const;
    bool setID(uint8_t id);
    uint8_t getID() const;
    void setFirmwareVersion(uint8_t version);
    uint8_t getFirmwareVersion() const;
    bool setPort(PortHandler &port);
    bool setPortProtocolVersion(float version);
    float getPortProtocolVersion();

    void processPacket();

    //bool addControlTableItem(uint16_t addr, uint16_t length, callbackFunc callback);
    uint8_t setControlTable(uint16_t start_addr, uint16_t length, uint8_t* p_data);
    uint8_t setControlTable(uint16_t start_addr, uint8_t data);
    uint8_t setControlTable(uint16_t start_addr, uint16_t data);
    uint8_t setControlTable(uint16_t start_addr, uint32_t data);
    uint8_t setControlTable(uint16_t start_addr, uint64_t data);
    uint8_t setControlTable(uint16_t start_addr, int8_t data);
    uint8_t setControlTable(uint16_t start_addr, int16_t data);
    uint8_t setControlTable(uint16_t start_addr, int32_t data);
    uint8_t setControlTable(uint16_t start_addr, int64_t data);
    uint8_t setControlTable(uint16_t start_addr, float data);
    uint8_t setControlTable(uint16_t start_addr, double data);

    uint8_t getControlTable(uint16_t start_addr, uint16_t length, uint8_t* recv_buf);
    bool setAddrRemoteWriteProtected(uint16_t start_addr, uint16_t length, bool enable);
    uint16_t getAddrProtected(uint16_t start_addr, uint16_t length) const;
    
    uint8_t getLastStatusPacketError() const;
    lib_err_code_t getLastLibErrCode() const;

  private:
    PortHandler *p_port_;
    const uint16_t model_num_;
    uint8_t firmware_ver_;
    uint8_t id_;

    ControlTableAddr_t control_table_[TABLE_SIZE];
    userCallbackFunc user_write_callback_;
    userCallbackFunc user_read_callback_;

    dxl_t packet_;
    uint8_t last_status_packet_error_; 
    lib_err_code_t last_lib_err_code_;

    virtual bool processInstPing();
    virtual bool processInstRead();
    virtual bool processInstWrite();
    
    bool processInst(uint8_t inst_idx);
};

} // namespace DYNAMIXEL

#endif /* DYNAMIXEL_SLAVE_H_ */