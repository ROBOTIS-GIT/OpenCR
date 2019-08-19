#include "config.h"
#include "slave.h"

using namespace DYNAMIXEL;

static lib_err_code_t dxlMakePacketStatus1_0(dxl_t &packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length );
static lib_err_code_t dxlMakePacketStatus2_0(dxl_t &packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length );
static lib_err_code_t dxlTxPacketStatus(dxl_t &packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length);
static lib_err_code_t dxlMakePacketStatus(dxl_t &packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length );


Slave::Slave(PortHandler &port, const uint16_t model_num, float protocol_ver)
  : model_num_(model_num), firmware_ver_(1), id_(1),
    user_write_callback_(nullptr), user_read_callback_(nullptr),
    last_status_packet_error_(0), last_lib_err_code_(DXL_LIB_OK)
{
  setPort(port);
  dxlInit(&packet_, protocol_ver);
}

Slave::Slave(const uint16_t model_num, float protocol_ver)
  : model_num_(model_num), firmware_ver_(1), id_(1),
    user_write_callback_(nullptr), user_read_callback_(nullptr),
    last_status_packet_error_(0), last_lib_err_code_(DXL_LIB_OK)
{
  dxlInit(&packet_, protocol_ver);
}

void Slave::setWriteCallbackFunc(userCallbackFunc callback_func)
{
  user_write_callback_ = callback_func;
}

void Slave::setReadCallbackFunc(userCallbackFunc callback_func)
{
  user_read_callback_ = callback_func;
}

bool Slave::setPort(PortHandler &port)
{
  bool ret = setDxlPort(&packet_, &port);

  p_port_ = &port;

  return ret;
}

bool Slave::setPortProtocolVersion(float version)
{
  return dxlSetProtocolVersion(&packet_, version);
}

float Slave::getPortProtocolVersion()
{
  return dxlGetProtocolVersion(&packet_);
}

uint16_t Slave::getModelNumber() const
{
  return model_num_;
}

bool Slave::setID(uint8_t id)
{
  if(getPortProtocolVersion() == 1.0){
    if(id > 253){
      last_lib_err_code_ = DXL_LIB_ERROR_INVAILD_ID;
      return false;
    }
  }else{
    if(id > 252){
      last_lib_err_code_ = DXL_LIB_ERROR_INVAILD_ID;
      return false;
    }
  }

  id_ = id;
  dxlSetId(&packet_, id);

  return true;
}

uint8_t Slave::getID() const
{
  return id_;
}

void Slave::setFirmwareVersion(uint8_t version)
{
  firmware_ver_ = version;
}

uint8_t Slave::getFirmwareVersion() const
{
  return firmware_ver_;
}

void Slave::processPacket()
{
  last_lib_err_code_ = dxlRxPacket(&packet_);

  if(last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_INST){
    if(packet_.rx.id == id_){
      processInst(packet_.rx.cmd);
    }
  }
}

uint8_t Slave::setControlTable(uint16_t start_addr, uint16_t length, uint8_t* p_data)
{
  if(TABLE_SIZE < start_addr + length){
    last_lib_err_code_ = DXL_LIB_ERROR_INVAILD_ADDR;
    return DXL_ERR_DATA_RANGE;
  }

  if(length == 0 || TABLE_SIZE < length){
    last_lib_err_code_ = DXL_LIB_ERROR_ADDR_LENGTH;
    return DXL_ERR_DATA_LENGTH;
  }

  if(p_data == nullptr){
    last_lib_err_code_ = DXL_LIB_ERROR_NULLPTR;
    return DXL_ERR_ACCESS;    
  }

  for(uint16_t i=0; i < length; i++){
    control_table_[start_addr+i].data = p_data[i];
  }
  
  return DXL_ERR_NONE;
}

uint8_t Slave::setControlTable(uint16_t start_addr, uint8_t data)
{
  uint16_t length = sizeof(data);
  uint8_t* p_data = (uint8_t*)&data;

  return setControlTable(start_addr, length, p_data);
}

uint8_t Slave::setControlTable(uint16_t start_addr, uint16_t data)
{
  uint16_t length = sizeof(data);
  uint8_t* p_data = (uint8_t*)&data;

  return setControlTable(start_addr, length, p_data);
}

uint8_t Slave::setControlTable(uint16_t start_addr, uint32_t data)
{
  uint16_t length = sizeof(data);
  uint8_t* p_data = (uint8_t*)&data;

  return setControlTable(start_addr, length, p_data);
}

uint8_t Slave::setControlTable(uint16_t start_addr, uint64_t data)
{
  uint16_t length = sizeof(data);
  uint8_t* p_data = (uint8_t*)&data;

  return setControlTable(start_addr, length, p_data);
}

uint8_t Slave::setControlTable(uint16_t start_addr, int8_t data)
{
  uint16_t length = sizeof(data);
  uint8_t* p_data = (uint8_t*)&data;

  return setControlTable(start_addr, length, p_data);
}

uint8_t Slave::setControlTable(uint16_t start_addr, int16_t data)
{
  uint16_t length = sizeof(data);
  uint8_t* p_data = (uint8_t*)&data;

  return setControlTable(start_addr, length, p_data);
}

uint8_t Slave::setControlTable(uint16_t start_addr, int32_t data)
{
  uint16_t length = sizeof(data);
  uint8_t* p_data = (uint8_t*)&data;

  return setControlTable(start_addr, length, p_data);
}

uint8_t Slave::setControlTable(uint16_t start_addr, int64_t data)
{
  uint16_t length = sizeof(data);
  uint8_t* p_data = (uint8_t*)&data;

  return setControlTable(start_addr, length, p_data);
}

uint8_t Slave::setControlTable(uint16_t start_addr, float data)
{
  uint16_t length = sizeof(data);
  uint8_t* p_data = (uint8_t*)&data;

  return setControlTable(start_addr, length, p_data);
}

uint8_t Slave::setControlTable(uint16_t start_addr, double data)
{
  uint16_t length = sizeof(data);
  uint8_t* p_data = (uint8_t*)&data;

  return setControlTable(start_addr, length, p_data);
}


uint8_t Slave::getControlTable(uint16_t start_addr, uint16_t length, uint8_t* recv_buf)
{
  if(TABLE_SIZE < start_addr + length){
    last_lib_err_code_ = DXL_LIB_ERROR_INVAILD_ADDR;
    return DXL_ERR_DATA_RANGE;
  }

  if(length == 0 || TABLE_SIZE < length){
    last_lib_err_code_ = DXL_LIB_ERROR_ADDR_LENGTH;
    return DXL_ERR_DATA_LENGTH;
  }

  if(recv_buf == nullptr){
    last_lib_err_code_ = DXL_LIB_ERROR_NULLPTR;
    return DXL_ERR_ACCESS;    
  }

  for(uint16_t i=0; i < length; i++){
    recv_buf[i] = control_table_[start_addr+i].data;
  }

  return DXL_ERR_NONE;
}

bool Slave::setAddrRemoteWriteProtected(uint16_t start_addr, uint16_t length, bool enable)
{
  bool ret = false;

  for(uint16_t i=start_addr; i<start_addr+length && i<TABLE_SIZE; i++){
    control_table_[i].is_protected = enable;
    ret = true;
  }

  return ret;  
}

uint16_t Slave::getAddrProtected(uint16_t start_addr, uint16_t length) const
{
  uint16_t i, protected_cnt = 0;

  for(i=start_addr; i<start_addr+length && i<TABLE_SIZE; i++){
    if( control_table_[i].is_protected == true)
      protected_cnt++;
  }

  return protected_cnt;
}

uint8_t Slave::getLastStatusPacketError() const
{
  return last_status_packet_error_;
}

lib_err_code_t Slave::getLastLibErrCode() const
{
  return last_lib_err_code_;
}






bool Slave::processInstPing()
{
  bool ret = false;
  uint16_t param_length = 0;
  uint8_t *p_param_data;

  if(packet_.rx.id == DXL_BROADCAST_ID){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  p_param_data = &packet_.tx.data[PKT_STATUS_PARAM_IDX];
  if(packet_.packet_ver == DXL_PACKET_VER_2_0){
    p_param_data[param_length++] = (uint8_t)(model_num_ >> 0);
    p_param_data[param_length++] = (uint8_t)(model_num_ >> 8);
    p_param_data[param_length++] = (uint8_t)firmware_ver_;
  }

  last_lib_err_code_ = dxlTxPacketStatus(packet_, id_, 0, p_param_data, param_length);

  if(last_lib_err_code_ == DXL_LIB_OK)
    ret = true;

  return ret;
}


bool Slave::processInstRead()
{
  bool ret = false;
  uint16_t addr;
  uint16_t length = 0;
  uint8_t process_ret = DXL_ERR_NONE;
  uint8_t *p_param_data;

  if(packet_.rx.id == DXL_BROADCAST_ID){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  if(packet_.packet_ver == DXL_PACKET_VER_1_0 )
  {
    if( packet_.rx.param_length != 2){
      last_lib_err_code_ = DXL_LIB_ERROR_LENGTH;
      return false;
    }

    addr   = packet_.rx.p_param[0];
    length = packet_.rx.p_param[1];

    if( length > 0xFF - 2){
      dxlTxPacketStatus(packet_, id_, DXL_ERR_DATA_LENGTH, nullptr, 0);
      last_lib_err_code_ = DXL_LIB_ERROR_LENGTH;
      return false;
    }
  }else{
    if( packet_.rx.param_length != 4){
      last_lib_err_code_ = DXL_LIB_ERROR_LENGTH;
      return false;
    } 

    addr   = ((uint16_t)packet_.rx.p_param[1]<<8) | (uint16_t)packet_.rx.p_param[0];
    length = ((uint16_t)packet_.rx.p_param[3]<<8) | (uint16_t)packet_.rx.p_param[2];
  }

  if(length > DXL_BUF_LENGTH){
    dxlTxPacketStatus(packet_, id_, DXL_ERR_DATA_LENGTH, nullptr, 0);
    last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
    return false;
  }

  if(user_read_callback_ != nullptr){
    user_read_callback_(this, addr, length);
  }

  p_param_data = &packet_.tx.data[PKT_STATUS_PARAM_IDX];
  process_ret = getControlTable(addr, length, p_param_data);

  last_lib_err_code_ = dxlTxPacketStatus(packet_, id_, process_ret, p_param_data, length);
  if(last_lib_err_code_ == DXL_LIB_OK)
    ret = true;

  return ret;
}


bool Slave::processInstWrite()
{
  bool ret = false;
  uint16_t addr;
  uint16_t length = 0;
  uint8_t  *p_data;
  uint8_t process_ret = DXL_ERR_NONE;

  if(packet_.rx.id == DXL_BROADCAST_ID){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }
    
  if(packet_.packet_ver == DXL_PACKET_VER_1_0 ){
    addr   =  packet_.rx.p_param[0];
    p_data = &packet_.rx.p_param[1];

    if(packet_.rx.param_length > 1 ){
      length = packet_.rx.param_length - 1;
    }else{
      dxlTxPacketStatus(packet_, id_, DXL_ERR_DATA_LENGTH, nullptr, 0);
      last_lib_err_code_ = DXL_LIB_ERROR_LENGTH;
      return false;
    }

    if( length > 0xFF - 2 ){
      dxlTxPacketStatus(packet_, id_, DXL_ERR_DATA_LENGTH, nullptr, 0);
      last_lib_err_code_ = DXL_LIB_ERROR_LENGTH;
      return false;
    }
  }else{
    addr   = ((uint16_t)packet_.rx.p_param[1]<<8) | (uint16_t)packet_.rx.p_param[0];
    p_data = &packet_.rx.p_param[2];

    if(packet_.rx.param_length > 2 ){
      length = packet_.rx.param_length - 2;
    }else{
      dxlTxPacketStatus(packet_, id_, DXL_ERR_DATA_LENGTH, nullptr, 0);
      last_lib_err_code_ = DXL_LIB_ERROR_LENGTH;
      return false;
    }    
  }

  if(length > DXL_BUF_LENGTH){
    dxlTxPacketStatus(packet_, id_, DXL_ERR_DATA_LENGTH, nullptr, 0);
    last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
    return false;
  }

  if(getAddrProtected(addr, length) > 0){
    process_ret = DXL_ERR_ACCESS;
  }else{
    process_ret = setControlTable(addr, length, p_data);
  }

  if(process_ret == DXL_ERR_NONE){
    ret = true;
    if(user_write_callback_ != nullptr){
      user_write_callback_(this, addr, length);
    }
  }

  dxlTxPacketStatus(packet_, id_, process_ret, nullptr, 0);

  return ret;
}


bool Slave::processInst(uint8_t inst_idx)
{
  bool ret = false;

  switch(inst_idx)
  {
    case INST_PING:
      ret = processInstPing();
      break;

    case INST_READ:
      ret = processInstRead();
      break;

    case INST_WRITE:
      ret = processInstWrite();
      break;

    default:
      last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORTED;
      break;  
  }

  return ret;
}







static lib_err_code_t dxlTxPacketStatus(dxl_t &packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length )
{
  lib_err_code_t ret;

  ret = dxlMakePacketStatus(packet, id, error, p_data, length);
  if(ret == DXL_LIB_OK) {
    dxlTxWrite(&packet, packet.tx.data, packet.tx.packet_length);
  }

  return ret;
}

static lib_err_code_t dxlMakePacketStatus(dxl_t &packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length )
{
  lib_err_code_t ret;

  if(packet.packet_ver == DXL_PACKET_VER_1_0){
    ret = dxlMakePacketStatus1_0(packet, id, error, p_data, length);
  }else{
    ret = dxlMakePacketStatus2_0(packet, id, error, p_data, length);
  }

  return ret;
}

static lib_err_code_t dxlMakePacketStatus1_0(dxl_t &packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length )
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t i = 0;
  uint16_t packet_length;
  uint8_t  check_sum;


  if(length > DXL_BUF_LENGTH){
    return DXL_LIB_ERROR_BUFFER_OVERFLOW;
  }

  if(length > 0xFF){
    return DXL_LIB_ERROR_LENGTH;
  }

  check_sum = 0;
  packet_length = length + 2; // param_length + Instruction + CheckSum

  packet.tx.data[PKT_1_0_HDR_1_IDX] = 0xFF;
  packet.tx.data[PKT_1_0_HDR_2_IDX] = 0xFF;
  packet.tx.data[PKT_1_0_ID_IDX]    = id;
  packet.tx.data[PKT_1_0_ERROR_IDX] = error;

  check_sum += id;
  check_sum += packet_length;
  check_sum += error;

  for (i=0; i<length; i++)
  {
    packet.tx.data[PKT_1_0_STATUS_PARAM_IDX + i] = p_data[i];
    check_sum += p_data[i];
  }

  packet.tx.data[PKT_1_0_LEN_IDX] = packet_length;
  packet.tx.data[PKT_1_0_ERROR_IDX + packet_length - 1] = ~(check_sum);

  return ret;
}

static lib_err_code_t dxlMakePacketStatus2_0(dxl_t &packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length )
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t i = 0;
  uint16_t packet_length;
  uint16_t stuff_length;
  uint16_t crc;

  if(length > DXL_BUF_LENGTH){
    return DXL_LIB_ERROR_BUFFER_OVERFLOW;
  }

  packet_length = length + 4; // param_length + Instruction + Error + CRC_L + CRC_H

  packet.tx.data[PKT_HDR_1_IDX] = 0xFF;
  packet.tx.data[PKT_HDR_2_IDX] = 0xFF;
  packet.tx.data[PKT_HDR_3_IDX] = 0xFD;
  packet.tx.data[PKT_RSV_IDX]   = 0x00;
  packet.tx.data[PKT_ID_IDX]    = id;
  packet.tx.data[PKT_INST_IDX]  = INST_STATUS;
  packet.tx.data[PKT_ERROR_IDX] = error;

  for (i=0; i<length; i++)
  {
    packet.tx.data[PKT_STATUS_PARAM_IDX + i] = p_data[i];
  }

  stuff_length = dxlAddStuffing(&packet, &packet.tx.data[PKT_INST_IDX], length + 2); // + instruction + error
  packet_length += stuff_length;

  packet.tx.data[PKT_LEN_L_IDX] = packet_length >> 0;
  packet.tx.data[PKT_LEN_H_IDX] = packet_length >> 8;

  crc = 0;
  for (i=0; i<packet_length+7-2; i++)
  {
    dxlUpdateCrc(&crc, packet.tx.data[i]);
  }

  packet.tx.data[PKT_INST_IDX + packet_length - 2] = crc >> 0;
  packet.tx.data[PKT_INST_IDX + packet_length - 1] = crc >> 8;

  packet.tx.packet_length = packet_length + 7;

  return ret;
}