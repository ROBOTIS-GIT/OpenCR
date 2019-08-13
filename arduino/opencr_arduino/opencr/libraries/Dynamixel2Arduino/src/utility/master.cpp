#include "master.h"


using namespace DYNAMIXEL;

Master::Master(PortHandler &port, float protocol_ver)
  : last_status_packet_error_(0), last_lib_err_code_(DXL_LIB_OK)
{
  setPort(port);
  dxlInit(&packet_, protocol_ver);
}

Master::Master(float protocol_ver)
  : last_status_packet_error_(0), last_lib_err_code_(DXL_LIB_OK)
{
  dxlInit(&packet_, protocol_ver);
}

bool Master::setPortProtocolVersion(float version)
{
  return dxlSetProtocolVersion(&packet_, version);
}

float Master::getPortProtocolVersion()
{
  return dxlGetProtocolVersion(&packet_);
}

bool Master::setPort(PortHandler &port)
{
  bool ret = setDxlPort(&packet_, &port);

  p_port_ = &port;

  return ret;
}

uint8_t Master::ping(uint8_t id, XelInfoFromPing_t *recv_info_array, uint8_t recv_array_cnt, uint32_t timeout)
{
  uint8_t recv_id_cnt = 0;
  uint32_t pre_time_ms, pre_time_us;

  if(recv_info_array == nullptr){
    last_lib_err_code_ = DXL_LIB_ERROR_NULLPTR;
    return 0;
  }

  if(recv_array_cnt == 0){
    last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
    return 0;    
  }

  if (p_port_->getOpenState() != true) {
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return 0;
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, id, INST_PING, nullptr, 0);
  if(last_lib_err_code_ != DXL_LIB_OK)
    return 0;
  packet_.tx_time = micros() - pre_time_us;

  pre_time_ms = millis();
  pre_time_us = micros();
  while(recv_id_cnt < recv_array_cnt)
  {
    last_lib_err_code_ = dxlRxPacket(&packet_);
    if (last_lib_err_code_ == DXL_LIB_OK 
        && packet_.rx.type == RX_PACKET_TYPE_STATUS) {
      packet_.rx_time = micros() - pre_time_us;
      pre_time_ms     = millis();

      recv_info_array[recv_id_cnt].id = packet_.rx.id;
      if(getPortProtocolVersion() == DXL_PACKET_VER_2_0) {
        recv_info_array[recv_id_cnt].model_number     = packet_.rx.p_param[0]<<0;
        recv_info_array[recv_id_cnt].model_number    |= packet_.rx.p_param[1]<<8;
        recv_info_array[recv_id_cnt].firmware_version = packet_.rx.p_param[2];
      }
      recv_id_cnt++;

      if (id != DXL_BROADCAST_ID) {
        last_lib_err_code_ = DXL_LIB_OK;
        break;
      }
    }

    if (millis()-pre_time_ms >= timeout) {
      last_lib_err_code_ = DXL_LIB_ERROR_TIMEOUT;
      break;
    }
  }
  return recv_id_cnt;
}

bool Master::ping(uint8_t id, RecvInfoFromPing_t &recv_info, uint32_t timeout)
{
  bool ret = false;

  recv_info.id_count = ping(id, recv_info.xel, sizeof(recv_info.xel)/sizeof(XelInfoFromPing_t), timeout);
  if(recv_info.id_count > 0)
    ret = true;

  return ret;
}

int32_t Master::read(uint8_t id, uint16_t addr, uint16_t addr_length,
 uint8_t *p_recv_buf, uint16_t recv_buf_length, uint32_t timeout)
{
  uint32_t pre_time_us, pre_time_ms;
  int32_t i, recv_param_len = -1;
  uint8_t tx_param[4];

  if (id == DXL_BROADCAST_ID) {
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return -1;
  }

  if(addr_length == 0) {
    last_lib_err_code_ = DXL_LIB_ERROR_ADDR_LENGTH;
    return -1;
  }
    
  if (p_port_->getOpenState() != true) {
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return -1;
  }
    
  // Send Read Instruction 
  if (packet_.packet_ver == DXL_PACKET_VER_1_0 ) {
    tx_param[0] = addr;
    tx_param[1] = addr_length;
  }else{
    tx_param[0] = addr >> 0;
    tx_param[1] = addr >> 8;
    tx_param[2] = addr_length >> 0;
    tx_param[3] = addr_length >> 8;
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, id, INST_READ, tx_param, 4);
  packet_.tx_time = micros() - pre_time_us;

  pre_time_ms = millis();
  pre_time_us = micros();

  // Receive Status Packet  
  while(1) {
    last_lib_err_code_ = dxlRxPacket(&packet_);

    if (last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_STATUS) {
      pre_time_ms = millis();
      packet_.rx_time = micros() - pre_time_us;
      recv_param_len = packet_.rx.param_length;
      if(recv_param_len > recv_buf_length) {
        recv_param_len = recv_buf_length;
      }

      for (i=0; i<recv_param_len; i++)
      {
        p_recv_buf[i] = packet_.rx.p_param[i];
      }
      last_status_packet_error_ = packet_.rx.error;

      break;
    }else if (last_lib_err_code_ != DXL_LIB_PROCEEDING){
      break;
    }

    if (millis()-pre_time_ms >= timeout) {
      last_lib_err_code_ = DXL_LIB_ERROR_TIMEOUT;
      break;
    }
  }

  return recv_param_len;
}

bool Master::write(uint8_t id, uint16_t addr, 
  const uint8_t *p_data, uint16_t data_length, uint32_t timeout)
{
  bool ret = false;
  uint32_t pre_time_us, pre_time_ms;
  
  if(writeNoResp(id, addr, p_data, data_length) == false){
    return ret;
  }
    
  pre_time_ms = millis();
  pre_time_us = micros();
  while(1)
  {
    last_lib_err_code_ = dxlRxPacket(&packet_);
    if (last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_STATUS) {
      pre_time_ms = millis();
      packet_.rx_time = micros() - pre_time_us;
      last_status_packet_error_ = packet_.rx.error;
      ret = true;
      break;
    } else if (last_lib_err_code_ != DXL_LIB_PROCEEDING) {
      break;
    }

    if (millis()-pre_time_ms >= timeout){
      last_lib_err_code_ = DXL_LIB_ERROR_TIMEOUT;
      break;
    }
  }

  return ret;
}

bool Master::writeNoResp(uint8_t id, uint16_t addr, const uint8_t *p_data, uint16_t data_length)
{
  bool ret = false;
  uint32_t i, pre_time_us;
  uint16_t tx_length = 0;
  uint8_t *p_tx_data;

  if(p_data == nullptr){
    last_lib_err_code_ = DXL_LIB_ERROR_NULLPTR;
    return false;    
  }

  if (id == DXL_BROADCAST_ID) {
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  if(data_length == 0) {
    last_lib_err_code_ = DXL_LIB_ERROR_ADDR_LENGTH;
    return false;
  }

  if (p_port_->getOpenState() != true)
  {
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }  

  if (packet_.packet_ver == DXL_PACKET_VER_1_0 )
  {
    if ((size_t)(PKT_1_0_INST_PARAM_IDX + 1 + data_length + 1) > sizeof(packet_.tx.data)){
      last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      return false;
    }
    p_tx_data = &packet_.tx.data[PKT_1_0_INST_PARAM_IDX];
    p_tx_data[tx_length++] = addr;
    for (i=0; i<data_length; i++)
    {
      p_tx_data[tx_length++] = p_data[i];
    }
  }
  else
  {
    if ((size_t)(PKT_INST_PARAM_IDX + 2 + data_length + 2) > sizeof(packet_.tx.data)){
      last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      return false;
    }
    p_tx_data = &packet_.tx.data[PKT_INST_PARAM_IDX];
    p_tx_data[tx_length++] = addr >> 0;
    p_tx_data[tx_length++] = addr >> 8;
    for (i=0; i<data_length; i++)
    {
      p_tx_data[tx_length++] = p_data[i];
    }
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, id, INST_WRITE, p_tx_data, tx_length);
  if(last_lib_err_code_ != DXL_LIB_OK)
    return false;
  packet_.tx_time = micros() - pre_time_us;
  ret = true;

  return ret;
}

bool Master::factoryReset(uint8_t id, uint8_t option, uint32_t timeout)
{
  bool ret = false;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;
  uint8_t tx_param[1];

  if (id == DXL_BROADCAST_ID){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  if (p_port_->getOpenState() != true){
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }
  
  tx_param[0] = option;

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, id, INST_RESET, tx_param, 1);
  packet_.tx_time = micros() - pre_time_us;

  pre_time_ms = millis();
  pre_time_us = micros();
  while(1)
  {
    last_lib_err_code_ = dxlRxPacket(&packet_);
    if (last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_STATUS) {
      pre_time_ms = millis();
      packet_.rx_time = micros() - pre_time_us;
      last_status_packet_error_ = packet_.rx.error;
      ret = true;
      break;
    }else if (last_lib_err_code_ != DXL_LIB_PROCEEDING){
      break;
    }

    if (millis()-pre_time_ms >= timeout){
      last_lib_err_code_ = DXL_LIB_ERROR_TIMEOUT;
      break;
    }
  }
  return ret;
}

bool Master::reboot(uint8_t id, uint32_t timeout)
{ 
  bool ret = false;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;
  uint8_t tx_param[1];

  if (id == DXL_BROADCAST_ID){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  if (p_port_->getOpenState() != true){
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, id, INST_REBOOT, tx_param, 1);
  packet_.tx_time = micros() - pre_time_us;

  pre_time_ms = millis();
  pre_time_us = micros();
  while(1)
  {
    last_lib_err_code_ = dxlRxPacket(&packet_);
    if (last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_STATUS) {
      pre_time_ms = millis();
      packet_.rx_time = micros() - pre_time_us;
      last_status_packet_error_ = packet_.rx.error;
      ret = true;
      break;
    }else if (last_lib_err_code_ != DXL_LIB_PROCEEDING){
      break;
    }

    if (millis()-pre_time_ms >= timeout){
      last_lib_err_code_ = DXL_LIB_ERROR_TIMEOUT;
      break;
    }
  }
  return ret;
}

bool Master::syncRead(const ParamForSyncReadInst_t &param_info, RecvInfoFromStatusInst_t &recv_info, uint32_t timeout)
{
  bool ret = false;
  uint32_t i, pre_time_us, pre_time_ms;
  uint16_t tx_length = 0;
  uint8_t *p_tx_data;

  if (packet_.packet_ver == DXL_PACKET_VER_1_0 ){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORTED;
    return false;
  }

  if(param_info.id_count > DXL_MAX_NODE
     || (size_t)(PKT_INST_PARAM_IDX + param_info.id_count * 5 + 2) > sizeof(packet_.tx.data)){
    last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
    return false;
  }

  if (p_port_->getOpenState() != true){
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }

  p_tx_data = &packet_.tx.data[PKT_INST_PARAM_IDX];

  p_tx_data[tx_length++] = param_info.addr >> 0;
  p_tx_data[tx_length++] = param_info.addr >> 8;
  p_tx_data[tx_length++] = param_info.length >> 0;
  p_tx_data[tx_length++] = param_info.length >> 8;

  for( i=0; i<param_info.id_count; i++)
  {
    p_tx_data[tx_length++] = param_info.xel[i].id;
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, DXL_BROADCAST_ID, INST_SYNC_READ, p_tx_data, tx_length);
  if(last_lib_err_code_ != DXL_LIB_OK)
    return false;
  packet_.tx_time = micros() - pre_time_us;

  recv_info.id_count = 0;
  pre_time_ms = millis();
  pre_time_us = micros();
  while(1)
  {
    last_lib_err_code_ = dxlRxPacket(&packet_);
    if (last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_STATUS) {
      pre_time_ms = millis();
      packet_.rx_time = micros() - pre_time_us;
    
      recv_info.xel[recv_info.id_count].id     = packet_.rx.id;
      recv_info.xel[recv_info.id_count].error  = packet_.rx.error;
      recv_info.xel[recv_info.id_count].length = packet_.rx.param_length;

      for (i=0; i<packet_.rx.param_length; i++)
      {
        recv_info.xel[recv_info.id_count].data[i] = packet_.rx.p_param[i];
      }

      recv_info.id_count++;

      if (recv_info.id_count >= param_info.id_count){
        ret = true;
        break;
      }
    }

    if (millis()-pre_time_ms >= timeout){
      last_lib_err_code_ = DXL_LIB_ERROR_TIMEOUT;
      return false;
    }
  }
  
  return ret;
}


bool Master::syncWrite(const ParamForSyncWriteInst_t &param_info)
{
  bool ret = false;
  uint32_t i, j, pre_time_us;
  uint16_t tx_length = 0;
  uint8_t *p_tx_data;

  if (p_port_->getOpenState() != true){
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }

  if (packet_.packet_ver == DXL_PACKET_VER_1_0 ){
    if(param_info.id_count > DXL_MAX_NODE
       || (size_t)(PKT_1_0_INST_PARAM_IDX + 2 + param_info.length + 1) > sizeof(packet_.tx.data)){
      last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      return false;
    }
    p_tx_data = &packet_.tx.data[PKT_1_0_INST_PARAM_IDX];
    p_tx_data[tx_length++] = param_info.addr;
    p_tx_data[tx_length++] = param_info.length;
  }else{
    if(param_info.id_count > DXL_MAX_NODE
       || (size_t)(PKT_INST_PARAM_IDX + 4 + param_info.length + 2) > sizeof(packet_.tx.data)){
      last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      return false;
    }
    p_tx_data = &packet_.tx.data[PKT_INST_PARAM_IDX];
    p_tx_data[tx_length++] = param_info.addr >> 0;
    p_tx_data[tx_length++] = param_info.addr >> 8;
    p_tx_data[tx_length++] = param_info.length >> 0;
    p_tx_data[tx_length++] = param_info.length >> 8;
  }

  for( i=0; i<param_info.id_count; i++)
  {
    p_tx_data[tx_length++] = param_info.xel[i].id;
    for (j=0; j<param_info.length; j++)
    {
      p_tx_data[tx_length++] = param_info.xel[i].data[j];
    }
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, DXL_BROADCAST_ID, INST_SYNC_WRITE, p_tx_data, tx_length);
  if(last_lib_err_code_ != DXL_LIB_OK)
    return false;
  packet_.tx_time = micros() - pre_time_us;
  ret = true;

  return ret;
}

bool Master::bulkRead(const ParamForBulkReadInst_t &param_info, RecvInfoFromStatusInst_t &recv_info, uint32_t timeout)
{
  bool ret = false;
  uint32_t i, pre_time_us, pre_time_ms;
  uint16_t tx_length = 0;
  uint8_t *p_tx_data;

  if (p_port_->getOpenState() != true){
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }

  if(getPortProtocolVersion() == DXL_PACKET_VER_1_0){
    if(param_info.id_count > DXL_MAX_NODE
       || (size_t)(PKT_1_0_INST_PARAM_IDX + 1 + param_info.id_count * 3 + 1) > sizeof(packet_.tx.data)){
      last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      return false;
    }
    p_tx_data = &packet_.tx.data[PKT_1_0_INST_PARAM_IDX];
    p_tx_data[tx_length++] = 0x00;
    for( i=0; i<param_info.id_count; i++)
    {
      p_tx_data[tx_length++] = param_info.xel[i].length;
      p_tx_data[tx_length++] = param_info.xel[i].id;
      p_tx_data[tx_length++] = param_info.xel[i].addr;
    }
  }else{
    if(param_info.id_count > DXL_MAX_NODE
       || (size_t)(PKT_INST_PARAM_IDX + param_info.id_count * 5 + 2) > sizeof(packet_.tx.data)){
      last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      return false;
    }
    p_tx_data = &packet_.tx.data[PKT_INST_PARAM_IDX];
    for( i=0; i<param_info.id_count; i++)
    {
      p_tx_data[tx_length++] = param_info.xel[i].id;
      p_tx_data[tx_length++] = param_info.xel[i].addr >> 0;
      p_tx_data[tx_length++] = param_info.xel[i].addr >> 8;
      p_tx_data[tx_length++] = param_info.xel[i].length >> 0;
      p_tx_data[tx_length++] = param_info.xel[i].length >> 8;
    }
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, DXL_BROADCAST_ID, INST_BULK_READ, p_tx_data, tx_length);
  if(last_lib_err_code_ != DXL_LIB_OK)
    return false;
  packet_.tx_time = micros() - pre_time_us;

  recv_info.id_count = 0;
  pre_time_ms = millis();
  pre_time_us = micros();
  while(1)
  {
    last_lib_err_code_ = dxlRxPacket(&packet_);
    if (last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_STATUS) {
      pre_time_ms = millis();
      packet_.rx_time = micros() - pre_time_us;

      recv_info.xel[recv_info.id_count].id     = packet_.rx.id;
      recv_info.xel[recv_info.id_count].error  = packet_.rx.error;
      recv_info.xel[recv_info.id_count].length = packet_.rx.param_length;

      for (i=0; i<packet_.rx.param_length; i++)
      {
        recv_info.xel[recv_info.id_count].data[i] = packet_.rx.p_param[i];
      }

      recv_info.id_count++;

      if (recv_info.id_count >= param_info.id_count){
        ret = true;
        break;
      }
    }
      
    if (millis()-pre_time_ms >= timeout){
      last_lib_err_code_ = DXL_LIB_ERROR_TIMEOUT;
      return false;
    }
  } 
  return ret;
}

bool Master::bulkWrite(const ParamForBulkWriteInst_t &param_info)
{
  bool ret = false;
  uint32_t i, j, pre_time_us;
  uint16_t tx_length = 0, total_data_length = 0;
  uint8_t *p_tx_data;

  if (packet_.packet_ver == DXL_PACKET_VER_1_0 ){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORTED;
    return false;
  }

  for( i=0; i<param_info.id_count; i++)
  {
    total_data_length += param_info.xel[i].length;
  }

  if(param_info.id_count > DXL_MAX_NODE
     || (size_t)(PKT_INST_PARAM_IDX + param_info.id_count * 5 + total_data_length +2) > sizeof(packet_.tx.data)){
    last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
    return false;
  }

  if (p_port_->getOpenState() != true){
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }

  p_tx_data = &packet_.tx.data[PKT_INST_PARAM_IDX];
  for( i=0; i<param_info.id_count; i++)
  {
    p_tx_data[tx_length++] = param_info.xel[i].id;
    p_tx_data[tx_length++] = param_info.xel[i].addr >> 0;
    p_tx_data[tx_length++] = param_info.xel[i].addr >> 8;
    p_tx_data[tx_length++] = param_info.xel[i].length >> 0;
    p_tx_data[tx_length++] = param_info.xel[i].length >> 8;
    for (j=0; j<param_info.xel[i].length; j++)
    {
      p_tx_data[tx_length++] = param_info.xel[i].data[j];
    }
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, DXL_BROADCAST_ID, INST_BULK_WRITE, p_tx_data, tx_length);
  if(last_lib_err_code_ != DXL_LIB_OK)
    return false;
  packet_.tx_time = micros() - pre_time_us;  
  ret = true;

  return ret;
}

uint8_t Master::getLastStatusPacketError() const
{
  return last_status_packet_error_;
}

lib_err_code_t Master::getLastLibErrCode() const
{
  return last_lib_err_code_;
}

