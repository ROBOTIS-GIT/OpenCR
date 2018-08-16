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

/* Authors: Taehun Lim (Darby) */

#include "../../include/dynamixel_workbench_toolbox/dynamixel_driver.h"

DynamixelDriver::DynamixelDriver() : tools_cnt_(0), sync_write_handler_cnt_(0), sync_read_handler_cnt_(0) {}

DynamixelDriver::~DynamixelDriver()
{
  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].dxl_info_cnt_; j++)
    {
      writeRegister(tools_[i].dxl_info_[j].id, "Torque_Enable", false);
    }
  }

  portHandler_->closePort();
}

void DynamixelDriver::initDXLinfo(void)
{
  for (int i = 0; i <= tools_cnt_; i++)
  {
    tools_[i].dxl_info_cnt_ = 0;
  }
}

void DynamixelDriver::setTools(uint16_t model_number, uint8_t id)
{
  if (tools_cnt_ == 0)
  {
    initDXLinfo();
    tools_[tools_cnt_].addTool(model_number, id);
  }
  else
  {
    if (!strncmp(tools_[tools_cnt_-1].dxl_info_[0].model_name, findModelName(model_number), strlen(findModelName(model_number))))
    {
      tools_[--tools_cnt_].addDXL(model_number, id);
    }
    else
    {
      tools_[tools_cnt_].addTool(model_number, id);
    }
  }

  tools_cnt_++;
}

bool DynamixelDriver::init(const char *device_name, uint32_t baud_rate)
{
  if (setPortHandler(device_name) == false)
    return false;

  if (setBaudrate(baud_rate) == false)
    return false;

  if (setPacketHandler() == false)
    return false;

  return true;
}

bool DynamixelDriver::setPortHandler(const char *device_name)
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name);

  if (portHandler_->openPort())
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool DynamixelDriver::setPacketHandler(void)
{
  packetHandler_1 = dynamixel::PacketHandler::getPacketHandler(1.0);
  packetHandler_2 = dynamixel::PacketHandler::getPacketHandler(2.0);

  if (packetHandler_1->getProtocolVersion() == 1.0 && packetHandler_2->getProtocolVersion() == 2.0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool DynamixelDriver::setPacketHandler(float protocol_version)
{
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  if (packetHandler_->getProtocolVersion() == protocol_version)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool DynamixelDriver::setBaudrate(uint32_t baud_rate)
{
  if (portHandler_->setBaudRate(baud_rate))
  {
    return true;
  }
  else
  {
    return false;
  }
}

float DynamixelDriver::getProtocolVersion(void)
{
  return packetHandler_->getProtocolVersion();
}

int DynamixelDriver::getBaudrate(void)
{
  return portHandler_->getBaudRate();
}

char *DynamixelDriver::getModelName(uint8_t id)
{
  uint8_t factor = getToolsFactor(id);

  for (int i = 0; i < tools_[factor].dxl_info_cnt_; i++)
  {
    if (tools_[factor].dxl_info_[i].id == id)
      return tools_[factor].dxl_info_[i].model_name;
  }
}

uint16_t DynamixelDriver::getModelNum(uint8_t id)
{
  uint8_t factor = getToolsFactor(id);

  for (int i = 0; i < tools_[factor].dxl_info_cnt_; i++)
  {
    if (tools_[factor].dxl_info_[i].id == id)
      return tools_[factor].dxl_info_[i].model_num;
  }
}

ControlTableItem* DynamixelDriver::getControlItemPtr(uint8_t id)
{
  uint8_t factor = getToolsFactor(id);

  return tools_[factor].getControlItemPtr();
}

uint8_t DynamixelDriver::getTheNumberOfItem(uint8_t id)
{
  uint8_t factor = getToolsFactor(id);

  return tools_[factor].getTheNumberOfItem();
}

bool DynamixelDriver::scan(uint8_t *get_id, uint8_t *get_id_num, uint8_t range)
{
  uint8_t id = 0;
  uint8_t id_cnt = 0;
  uint16_t model_number = 0;
  float protocol_version = 2.0;

  tools_cnt_ = 0;

  for (id = 1; id <= range; id++)
  {
    if (packetHandler_1->ping(portHandler_, id, &model_number) == COMM_SUCCESS)
    {
      get_id[id_cnt] = id;
      setTools(model_number, id);
      id_cnt++;
      protocol_version = 1.0;
    }
  }

  for (id = 1; id <= range; id++)
  {
    if (packetHandler_2->ping(portHandler_, id, &model_number) == COMM_SUCCESS)
    {
      get_id[id_cnt] = id;
      setTools(model_number, id);
      id_cnt++;
      protocol_version = 2.0;
    }
  }

  if (id_cnt == 0)
  {
    return false;
  }
  else
  {
    *get_id_num = id_cnt;
    if (setPacketHandler(protocol_version) == false)
      return false;

    return true;
  }
}

bool DynamixelDriver::ping(uint8_t id, uint16_t *get_model_number)
{
  uint16_t model_number = 0;
  float protocol_version = 2.0;

  if (packetHandler_1->ping(portHandler_, id, &model_number) == COMM_SUCCESS)
  {
    setTools(model_number, id);
    protocol_version = 1.0;
  }
  else if (packetHandler_2->ping(portHandler_, id, &model_number) == COMM_SUCCESS)
  {
    setTools(model_number, id);
    protocol_version = 2.0;
  }
  else
  {
    return false;
  } 

  *get_model_number = model_number;
  if (setPacketHandler(protocol_version) == false)
    return false;

  return true;
}

bool DynamixelDriver::reboot(uint8_t id)
{
  if (packetHandler_->getProtocolVersion() == 1.0)
  {
    return false;
  }
  else
  {
    uint8_t error = 0;
    uint16_t comm_result = COMM_RX_FAIL;

    comm_result = packetHandler_->reboot(portHandler_, id, &error);
    millis(2000);

    if (comm_result == COMM_SUCCESS)
    {
      if (error != 0)
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }

  return true;
}

bool DynamixelDriver::reset(uint8_t id)
{
  uint8_t error = 0;
  uint16_t comm_result = COMM_RX_FAIL;
  bool isOK = false;

  uint32_t baud = 0;
  uint8_t new_id = 1;

  if (packetHandler_->getProtocolVersion() == 1.0)
  {
    // Reset Dynamixel except ID and Baudrate
    comm_result = packetHandler_->factoryReset(portHandler_, id, 0x00, &error);
    millis(2000);

    if (comm_result == COMM_SUCCESS)
    {
      if (error != 0)
      {
        return false;
      }

      uint8_t factor = getToolsFactor(id);

      for (int i = 0; i < tools_[factor].dxl_info_cnt_; i++)
      {
        if (tools_[factor].dxl_info_[i].id == id)
          tools_[factor].dxl_info_[i].id = new_id;
      }

      char* model_name = getModelName(new_id);
      if (!strncmp(model_name, "AX", strlen("AX")) ||
          !strncmp(model_name, "MX-12W", strlen("MX-12W")))
        baud = 1000000;
      else
        baud = 57600;

      if (portHandler_->setBaudRate(baud) == false)
      {
        millis(2000);
        return false;
      }
      else
      {
        millis(2000);

        if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
            !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
            !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
            !strncmp(model_name, "XL", strlen("XL")) ||
            !strncmp(model_name, "XM", strlen("XM")) ||
            !strncmp(model_name, "XH", strlen("XH")) ||
            !strncmp(model_name, "PRO", strlen("PRO")))
          isOK = setPacketHandler(2.0);
        else
          isOK = setPacketHandler(1.0);

        if (isOK)
          return true;
        else
          return false;
      }
    }
    else
    {
      return false;
    }
  }
  else if (packetHandler_->getProtocolVersion() == 2.0)
  {
    comm_result = packetHandler_->factoryReset(portHandler_, id, 0xff, &error);
    millis(2000);

    if (comm_result == COMM_SUCCESS)
    {
      if (error != 0)
      {   
        return false;
      }

      uint8_t factor = getToolsFactor(id);

      for (int i = 0; i < tools_[factor].dxl_info_cnt_; i++)
      {
        if (tools_[factor].dxl_info_[i].id == id)
          tools_[factor].dxl_info_[i].id = new_id;
      }

      if (!strncmp(getModelName(new_id), "XL-320", strlen("XL-320")))
      {
        baud = 1000000;
      }
      else
      {
        baud = 57600;
      }

      if (portHandler_->setBaudRate(baud) == false)
      {
        millis(2000);
        return false;
      }
      else
      {
        millis(2000);

        isOK = setPacketHandler(2.0);
        if (isOK)
          return true;
        else
          return false;
      }
    }
    else
    {
      return false;
    }
  }
}

bool DynamixelDriver::writeRegister(uint8_t id, const char *item_name, int32_t data)
{
  uint8_t error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  ControlTableItem *cti;
  cti = tools_[getToolsFactor(id)].getControlItem(item_name);

  if (cti->data_length == BYTE)
  {
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, cti->address, (uint8_t)data, &error);
  }
  else if (cti->data_length == WORD)
  {
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, cti->address, (uint16_t)data, &error);
  }
  else if (cti->data_length == DWORD)
  {
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, cti->address, (uint32_t)data, &error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (error != 0)
    {
      return false;
    }
  }
  else
  {
    return false;
  }

  return true;
}

bool DynamixelDriver::writeRegister(uint8_t id, uint16_t addr, uint8_t length, int32_t data)
{
  uint8_t error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (length == BYTE)
  {
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, addr, (uint8_t)data, &error);
  }
  else if (length == WORD)
  {
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, addr, (uint16_t)data, &error);
  }
  else if (length == DWORD)
  {
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, addr, (uint32_t)data, &error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (error != 0)
    {
      return false;
    }
  }
  else
  {
    return false;
  }

  return true;
}

bool DynamixelDriver::readRegister(uint8_t id, const char *item_name, int32_t *data)
{
  uint8_t error = 0;
  int dxl_comm_result = COMM_RX_FAIL;

  int8_t value_8_bit = 0;
  int16_t value_16_bit = 0;
  int32_t value_32_bit = 0;

  ControlTableItem *cti;
  cti = tools_[getToolsFactor(id)].getControlItem(item_name);

  if (cti->data_length == BYTE)
  {
    dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, cti->address, (uint8_t *)&value_8_bit, &error);
  }
  else if (cti->data_length == WORD)
  {
    dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, cti->address, (uint16_t *)&value_16_bit, &error);
  }
  else if (cti->data_length == DWORD)
  {
    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, cti->address, (uint32_t *)&value_32_bit, &error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (error != 0)
    {
      return false;
    }

    if (cti->data_length == BYTE)
    {
      *data = value_8_bit;
    }
    else if (cti->data_length == WORD)
    {
      *data = value_16_bit;
    }
    else if (cti->data_length == DWORD)
    {
      *data = value_32_bit;
    }

    return true;
  }
  else
  {
    return false;
  }
}

bool DynamixelDriver::readRegister(uint8_t id, uint16_t addr, uint8_t length, int32_t *data)
{
  uint8_t error = 0;
  int dxl_comm_result = COMM_RX_FAIL;

  int8_t value_8_bit = 0;
  int16_t value_16_bit = 0;
  int32_t value_32_bit = 0;

  if (length == BYTE)
  {
    dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, addr, (uint8_t *)&value_8_bit, &error);
  }
  else if (length == WORD)
  {
    dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, addr, (uint16_t *)&value_16_bit, &error);
  }
  else if (length == DWORD)
  {
    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, addr, (uint32_t *)&value_32_bit, &error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (error != 0)
    {
      return false;
    }

    if (length == BYTE)
    {
      *data = value_8_bit;
    }
    else if (length == WORD)
    {
      *data = value_16_bit;
    }
    else if (length == DWORD)
    {
      *data = value_32_bit;
    }

    return true;
  }
  else
  {
    return false;
  }
}

bool DynamixelDriver::readRegister(uint8_t id, uint16_t length, uint8_t *data)
{
  uint8_t error = 0;
  int dxl_comm_result = COMM_RX_FAIL;

  dxl_comm_result = packetHandler_->readTxRx(portHandler_, id, 0, length, data, &error);

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (error != 0)
    {
      return false;
    }

    return true;
  }
  else
  {
    return false;
  }
}

uint8_t DynamixelDriver::getToolsFactor(uint8_t id)
{
  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].dxl_info_cnt_; j++)
    {
      if (tools_[i].dxl_info_[j].id == id)
      {
        return i;
      }
    }
  }
}

const char *DynamixelDriver::findModelName(uint16_t model_num)
{
  uint16_t num = model_num;
  static const char* model_name = NULL;

  if (num == AX_12A)
    model_name = "AX-12A";
  else if (num == AX_12W)
    model_name = "AX-12W";
  else if (num == AX_18A)
    model_name = "AX-18A";

  else if (num == RX_24F)
    model_name = "RX-24F";
  else if (num == RX_28)
    model_name = "RX-28";
  else if (num == RX_64)
    model_name = "RX-64";

  else if (num == EX_106)
    model_name = "EX-106";

  else if (num == MX_12W)
    model_name = "MX-12W";
  else if (num == MX_28)
    model_name = "MX-28";
  else if (num == MX_28_2)
    model_name = "MX-28-2";
  else if (num == MX_64)
    model_name = "MX-64";
  else if (num == MX_64_2)
    model_name = "MX-64-2";
  else if (num == MX_106)
    model_name = "MX-106";
  else if (num == MX_106_2)
    model_name = "MX-106-2";

  else if (num == XL_320)
    model_name = "XL-320";
  else if (num == XL430_W250)
    model_name = "XL430-W250";

  else if (num == XM430_W210)
    model_name = "XM430-W210";
  else if (num == XM430_W350)
    model_name = "XM430-W350";
  else if (num == XM540_W150)
    model_name = "XM540-W150";
  else if (num == XM540_W270)
    model_name = "XM540-W270";

  else if (num == XH430_V210)
    model_name = "XH430-V210";
  else if (num == XH430_V350)
    model_name = "XH430-V350";
  else if (num == XH430_W210)
    model_name = "XH430-W210";
  else if (num == XH430_W350)
    model_name = "XH430-W350";

  else if (num == PRO_L42_10_S300_R)
    model_name = "PRO-L42-10-S300-R";
  else if (num == PRO_L54_30_S400_R)
    model_name = "PRO-L54-30-S400-R";
  else if (num == PRO_L54_30_S500_R)
    model_name = "PRO-L54-30-S500-R";
  else if (num == PRO_L54_50_S290_R)
    model_name = "PRO-L54-50-S290-R";
  else if (num == PRO_L54_50_S500_R)
    model_name = "PRO-L54-50-S500-R";

  else if (num == PRO_M42_10_S260_R)
    model_name = "PRO-M42-10-S260-R";
  else if (num == PRO_M54_40_S250_R)
    model_name = "PRO-M54-40-S250-R";
  else if (num == PRO_M54_60_S250_R)
    model_name = "PRO-M54-60-S250-R";

  else if (num == PRO_H42_20_S300_R)
    model_name = "PRO-H42-20-S300-R";
  else if (num == PRO_H54_100_S500_R)
    model_name = "PRO-H54-100-S500-R";
  else if (num == PRO_H54_200_S500_R)
    model_name = "PRO-H54-200-S500-R";

  return model_name;
}

void DynamixelDriver::addSyncWrite(const char *item_name)
{
  ControlTableItem *cti;
  cti = tools_[0].getControlItem(item_name);

  syncWriteHandler_[sync_write_handler_cnt_].cti = cti;

  syncWriteHandler_[sync_write_handler_cnt_++].groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler_,
                                                                                              packetHandler_,
                                                                                              cti->address,
                                                                                              cti->data_length);
}

bool DynamixelDriver::syncWrite(const char *item_name, int32_t *data)
{
  bool dxl_addparam_result = false;
  int dxl_comm_result = COMM_TX_FAIL;

  uint8_t data_byte[4] = {0, };
  uint8_t cnt = 0;

  SyncWriteHandler swh;

  for (int index = 0; index < sync_write_handler_cnt_; index++)
  {
    if (!strncmp(syncWriteHandler_[index].cti->item_name, item_name, strlen(item_name)))
    {
      swh = syncWriteHandler_[index];
    }
  }

  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].dxl_info_cnt_; j++)
    {
      data_byte[0] = DXL_LOBYTE(DXL_LOWORD(data[cnt]));
      data_byte[1] = DXL_HIBYTE(DXL_LOWORD(data[cnt]));
      data_byte[2] = DXL_LOBYTE(DXL_HIWORD(data[cnt]));
      data_byte[3] = DXL_HIBYTE(DXL_HIWORD(data[cnt]));

      dxl_addparam_result = swh.groupSyncWrite->addParam(tools_[i].dxl_info_[j].id, (uint8_t *)&data_byte);
      if (dxl_addparam_result != true)
      {
        return false;
      }

      cnt++;
    }
  }

  dxl_comm_result = swh.groupSyncWrite->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    return false;
  }
  swh.groupSyncWrite->clearParam();
  return true;
}

bool DynamixelDriver::syncWrite(uint8_t *id, uint8_t id_num, const char *item_name, int32_t *data)
{
  bool dxl_addparam_result = false;
  int dxl_comm_result = COMM_TX_FAIL;

  uint8_t data_byte[4] = {0, };
  uint8_t cnt = 0;

  SyncWriteHandler swh;

  for (int index = 0; index < sync_write_handler_cnt_; index++)
  {
    if (!strncmp(syncWriteHandler_[index].cti->item_name, item_name, strlen(item_name)))
    {
      swh = syncWriteHandler_[index];
    }
  }

  for (int i = 0; i < id_num; i++)
  {
    data_byte[0] = DXL_LOBYTE(DXL_LOWORD(data[cnt]));
    data_byte[1] = DXL_HIBYTE(DXL_LOWORD(data[cnt]));
    data_byte[2] = DXL_LOBYTE(DXL_HIWORD(data[cnt]));
    data_byte[3] = DXL_HIBYTE(DXL_HIWORD(data[cnt]));

    dxl_addparam_result = swh.groupSyncWrite->addParam(id[i], (uint8_t *)&data_byte);
    if (dxl_addparam_result != true)
    {
      return false;
    }
  }

  dxl_comm_result = swh.groupSyncWrite->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    return false;
  }
  swh.groupSyncWrite->clearParam();
  return true;
}

void DynamixelDriver::addSyncRead(const char *item_name)
{
  ControlTableItem *cti;
  cti = tools_[0].getControlItem(item_name);

  syncReadHandler_[sync_read_handler_cnt_].cti = cti;
  
  syncReadHandler_[sync_read_handler_cnt_++].groupSyncRead = new dynamixel::GroupSyncRead(portHandler_,
                                                                                          packetHandler_,
                                                                                          cti->address,
                                                                                          cti->data_length);
}

bool DynamixelDriver::syncRead(const char *item_name, int32_t *data)
{
  int dxl_comm_result = COMM_RX_FAIL;
  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;

  int index = 0;

  SyncReadHandler srh;
  
  for (int index = 0; index < sync_read_handler_cnt_; index++)
  {
    if (!strncmp(syncReadHandler_[index].cti->item_name, item_name, strlen(item_name)))
    {
      srh = syncReadHandler_[index];
    }
  }

  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].dxl_info_cnt_; j++)
    {
      dxl_addparam_result = srh.groupSyncRead->addParam(tools_[i].dxl_info_[j].id);
      if (dxl_addparam_result != true)
        return false;
    }
  }

  dxl_comm_result = srh.groupSyncRead->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    return false;
  }

  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].dxl_info_cnt_; j++)
    {
      uint8_t id = tools_[i].dxl_info_[j].id;

      dxl_getdata_result = srh.groupSyncRead->isAvailable(id, srh.cti->address, srh.cti->data_length);
      if (dxl_getdata_result)
      {
        data[index++] = srh.groupSyncRead->getData(id, srh.cti->address, srh.cti->data_length);
      }
      else
      {
        return false;
      }
    }
  }

  srh.groupSyncRead->clearParam();

  return true;
}

void DynamixelDriver::initBulkWrite()
{
  groupBulkWrite_ = new dynamixel::GroupBulkWrite(portHandler_, packetHandler_);
}

bool DynamixelDriver::addBulkWriteParam(uint8_t id, const char *item_name, int32_t data)
{
  bool dxl_addparam_result = false;
  uint8_t data_byte[4] = {0, };

  ControlTableItem *cti;
  cti = tools_[getToolsFactor(id)].getControlItem(item_name);

  data_byte[0] = DXL_LOBYTE(DXL_LOWORD(data));
  data_byte[1] = DXL_HIBYTE(DXL_LOWORD(data));
  data_byte[2] = DXL_LOBYTE(DXL_HIWORD(data));
  data_byte[3] = DXL_HIBYTE(DXL_HIWORD(data));

  dxl_addparam_result = groupBulkWrite_->addParam(id, cti->address, cti->data_length, data_byte);
  if (dxl_addparam_result != true)
  {
    return false;
  }

  return true;
}

bool DynamixelDriver::bulkWrite()
{
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = groupBulkWrite_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    return false;
  }

  groupBulkWrite_->clearParam();

  return true;
}

void DynamixelDriver::initBulkRead()
{
  groupBulkRead_ = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);
}

bool DynamixelDriver::addBulkReadParam(uint8_t id, const char *item_name)
{
  bool dxl_addparam_result = false;

  ControlTableItem *cti;
  cti = tools_[getToolsFactor(id)].getControlItem(item_name);

  dxl_addparam_result = groupBulkRead_->addParam(id, cti->address, cti->data_length);
  if (dxl_addparam_result != true)
  {
    return false;
  }

  return true;
}

bool DynamixelDriver::sendBulkReadPacket()
{
  int dxl_comm_result = COMM_RX_FAIL;

  dxl_comm_result = groupBulkRead_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    return false;
  }

  return true;
}

bool DynamixelDriver::bulkRead(uint8_t id, const char *item_name, int32_t *data)
{
  bool dxl_getdata_result = false;
  ControlTableItem *cti;
  cti = tools_[getToolsFactor(id)].getControlItem(item_name);

  dxl_getdata_result = groupBulkRead_->isAvailable(id, cti->address, cti->data_length);
  if (dxl_getdata_result != true)
  {
    return false;
  }

  *data = groupBulkRead_->getData(id, cti->address, cti->data_length);

  return true;
}

int32_t DynamixelDriver::convertRadian2Value(uint8_t id, float radian)
{
  int32_t value = 0;
  int8_t factor = getToolsFactor(id);

  if (radian > 0)
  {
    value = (radian * (tools_[factor].getValueOfMaxRadianPosition() - tools_[factor].getValueOfZeroRadianPosition()) / tools_[factor].getMaxRadian()) + tools_[factor].getValueOfZeroRadianPosition();
  }
  else if (radian < 0)
  {
    value = (radian * (tools_[factor].getValueOfMinRadianPosition() - tools_[factor].getValueOfZeroRadianPosition()) / tools_[factor].getMinRadian()) + tools_[factor].getValueOfZeroRadianPosition();
  }
  else
  {
    value = tools_[factor].getValueOfZeroRadianPosition();
  }

  return value;
}

float DynamixelDriver::convertValue2Radian(uint8_t id, int32_t value)
{
  float radian = 0.0;
  int8_t factor = getToolsFactor(id);

  if (value > tools_[factor].getValueOfZeroRadianPosition())
  {
    radian = (float)(value - tools_[factor].getValueOfZeroRadianPosition()) * tools_[factor].getMaxRadian() / (float)(tools_[factor].getValueOfMaxRadianPosition() - tools_[factor].getValueOfZeroRadianPosition());
  }
  else if (value < tools_[factor].getValueOfZeroRadianPosition())
  {
    radian = (float)(value - tools_[factor].getValueOfZeroRadianPosition()) * tools_[factor].getMinRadian() / (float)(tools_[factor].getValueOfMinRadianPosition() - tools_[factor].getValueOfZeroRadianPosition());
  }

  return radian;
}

int32_t DynamixelDriver::convertRadian2Value(float radian, int32_t max_position, int32_t min_position, float max_radian, float min_radian)
{
  int32_t value = 0;
  int32_t zero_position = (max_position + min_position)/2;

  if (radian > 0)
  {
    value = (radian * (max_position - zero_position) / max_radian) + zero_position;
  }
  else if (radian < 0)
  {
    value = (radian * (min_position - zero_position) / min_radian) + zero_position;
  }
  else
  {
    value = zero_position;
  }

  return value;
}

float DynamixelDriver::convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position, float max_radian, float min_radian)
{
  float radian = 0.0;
  int32_t zero_position = (max_position + min_position)/2;

  if (value > zero_position)
  {
    radian = (float)(value - zero_position) * max_radian / (float)(max_position - zero_position);
  }
  else if (value < zero_position)
  {
    radian = (float)(value - zero_position) * min_radian / (float)(min_position - zero_position);
  }

  return radian;
}

int32_t DynamixelDriver::convertVelocity2Value(uint8_t id, float velocity)
{
  int32_t value = 0;
  int8_t factor = getToolsFactor(id);

  value = velocity * tools_[factor].getVelocityToValueRatio();

  return value;
}

float DynamixelDriver::convertValue2Velocity(uint8_t id, int32_t value)
{
  float velocity = 0;
  int8_t factor = getToolsFactor(id);

  velocity = value / tools_[factor].getVelocityToValueRatio();

  return velocity;
}

int16_t DynamixelDriver::convertTorque2Value(uint8_t id, float torque)
{
  int16_t value = 0;
  int8_t factor = getToolsFactor(id);

  value = torque * tools_[factor].getTorqueToCurrentValueRatio();

  return value;
}

float DynamixelDriver::convertValue2Torque(uint8_t id, int16_t value)
{
  float torque = 0.0;
  int8_t factor = getToolsFactor(id);

  torque = value / tools_[factor].getTorqueToCurrentValueRatio();

  return torque;
}

void DynamixelDriver::millis(uint16_t msec)
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(msec);
#else
    usleep(1000*msec);
#endif
}
