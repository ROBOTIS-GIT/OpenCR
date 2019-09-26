/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

DynamixelDriver::DynamixelDriver() : tools_cnt_(0), 
                                    sync_write_handler_cnt_(0), 
                                    sync_read_handler_cnt_(0),
                                    bulk_read_parameter_cnt_(0)
{

}

DynamixelDriver::~DynamixelDriver()
{ 
  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
    {
      writeRegister(tools_[i].getID()[j], "Torque_Enable", (uint8_t)0);
    }
  }

  portHandler_->closePort();
}

void DynamixelDriver::initTools(void)
{
  for (uint8_t num = 0; num < MAX_DXL_SERIES_NUM; num++)
    tools_[num].initTool();

  tools_cnt_ = 0;
}

bool DynamixelDriver::setTool(uint16_t model_number, uint8_t id, const char **log)
{
  bool result = false;

  // See if we have a matching tool? 
  for (uint8_t num = 0; num < tools_cnt_; num++)
  {
    if (tools_[num].getModelNumber() == model_number)
    {
      if (tools_[num].getDynamixelCount() < tools_[num].getDynamixelBuffer())
      {
        // Found one with the right model number and it is not full
        tools_[num].addDXL(id);
        return true;
      }
      else
      {
        if (log != NULL) *log = "[DynamixelDriver] Too many Dynamixels are connected (default buffer size is 16, the same series of Dynamixels)";
        return false;
      }
    }
  }
  // We did not find one so lets allocate a new one
  if (tools_cnt_ < MAX_DXL_SERIES_NUM) 
  {
    // only do it if we still have some room...
    result = tools_[tools_cnt_++].addTool(model_number, id, log);
    return result;
  }
  else
  {
    if (log != NULL) *log = "[DynamixelDriver] Too many series are connected (MAX = 5 different series)";
    return false;
  }

  if (log != NULL) *log = "[DynamixelDriver] Failed to set the Tool";
  return false;
}

uint8_t DynamixelDriver::getTool(uint8_t id, const char **log)
{
  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
    {
      if (tools_[i].getID()[j] == id)
      {
        return i;
      }
    }
  }

  if (log != NULL) *log = "[DynamixelDriver] Failed to get the Tool";
  return 0xff;
}

bool DynamixelDriver::init(const char *device_name, uint32_t baud_rate, const char **log)
{
  bool result = false;

  result = setPortHandler(device_name, log);
  if (result == false) return false;

  result = setBaudrate(baud_rate, log);
  if (result == false) return false;

  result = setPacketHandler(2.0f, log);
  if (result == false) return false;

  return result;
}

bool DynamixelDriver::begin(const char *device_name, uint32_t baud_rate, const char **log)
{
  return init(device_name, baud_rate, log);
}

bool DynamixelDriver::setPortHandler(const char *device_name, const char **log)
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name);

  if (portHandler_->openPort())
  {
    if (log != NULL) *log = "[DynamixelDriver] Succeeded to open the port!";
    return true;
  }

  if (log != NULL) *log = "[DynamixelDriver] Failed to open the port!";
  return false;
}

bool DynamixelDriver::setBaudrate(uint32_t baud_rate, const char **log)
{
  if (portHandler_->setBaudRate((int)baud_rate))
  {
    if (log != NULL) *log = "[DynamixelDriver] Succeeded to change the baudrate!";
    return true;
  }

  if (log != NULL) *log = "[DynamixelDriver] Failed to change the baudrate!";
  return false;
}

bool DynamixelDriver::setPacketHandler(float protocol_version, const char **log)
{
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  if (packetHandler_->getProtocolVersion() == protocol_version)
  {
    if (log != NULL) *log = "[DynamixelDriver] Succeeded to set the protocol!";
    return true;
  }

  if (log != NULL) *log = "[DynamixelDriver] Failed to set the protocol!";
  return false;
}

float DynamixelDriver::getProtocolVersion(void)
{
  return packetHandler_->getProtocolVersion();
}

uint32_t DynamixelDriver::getBaudrate(void)
{
  return portHandler_->getBaudRate();
}

const char* DynamixelDriver::getModelName(uint8_t id, const char **log)
{
  uint8_t factor = getTool(id, log);
  if (factor == 0xff) 
    return NULL;
  else
    return tools_[factor].getModelName();

  return NULL;
}

uint16_t DynamixelDriver::getModelNumber(uint8_t id, const char **log)
{
  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false;

  for (int i = 0; i < tools_[factor].getDynamixelCount(); i++)
  {
    if (tools_[factor].getID()[i] == id)
      return tools_[factor].getModelNumber();
  }

  return false;
}

const ControlItem* DynamixelDriver::getControlTable(uint8_t id, const char **log)
{
  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return NULL;

  return tools_[factor].getControlTable();
}

const ControlItem* DynamixelDriver::getItemInfo(uint8_t id, const char *item_name, const char **log)
{
  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return NULL; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return NULL;
  else return control_item;

  return NULL;
}

uint8_t DynamixelDriver::getTheNumberOfControlItem(uint8_t id, const char **log)
{
  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false;

  return tools_[factor].getTheNumberOfControlItem();
}

const ModelInfo* DynamixelDriver::getModelInfo(uint8_t id, const char **log)
{
  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return NULL;

  return tools_[factor].getModelInfo();
}

uint8_t DynamixelDriver::getTheNumberOfSyncWriteHandler(void)
{
  return sync_write_handler_cnt_;
}

uint8_t DynamixelDriver::getTheNumberOfSyncReadHandler(void)
{
  return sync_read_handler_cnt_;
}

uint8_t DynamixelDriver::getTheNumberOfBulkReadParam(void)
{
  return bulk_read_parameter_cnt_;
}

bool DynamixelDriver::scan(uint8_t *get_id, uint8_t *get_the_number_of_id, uint8_t start_num, uint8_t end_num, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  bool result = false;

  uint8_t id = 0;
  uint8_t id_cnt = 0;

  uint16_t model_number = 0;

  uint8_t get_end_num = end_num;

  if (get_end_num > 253) get_end_num = 253;

  initTools();

  result = setPacketHandler(1.0f, log);
  if (result == false) return false;

  for (id = start_num; id <= get_end_num; id++)
  { 
    sdk_error.dxl_comm_result = packetHandler_->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    }
    else if (sdk_error.dxl_error != 0)
    {
      if (log != NULL) *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    }
    else
    {
      get_id[id_cnt++] = id;
      setTool(model_number, id);
    }    
  }

  if (id_cnt > 0)
  {
    *get_the_number_of_id = id_cnt;
    return result;
  }

  result = setPacketHandler(2.0f, log);
  if (result == false) return false;

  for (id = start_num; id <= get_end_num; id++)
  {
    sdk_error.dxl_comm_result = packetHandler_->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
    
    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    }
    else if (sdk_error.dxl_error != 0)
    {
      if (log != NULL) *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    }
    else
    {
      get_id[id_cnt++] = id;
      setTool(model_number, id);
    }   
  }

  if (id_cnt > 0)
  {
    *get_the_number_of_id = id_cnt;
    return result;
  }

  return result;
}

bool DynamixelDriver::scan(uint8_t *get_id, uint8_t *get_the_number_of_id, uint8_t range, const char **log)
{
  return scan(get_id, get_the_number_of_id, 0, range, log);
}

bool DynamixelDriver::ping(uint8_t id, uint16_t *get_model_number, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  bool result = false;

  uint16_t model_number = 0;

  result = setPacketHandler(1.0f, log);
  if (result == false) return false;

  sdk_error.dxl_comm_result = packetHandler_->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);  
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL)  *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
  }
  else if (sdk_error.dxl_error != 0)
  {
    if (log != NULL)  *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
  }
  else
  {
    setTool(model_number, id);
    if (get_model_number != NULL) *get_model_number = model_number;
    return true;
  }

  result = setPacketHandler(2.0f, log);
  if (result == false) return false;

  sdk_error.dxl_comm_result = packetHandler_->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);  
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL)  *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
  }
  else if (sdk_error.dxl_error != 0)
  {
    if (log != NULL)  *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
  }
  else
  {
    setTool(model_number, id);
    if (get_model_number != NULL) *get_model_number = model_number;
    return true;
  }  

  return false;
}

bool DynamixelDriver::ping(uint8_t id, const char **log)
{
  return ping(id, NULL, log);
}

bool DynamixelDriver::clearMultiTurn(uint8_t id, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  sdk_error.dxl_comm_result = packetHandler_->clearMultiTurn(portHandler_, id, &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    if (log != NULL) *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    if (log != NULL) *log = "[DynamixelDriver] Succeeded to clear!";
    return true;
  }

  return false;
}

bool DynamixelDriver::reboot(uint8_t id, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  if (getProtocolVersion() == 1.0)
  {
    if (log != NULL) *log = "[DynamixelDriver] reboot functions is not available with the Dynamixel Protocol 1.0.";
    return false;
  }
  else
  {
    sdk_error.dxl_comm_result = packetHandler_->reboot(portHandler_, id, &sdk_error.dxl_error);
#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(1000);
#else
    usleep(1000*1000);
#endif

    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      if (log != NULL) *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      if (log != NULL) *log = "[DynamixelDriver] Succeeded to reboot!";
      return true;
    }
  }

  return false;
}

bool DynamixelDriver::reset(uint8_t id, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  bool result = false;

  uint32_t new_baud_rate = 0;
  uint8_t new_id = 1;

  const char* model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  uint16_t model_number = getModelNumber(id, log);
  if (model_number == 0) return false;

  if (getProtocolVersion() == 1.0)
  {
    sdk_error.dxl_comm_result = packetHandler_->factoryReset(portHandler_, id, 0x00, &sdk_error.dxl_error);
#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(2000);
#else
    usleep(1000*2000);
#endif

    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      if (log != NULL) *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      if (!strncmp(model_name, "AX", strlen("AX")) ||
          !strncmp(model_name, "MX-12W", strlen("MX-12W")))
        new_baud_rate = 1000000;
      else
        new_baud_rate = 57600;

      result = setBaudrate(new_baud_rate, log);
      if (result == false) 
        return false;
      else
      {
        if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
            !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
            !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
            !strncmp(model_name, "XL", strlen("XL"))  ||
            !strncmp(model_name, "XM", strlen("XM"))  ||
            !strncmp(model_name, "XH", strlen("XH"))  ||
            !strncmp(model_name, "PRO", strlen("PRO"))||
            !strncmp(model_name, "RH", strlen("RH")))
        {
          result = setPacketHandler(2.0f, log);
          if (result == false) return false;
        }          
        else
        {
          result = setPacketHandler(1.0f, log);
          if (result == false) return false; 
        }
      }
    }

    initTools();
    result = setTool(model_number, new_id, log);
    if (result == false) return false; 
    
    if (log != NULL) *log = "[DynamixelDriver] Succeeded to reset!";
    return true;
  }
  else if (getProtocolVersion() == 2.0)
  {
    sdk_error.dxl_comm_result = packetHandler_->factoryReset(portHandler_, id, 0xff, &sdk_error.dxl_error);
#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(2000);
#else
    usleep(1000*2000);
#endif

    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      if (log != NULL) *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      if (!strncmp(model_name, "XL-320", strlen("XL-320"))) 
        new_baud_rate = 1000000;
      else 
        new_baud_rate = 57600;

      result = setBaudrate(new_baud_rate, log);
      if (result == false) 
        return false;
      else
      {
        result = setPacketHandler(2.0f, log);
        if (result == false)  return false;
      }
    }

    initTools();
    result = setTool(model_number, new_id, log);
    if (result == false) return false; 
    
    if (log != NULL) *log = "[DynamixelDriver] Succeeded to reset!";
    return true;
  }

  return result;
}

bool DynamixelDriver::writeRegister(uint8_t id, uint16_t address, uint16_t length, uint8_t* data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(10);
#else
    usleep(1000*10);
#endif

  sdk_error.dxl_comm_result = packetHandler_->writeTxRx(portHandler_, 
                                                        id, 
                                                        address, 
                                                        length, 
                                                        data,
                                                        &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    if (log != NULL) *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    if (log != NULL) *log = "[DynamixelDriver] Succeeded to write!";
    return true;
  }

  return false;
}

bool DynamixelDriver::writeRegister(uint8_t id, const char *item_name, int32_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false;

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  uint8_t data_1_byte = (uint8_t)data;
  uint16_t data_2_byte = (uint16_t)data;
  uint32_t data_4_byte = (uint32_t)data;

#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(10);
#else
    usleep(1000*10);
#endif

  switch (control_item->data_length)
  {
    case BYTE:
      sdk_error.dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_,
                                                            id,
                                                            control_item->address,
                                                            data_1_byte,
                                                            &sdk_error.dxl_error);
     break;

    case WORD:
      sdk_error.dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_,
                                                            id,
                                                            control_item->address,
                                                            data_2_byte,
                                                            &sdk_error.dxl_error);
     break;

    case DWORD:
      sdk_error.dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_,
                                                            id,
                                                            control_item->address,
                                                            data_4_byte,
                                                            &sdk_error.dxl_error);
     break;

    default:
      sdk_error.dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_,
                                                            id,
                                                            control_item->address,
                                                            data_1_byte,
                                                            &sdk_error.dxl_error);
     break;
  }

  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    if (log != NULL) *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    if (log != NULL) *log = "[DynamixelDriver] Succeeded to write!";
    return true;
  }

  return false;
}

bool DynamixelDriver::writeOnlyRegister(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(10);
#else
    usleep(1000*10);
#endif

  sdk_error.dxl_comm_result = packetHandler_->writeTxOnly(portHandler_, 
                                                          id, 
                                                          address, 
                                                          length, 
                                                          data);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else
  {
    if (log != NULL) *log = "[DynamixelDriver] Succeeded to write!";
    return true;
  }

  return false;
}

bool DynamixelDriver::writeOnlyRegister(uint8_t id, const char *item_name, int32_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false;

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(10);
#else
    usleep(1000*10);
#endif

  switch (control_item->data_length)
  {
    case BYTE:
      sdk_error.dxl_comm_result = packetHandler_->write1ByteTxOnly(portHandler_,
                                                            id,
                                                            control_item->address,
                                                            (uint8_t)data);
     break;

    case WORD:
      sdk_error.dxl_comm_result = packetHandler_->write2ByteTxOnly(portHandler_,
                                                            id,
                                                            control_item->address,
                                                            (uint16_t)data);
     break;

    case DWORD:
      sdk_error.dxl_comm_result = packetHandler_->write4ByteTxOnly(portHandler_,
                                                            id,
                                                            control_item->address,
                                                            (uint32_t)data);
     break;

    default:
      sdk_error.dxl_comm_result = packetHandler_->write1ByteTxOnly(portHandler_,
                                                            id,
                                                            control_item->address,
                                                            (uint8_t)data);
     break;
  }

  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else
  {
    if (log != NULL) *log = "[DynamixelDriver] Succeeded to write!";
    return true;
  }

  return false;
}

bool DynamixelDriver::readRegister(uint8_t id, uint16_t address, uint16_t length, uint32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  
  uint8_t data_read[length];

  sdk_error.dxl_comm_result = packetHandler_->readTxRx(portHandler_, 
                                                      id, 
                                                      address,
                                                      length, 
                                                      (uint8_t *)&data_read, 
                                                      &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    if (log != NULL) *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    switch (length)
    {
      case BYTE:
        *data = data_read[0];
       break;

      case WORD:
        *data = DXL_MAKEWORD(data_read[0], data_read[1]);
       break;

      case DWORD:
        *data = DXL_MAKEDWORD(DXL_MAKEWORD(data_read[0], data_read[1]), DXL_MAKEWORD(data_read[2], data_read[3]));
       break;

      default:
        for (uint16_t index = 0; index < length; index++)
        {
          data[index] = (uint32_t)data_read[index];
        }
       break;
    }
    if (log != NULL) *log = "[DynamixelDriver] Succeeded to read!";
    return true;
  }

  return false;
}

bool DynamixelDriver::readRegister(uint8_t id, const char *item_name, int32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false;

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  uint8_t data_1_byte  = 0;
  uint16_t data_2_byte = 0;
  uint32_t data_4_byte = 0;

  switch (control_item->data_length)
  {
    case BYTE:
      sdk_error.dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_,
                                                            id,
                                                            control_item->address,
                                                            &data_1_byte,
                                                            &sdk_error.dxl_error);
     break;

    case WORD:
      sdk_error.dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_,
                                                            id,
                                                            control_item->address,
                                                            &data_2_byte,
                                                            &sdk_error.dxl_error);
     break;

    case DWORD:
      sdk_error.dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_,
                                                            id,
                                                            control_item->address,
                                                            &data_4_byte,
                                                            &sdk_error.dxl_error);
     break;

    default:
      sdk_error.dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_,
                                                            id,
                                                            control_item->address,
                                                            &data_1_byte,
                                                            &sdk_error.dxl_error);
     break;
  }

  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    if (log != NULL) *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    switch (control_item->data_length)
    {
      case BYTE:
        *data = data_1_byte;
      break;

      case WORD:
        *data = data_2_byte;
      break;

      case DWORD:
        *data = data_4_byte;
      break;

      default:
        *data = data_1_byte;
      break;
    }

    if (log != NULL) *log = "[DynamixelDriver] Succeeded to read!";
    return true;
  }

  return false;
}

void DynamixelDriver::getParam(int32_t data, uint8_t *param)
{
  param[0] = DXL_LOBYTE(DXL_LOWORD(data));
  param[1] = DXL_HIBYTE(DXL_LOWORD(data));
  param[2] = DXL_LOBYTE(DXL_HIWORD(data));
  param[3] = DXL_HIBYTE(DXL_HIWORD(data));
}

bool DynamixelDriver::addSyncWriteHandler(uint16_t address, uint16_t length, const char **log)
{
  if (sync_write_handler_cnt_ > (MAX_HANDLER_NUM-1))
  {
    if (log != NULL) *log = "[DynamixelDriver] Too many sync write handler are added (MAX = 5)";
    return false;
  }

  syncWriteHandler_[sync_write_handler_cnt_].control_item = NULL;

  syncWriteHandler_[sync_write_handler_cnt_].groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler_,
                                                                                            packetHandler_,
                                                                                            address,
                                                                                            length);

  sync_write_handler_cnt_++;

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to add sync write handler";
  return true;    
}

bool DynamixelDriver::addSyncWriteHandler(uint8_t id, const char *item_name, const char **log)
{
  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  if (sync_write_handler_cnt_ > (MAX_HANDLER_NUM-1))
  {
    if (log != NULL) *log = "[DynamixelDriver] Too many sync write handler are added (MAX = 5)";
    return false;
  }

  syncWriteHandler_[sync_write_handler_cnt_].control_item = control_item;

  syncWriteHandler_[sync_write_handler_cnt_].groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler_,
                                                                                            packetHandler_,
                                                                                            control_item->address,
                                                                                            control_item->data_length);

  sync_write_handler_cnt_++;

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to add sync write handler";
  return true;                                                            
}

bool DynamixelDriver::syncWrite(uint8_t index, int32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t dxl_cnt = 0;
  uint8_t parameter[4] = {0, 0, 0, 0};

  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
    {
      getParam(data[dxl_cnt], parameter);
      sdk_error.dxl_addparam_result = syncWriteHandler_[index].groupSyncWrite->addParam(tools_[i].getID()[j], (uint8_t *)&parameter);
      if (sdk_error.dxl_addparam_result != true)
      {
        if (log != NULL) *log = "groupSyncWrite addparam failed";
        return false;
      }
      else
        dxl_cnt++;
    }
  }

  sdk_error.dxl_comm_result = syncWriteHandler_[index].groupSyncWrite->txPacket();
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }

  syncWriteHandler_[index].groupSyncWrite->clearParam();

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to sync write!";
  return true;
}

bool DynamixelDriver::syncWrite(uint8_t index, uint8_t *id, uint8_t id_num, int32_t *data, uint8_t data_num_for_each_id, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t parameter[4] = {0, 0, 0, 0};
  uint8_t multi_parameter[4*data_num_for_each_id];
  uint8_t cnt = 0;

  for (int i = 0; i < id_num; i++)
  {
    for (int j = 0; j < data_num_for_each_id; j++)
    {
      getParam(data[cnt++], parameter);
      for (int k = 0; k < 4; k++)
      {
        multi_parameter[4*j+k] = parameter[k];
      }
    }

    sdk_error.dxl_addparam_result = syncWriteHandler_[index].groupSyncWrite->addParam(id[i], (uint8_t *)&multi_parameter);
    if (sdk_error.dxl_addparam_result != true)
    {
      if (log != NULL) *log = "groupSyncWrite addparam failed";
      return false;
    }
  }

  sdk_error.dxl_comm_result = syncWriteHandler_[index].groupSyncWrite->txPacket();
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }

  syncWriteHandler_[index].groupSyncWrite->clearParam();

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to sync write!";
  return true;
}

bool DynamixelDriver::addSyncReadHandler(uint16_t address, uint16_t length, const char **log)
{
  if (sync_read_handler_cnt_ > (MAX_HANDLER_NUM-1))
  {
    if (log != NULL) *log = "[DynamixelDriver] Too many sync read handler are added (MAX = 5)";
    return false;
  }

  syncReadHandler_[sync_read_handler_cnt_].control_item = NULL;

  syncReadHandler_[sync_read_handler_cnt_].groupSyncRead = new dynamixel::GroupSyncRead(portHandler_,
                                                                                          packetHandler_,
                                                                                          address,
                                                                                          length);

  sync_read_handler_cnt_++;

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to add sync read handler";
  return true;
}

bool DynamixelDriver::addSyncReadHandler(uint8_t id, const char *item_name, const char **log)
{
  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  if (sync_read_handler_cnt_ > (MAX_HANDLER_NUM-1))
  {
    if (log != NULL) *log = "[DynamixelDriver] Too many sync read handler are added (MAX = 5)";
    return false;
  }

  syncReadHandler_[sync_read_handler_cnt_].control_item = control_item;

  syncReadHandler_[sync_read_handler_cnt_].groupSyncRead = new dynamixel::GroupSyncRead(portHandler_,
                                                                                          packetHandler_,
                                                                                          control_item->address,
                                                                                          control_item->data_length);

  sync_read_handler_cnt_++;

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to add sync read handler";
  return true;       
}

bool DynamixelDriver::syncRead(uint8_t index, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  syncReadHandler_[index].groupSyncRead->clearParam();
  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
    {
      sdk_error.dxl_addparam_result = syncReadHandler_[index].groupSyncRead->addParam(tools_[i].getID()[j]);
      if (sdk_error.dxl_addparam_result != true)
      {
        if (log != NULL) *log = "groupSyncWrite addparam failed";
        return false;
      }
    }
  }

  sdk_error.dxl_comm_result = syncReadHandler_[index].groupSyncRead->txRxPacket();
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to sync read!";
  return true;
}

bool DynamixelDriver::syncRead(uint8_t index, uint8_t *id, uint8_t id_num, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  syncReadHandler_[index].groupSyncRead->clearParam();
  for (int i = 0; i < id_num; i++)
  {
    sdk_error.dxl_addparam_result = syncReadHandler_[index].groupSyncRead->addParam(id[i]);
    if (sdk_error.dxl_addparam_result != true)
    {
      if (log != NULL) *log = "groupSyncWrite addparam failed";
      return false;
    }
  }

  sdk_error.dxl_comm_result = syncReadHandler_[index].groupSyncRead->txRxPacket();
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to sync read!";
  return true;
}

bool DynamixelDriver::getSyncReadData(uint8_t index, int32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
    {
      sdk_error.dxl_getdata_result = syncReadHandler_[index].groupSyncRead->isAvailable(tools_[i].getID()[j], 
                                                                                        syncReadHandler_[index].control_item->address, 
                                                                                        syncReadHandler_[index].control_item->data_length);
      if (sdk_error.dxl_getdata_result != true)
      {
        if (log != NULL) *log = "groupSyncRead getdata failed";
        return false;
      }
      else
      {
        data[i+j] = syncReadHandler_[index].groupSyncRead->getData(tools_[i].getID()[j], 
                                                                    syncReadHandler_[index].control_item->address, 
                                                                    syncReadHandler_[index].control_item->data_length);
      }
    }
  }

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to get sync read data!";
  return true;
}

bool DynamixelDriver::getSyncReadData(uint8_t index, uint8_t *id, uint8_t id_num, int32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  for (int i = 0; i < id_num; i++)
  {
    sdk_error.dxl_getdata_result = syncReadHandler_[index].groupSyncRead->isAvailable(id[i], 
                                                                                      syncReadHandler_[index].control_item->address, 
                                                                                      syncReadHandler_[index].control_item->data_length);
    if (sdk_error.dxl_getdata_result != true)
    {
      if (log != NULL) *log = "groupSyncRead getdata failed";
      return false;
    }
    else
    {
      data[i] = syncReadHandler_[index].groupSyncRead->getData(id[i], 
                                                                syncReadHandler_[index].control_item->address, 
                                                                syncReadHandler_[index].control_item->data_length);
    }
  }

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to get sync read data!";
  return true;
}

bool DynamixelDriver::getSyncReadData(uint8_t index, uint8_t *id, uint8_t id_num, uint16_t address, uint16_t length, int32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  
  for (int i = 0; i < id_num; i++)
  {
    sdk_error.dxl_getdata_result = syncReadHandler_[index].groupSyncRead->isAvailable(id[i], 
                                                                                      address, 
                                                                                      length);
    if (sdk_error.dxl_getdata_result != true)
    {
      if (log != NULL) *log = "groupSyncRead getdata failed";
      return false;
    }
    else
    {
      data[i] = syncReadHandler_[index].groupSyncRead->getData(id[i], 
                                                              address, 
                                                              length);
    }
  }

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to get sync read data!";
  return true;
}

bool DynamixelDriver::initBulkWrite(const char **log)
{
  if (portHandler_ == NULL)
  {
    if (log != NULL) *log = "[DynamixelDriver] Failed to load portHandler!";
  }
  else if (packetHandler_ == NULL)
  {
    if (log != NULL) *log = "[DynamixelDriver] Failed to load packetHandler!";
  }
  else
  {
    groupBulkWrite_ = new dynamixel::GroupBulkWrite(portHandler_, packetHandler_);

    if (log != NULL) *log = "[DynamixelDriver] Succeeded to init groupBulkWrite!";
    return true;
  }

  return false;
}

bool DynamixelDriver::addBulkWriteParam(uint8_t id, uint16_t address, uint16_t length, int32_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t parameter[4] = {0, 0, 0, 0};

  getParam(data, parameter);
  sdk_error.dxl_addparam_result = groupBulkWrite_->addParam(id, 
                                                            address, 
                                                            length, 
                                                            parameter);
  if (sdk_error.dxl_addparam_result != true)
  {
    if (log != NULL) *log = "groupBulkWrite addparam failed";
    return false;
  }

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to add param for bulk write!";
  return true;
}

bool DynamixelDriver::addBulkWriteParam(uint8_t id, const char *item_name, int32_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t parameter[4] = {0, 0, 0, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  getParam(data, parameter);
  sdk_error.dxl_addparam_result = groupBulkWrite_->addParam(id, 
                                                            control_item->address, 
                                                            control_item->data_length, 
                                                            parameter);
  if (sdk_error.dxl_addparam_result != true)
  {
    if (log != NULL) *log = "groupBulkWrite addparam failed";
    return false;
  }

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to add param for bulk write!";
  return true;
}

bool DynamixelDriver::bulkWrite(const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  sdk_error.dxl_comm_result = groupBulkWrite_->txPacket();
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }

  groupBulkWrite_->clearParam();
  if (log != NULL) *log = "[DynamixelDriver] Succeeded to bulk write!";

  return true;
}

bool DynamixelDriver::initBulkRead(const char **log)
{
  if (portHandler_ == NULL)
  {
    if (log != NULL) *log = "[DynamixelDriver] Failed to load portHandler!";
  }
  else if (packetHandler_ == NULL)
  {
    if (log != NULL) *log = "[DynamixelDriver] Failed to load packetHandler!";
  }
  else
  {
    groupBulkRead_ = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);

    if (log != NULL) *log = "[DynamixelDriver] Succeeded to init groupBulkRead!";

    return true;
  }

  return false;
}

bool DynamixelDriver::addBulkReadParam(uint8_t id, uint16_t address, uint16_t length, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  sdk_error.dxl_addparam_result = groupBulkRead_->addParam(id, 
                                                           address, 
                                                           length);
  if (sdk_error.dxl_addparam_result != true)
  {
    if (log != NULL) *log = "grouBulkRead addparam failed";
    return false;
  }

  if (bulk_read_parameter_cnt_ < (MAX_BULK_PARAMETER-1))
  {
    bulk_read_param_[bulk_read_parameter_cnt_].id = id;
    bulk_read_param_[bulk_read_parameter_cnt_].address = address;
    bulk_read_param_[bulk_read_parameter_cnt_].data_length = length;
    bulk_read_parameter_cnt_++;
  }
  else
  {
    if (log != NULL) *log = "[DynamixelDriver] Too many bulk parameter are added (default buffer size is 10)";
    return false;
  }

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to add param for bulk read!";
  return true;
}

bool DynamixelDriver::addBulkReadParam(uint8_t id, const char *item_name, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  sdk_error.dxl_addparam_result = groupBulkRead_->addParam(id, 
                                                          control_item->address, 
                                                          control_item->data_length);
  if (sdk_error.dxl_addparam_result != true)
  {
    if (log != NULL) *log = "grouBulkRead addparam failed";
    return false;
  }

  if (bulk_read_parameter_cnt_ < (MAX_BULK_PARAMETER-1))
  {
    bulk_read_param_[bulk_read_parameter_cnt_].id = id;
    bulk_read_param_[bulk_read_parameter_cnt_].address = control_item->address;
    bulk_read_param_[bulk_read_parameter_cnt_].data_length = control_item->data_length;
    bulk_read_parameter_cnt_++;
  }
  else
  {
    if (log != NULL) *log = "[DynamixelDriver] Too many bulk parameter are added (default buffer size is 10)";
    return false;
  }

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to add param for bulk read!";
  return true;
}

bool DynamixelDriver::bulkRead(const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  
  sdk_error.dxl_comm_result = groupBulkRead_->txRxPacket();
  if (sdk_error.dxl_comm_result != COMM_SUCCESS) 
  {
    if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to bulk read!";
  return true;
}

bool DynamixelDriver::getBulkReadData(int32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  for (int i = 0; i < bulk_read_parameter_cnt_; i++)
  {
    sdk_error.dxl_getdata_result = groupBulkRead_->isAvailable(bulk_read_param_[i].id, 
                                                              bulk_read_param_[i].address, 
                                                              bulk_read_param_[i].data_length);
    if (sdk_error.dxl_getdata_result != true)
    {
      if (log != NULL) *log = "groupBulkRead getdata failed";
      return false;
    }
    else
    {
      data[i] = groupBulkRead_->getData(bulk_read_param_[i].id, 
                                        bulk_read_param_[i].address, 
                                        bulk_read_param_[i].data_length);
    }
  }

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to get bulk read data!";
  return true;
}

bool DynamixelDriver::getBulkReadData(uint8_t *id, uint8_t id_num, uint16_t *address, uint16_t *length, int32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  for (int i = 0; i < id_num; i++)
  {
    sdk_error.dxl_getdata_result = groupBulkRead_->isAvailable(id[i], 
                                                              address[i], 
                                                              length[i]);
    if (sdk_error.dxl_getdata_result != true)
    {
      if (log != NULL) *log = "groupBulkRead getdata failed";
      return false;
    }
    else
    {
      data[i] = groupBulkRead_->getData(id[i], 
                                        address[i], 
                                        length[i]);
    }
  }

  if (log != NULL) *log = "[DynamixelDriver] Succeeded to get bulk read data!";
  return true;
}

bool DynamixelDriver::clearBulkReadParam(void)
{
  groupBulkRead_->clearParam();
  bulk_read_parameter_cnt_ = 0;

  return true;
}
