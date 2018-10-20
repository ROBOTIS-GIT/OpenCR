/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: zerom, Ryu Woon Jung (Leon) */

#include <algorithm>

#if defined(__linux__)
#include "group_sync_write.h"
#elif defined(__APPLE__)
#include "group_sync_write.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "group_sync_write.h"
#elif defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__)
#include "../../include/dynamixel_sdk/group_sync_write.h"
#endif

using namespace dynamixel;

GroupSyncWrite::GroupSyncWrite(PortHandler *port, PacketHandler *ph, 
      uint16_t start_address, uint16_t data_length, uint8_t max_ids)
  : port_(port),
    ph_(ph),
    is_user_buffer_(false),
    max_ids_(max_ids),
    count_ids_(0),
    param_(0),
    start_address_(start_address),
    data_length_(data_length)
{
  clearParam();
}

GroupSyncWrite::GroupSyncWrite(uint16_t start_address, uint16_t data_length, uint8_t max_ids)
  : port_(NULL),
    ph_(NULL),
    is_user_buffer_(false),
    max_ids_(max_ids),
    count_ids_(0),
    param_(0),
    start_address_(start_address),
    data_length_(data_length)
{
}

GroupSyncWrite::~GroupSyncWrite()
{
  if (!is_user_buffer_ && param_)
  {
    delete[] param_;
    param_ = NULL;  // probably not needed
  }
}

void GroupSyncWrite::init(PortHandler *port, PacketHandler *ph)
{
  port_ = port;
  ph_ = ph;
  //Serial.printf("GroupSyncWrite::Init called %x %x\n", (uint32_t)port_, (uint32_t)ph_); Serial.flush();
  clearParam();
}

bool GroupSyncWrite::setBuffer(uint8_t *buffer_pointer, uint16_t buffer_size)
{
  // See if there already was a buffer 
  if (!is_user_buffer_ && param_)
  {
    delete[] param_;
  }  
  // User passed in a buffer.
  param_ = buffer_pointer;
  count_ids_ = 0; 
  if (buffer_pointer)
  {
    is_user_buffer_ = true;
    max_ids_ = buffer_size / (data_length_ + 1);  // calculate how many servos this buffer could service
    return (max_ids_ > 0);
  }
  is_user_buffer_ = false;
  return true;
}  


uint8_t *GroupSyncWrite::findParam(uint8_t id)
{
  if (!param_) return NULL;

  uint8_t *pb = param_;
  for (uint8_t i = 0; i < count_ids_; i++)
  {
    if (*pb == id)
      return pb + 1;  // return item in Parameter list for this ID
    pb += (data_length_ + 1); // look at next element
  }
  return NULL;  // not found

}


bool GroupSyncWrite::addParam(uint8_t id, uint8_t *data)
{
  if (!ph_)
    return false;

  if (!param_) 
  {
    param_ = new uint8_t[max_ids_ * (1 + data_length_)]; // ID(1) + DATA(data_length)
    is_user_buffer_ = false;
    if (!param_)
      return false; 
  }

  // 
  uint8_t *item_pointer = findParam(id);  
  if (!item_pointer)
  {
    if (count_ids_ >= max_ids_)
      return false;
    item_pointer = param_ + (data_length_ + 1)*count_ids_;
    count_ids_++;
    *item_pointer++ = id; // save away the ID
  }

  for (uint16_t c = 0; c < data_length_; c++)
  {
    *item_pointer++ = *data++;
  }
  return true;
}

bool GroupSyncWrite::setParam (uint8_t id, uint16_t address, uint16_t data_length, uint32_t data)
{
  // Like addParam, but can set a subset and knows about 1 byte 2 byte and 4 byte fields...
  if (!ph_)
    return false;
  //Serial.printf("\nGroupSyncWrite(%u %d)::setParam %u %u %u %d ", start_address_, data_length_, id, address, data_length, data);
  // make sure address is valid
  if ((address < start_address_) || ((start_address_ + data_length_ - data_length) < address))
    return false;

  if (!param_) 
  {
    param_ = new uint8_t[max_ids_ * (1 + data_length_)]; // ID(1) + DATA(data_length)
    is_user_buffer_ = false;
    if (!param_)
      return false; 
  }

  // 
  uint8_t *item_pointer = findParam(id);  
  
  if (!item_pointer)
  {
    //Serial.print(" new item ");
    if (count_ids_ >= max_ids_)
      return false;
    item_pointer = param_ + (data_length_ + 1)*count_ids_;
    count_ids_++;
    *item_pointer++ = id; // save away the ID
    if (data_length != data_length_) 
    {
      memset(item_pointer, 0, data_length_); // initialize full area
    }
  }

  item_pointer += address - start_address_; // start of where we are writing
  //Serial.println((uint32_t)item_pointer, HEX);

  switch (data_length) 
  {
    case 1: 
      *item_pointer = DXL_LOBYTE(DXL_LOWORD(data));
      break;
    case 2:
      *item_pointer++ = DXL_LOBYTE(DXL_LOWORD(data));
      *item_pointer =   DXL_HIBYTE(DXL_LOWORD(data));
      break;
    case 4:
      *item_pointer++ = DXL_LOBYTE(DXL_LOWORD(data));
      *item_pointer++ = DXL_HIBYTE(DXL_LOWORD(data));
      *item_pointer++ = DXL_LOBYTE(DXL_HIWORD(data));
      *item_pointer   = DXL_HIBYTE(DXL_HIWORD(data));
      break;
    default:
      return false;
    }

  return true;
}


void GroupSyncWrite::removeParam(uint8_t id)
{
  if (!ph_)
    return;
  
  uint8_t *item_pointer = findParam(id);  
  if (!item_pointer)
    return;

  // copy data down;
  uint8_t *next_item_pointer = item_pointer += (data_length_ + 1);

  uint8_t *end_pointer = param_ + (data_length_ + 1)*count_ids_;

  while (next_item_pointer < end_pointer) 
  {
    *item_pointer++ = *next_item_pointer++;
  }
  count_ids_--; // decrement the count;
}

bool GroupSyncWrite::changeParam(uint8_t id, uint8_t *data)
{
  // in this version just use addParam
  return addParam(id, data);
}

void GroupSyncWrite::clearParam()
{
  count_ids_ = 0;
}  


int GroupSyncWrite::txPacket()
{
  if (!ph_)
    return COMM_NOT_AVAILABLE;
  
  if (count_ids_ == 0)
    return COMM_NOT_AVAILABLE;


  return ph_->syncWriteTxOnly(port_, start_address_, data_length_, param_, count_ids_ * (1 + data_length_));
}
