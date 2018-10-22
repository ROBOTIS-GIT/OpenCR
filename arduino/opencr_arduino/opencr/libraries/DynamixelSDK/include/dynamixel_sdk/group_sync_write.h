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

////////////////////////////////////////////////////////////////////////////////
/// @file The file for Dynamixel Sync Write
/// @author Zerom, Leon (RyuWoon Jung)
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCWRITE_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCWRITE_H_


#include <map>
#include <vector>
#include "port_handler.h"
#include "packet_handler.h"

namespace dynamixel
{

////////////////////////////////////////////////////////////////////////////////
/// @brief The class for writing multiple Dynamixel data from same address with same length at once
////////////////////////////////////////////////////////////////////////////////
class WINDECLSPEC GroupSyncWrite
{
 private:
  PortHandler    *port_;
  PacketHandler  *ph_;

  //std::vector<uint8_t>            id_list_;
  //std::map<uint8_t, uint8_t* >    data_list_; // <id, data>

  bool            is_user_buffer_;  // did the user setup this buffer?
  uint8_t         max_ids_;         // Max number of IDs we can handle
  uint8_t         count_ids_;       // Actual count of ids 

  uint8_t        *param_;           // this will hold our buffer.
  uint16_t        start_address_;
  uint16_t        data_length_;

  uint8_t *findParam(uint8_t id, bool add_if_not_found);

 public:
  
  ////////////////////////////////////////////////////////////////////////////////
  /// @brief - Lets sketches know how many extra bytes per servo to allocate
  ///  they can add this to number of bytes they write (data_length)
  // 
  ////////////////////////////////////////////////////////////////////////////////
  enum {EXTRA_BYTES_PER_ITEM = 1, DEFAULT_COUNT_MAX_IDS = 16 };

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that Initializes instance for Sync Write
  ///     Warning don't use this version for global objects, it will crash!
  /// @param port PortHandler instance
  /// @param ph PacketHandler instance
  /// @param start_address Address of the data for write
  /// @param data_length Length of the data for write
  /// @param max_ids max number of IDs we will use with this object... 
  ////////////////////////////////////////////////////////////////////////////////
  GroupSyncWrite(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length, 
      uint8_t max_ids=DEFAULT_COUNT_MAX_IDS);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief constructor first part of two part initialize use for global objects
  /// @param start_address Address of the data for write
  /// @param data_length Length of the data for write
  /// @param max_ids max number of IDs we will use with this object... 
  ////////////////////////////////////////////////////////////////////////////////
  GroupSyncWrite(uint16_t start_address, uint16_t data_length, 
      uint8_t max_ids=DEFAULT_COUNT_MAX_IDS);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief Init the second half of two part init
  /// @param port PortHandler instance
  /// @param ph PacketHandler instance
  ////////////////////////////////////////////////////////////////////////////////
  void    init(PortHandler *port, PacketHandler *ph);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief setBuffer allows the user to pass in the buffer to use
  /// @param buffer pointer to data buffer to use
  /// @param cb size of the buffer in bytes
  ////////////////////////////////////////////////////////////////////////////////
  bool  setBuffer(uint8_t *buffer_pointer, uint16_t buffer_size);  

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that calls clearParam function to clear the parameter list for Sync Write
  ////////////////////////////////////////////////////////////////////////////////
  ~GroupSyncWrite();

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that returns PortHandler instance
  /// @return PortHandler instance
  ////////////////////////////////////////////////////////////////////////////////
  PortHandler     *getPortHandler()   { return port_; }

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that returns PacketHandler instance
  /// @return PacketHandler instance
  ////////////////////////////////////////////////////////////////////////////////
  PacketHandler   *getPacketHandler() { return ph_; }

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that adds id, start_address, data_length to the Sync Write list
  /// @param id Dynamixel ID
  /// @param data Data for write
  /// @return false
  /// @return   when the ID exists already in the list
  /// @return or true
  ////////////////////////////////////////////////////////////////////////////////
  bool    addParam    (uint8_t id, uint8_t *data);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that adds id, start_address, data_length to the Sync Write list
  /// @param id Dynamixel ID
  /// @param start_address starting register number
  /// @param length how many bytes (1, 2, or 4)
  /// @param data Data for write
  /// @return false
  /// @return   when the ID exists already in the list
  /// @return or true
  ////////////////////////////////////////////////////////////////////////////////
  bool    setParam    (uint8_t id, uint16_t address, uint16_t data_length, uint32_t data);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that removes id from the Sync Write list
  /// @param id Dynamixel ID
  ////////////////////////////////////////////////////////////////////////////////
  void    removeParam (uint8_t id);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that changes the data for write in id -> start_address -> data_length to the Sync Write list
  /// @param id Dynamixel ID
  /// @param data for replacement
  /// @return false
  /// @return   when the ID doesn't exist in the list
  /// @return or true
  ////////////////////////////////////////////////////////////////////////////////
  bool    changeParam (uint8_t id, uint8_t *data);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that clears the Sync Write list
  ////////////////////////////////////////////////////////////////////////////////
  void    clearParam  ();

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that transmits the Sync Write instruction packet which might be constructed by GroupSyncWrite::addParam function
  /// @return COMM_NOT_AVAILABLE
  /// @return   when the list for Sync Write is empty
  /// @return or the other communication results which come from PacketHandler::syncWriteTxOnly
  ////////////////////////////////////////////////////////////////////////////////
  int     txPacket();
};

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCWRITE_H_ */
