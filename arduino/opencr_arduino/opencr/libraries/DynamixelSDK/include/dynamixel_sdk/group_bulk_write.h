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
/// @file The file for Dynamixel Bulk Write
/// @author Zerom, Leon (RyuWoon Jung)
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPBULKWRITE_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPBULKWRITE_H_


#include <map>
#include <vector>
#include "port_handler.h"
#include "packet_handler.h"

namespace dynamixel
{

////////////////////////////////////////////////////////////////////////////////
/// @brief The class for writing multiple Dynamixel data from different addresses with different lengths at once
////////////////////////////////////////////////////////////////////////////////
class WINDECLSPEC GroupBulkWrite
{
 private:
  PortHandler    *port_;
  PacketHandler  *ph_;

  std::vector<uint8_t>            id_list_;
  std::map<uint8_t, uint16_t>     address_list_;  // <id, start_address>
  std::map<uint8_t, uint16_t>     length_list_;   // <id, data_length>
  std::map<uint8_t, uint8_t *>    data_list_;     // <id, data>

  bool            is_param_changed_;

  uint8_t        *param_;
  uint16_t        param_length_;

  void    makeParam();

 public:
  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that Initializes instance for Bulk Write
  /// @param port PortHandler instance
  /// @param ph PacketHandler instance
  ////////////////////////////////////////////////////////////////////////////////
  GroupBulkWrite(PortHandler *port, PacketHandler *ph);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that calls clearParam function to clear the parameter list for Bulk Write
  ////////////////////////////////////////////////////////////////////////////////
  ~GroupBulkWrite() { clearParam(); }

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
  /// @brief The function that adds id, start_address, data_length to the Bulk Write list
  /// @param id Dynamixel ID
  /// @param start_address Address of the data for write
  /// @param data_length Length of the data for write
  /// @return false
  /// @return   when the ID exists already in the list
  /// @return or true
  ////////////////////////////////////////////////////////////////////////////////
  bool    addParam    (uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t *data);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that removes id from the Bulk Write list
  /// @param id Dynamixel ID
  ////////////////////////////////////////////////////////////////////////////////
  void    removeParam (uint8_t id);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that changes the data for write in id -> start_address -> data_length to the Bulk Write list
  /// @param id Dynamixel ID
  /// @param start_address Address of the data for write
  /// @param data_length Length of the data for write
  /// @param data for replacement
  /// @return false
  /// @return   when the ID doesn't exist in the list
  /// @return or true
  ////////////////////////////////////////////////////////////////////////////////
  bool    changeParam (uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t *data);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that clears the Bulk Write list
  ////////////////////////////////////////////////////////////////////////////////
  void    clearParam  ();

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that transmits the Bulk Write instruction packet which might be constructed by GroupBulkWrite::addParam function
  /// @return COMM_NOT_AVAILABLE
  /// @return   when the list for Bulk Write is empty
  /// @return   when Protocol1.0 has been used
  /// @return or the other communication results which come from PacketHandler::bulkWriteTxOnly
  ////////////////////////////////////////////////////////////////////////////////
  int     txPacket();
};

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPBULKWRITE_H_ */
