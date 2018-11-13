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

/* Author: Kei */

#include <CAN.h>

#define SEND_INTERVAL_MS 1000

uint32_t t_time, id, i;
can_message_t tx_msg, rx_msg;
/*
 *  typedef struct 
 *  {
 *    uint32_t id      : Identifier of received message
 *    uint32_t length  : Length of received message data
 *    uint8_t  data[8] : Data of received message
 *    uint8_t  format  : Type of ID
 *  } can_message_t;
 * 
 * BAUDRATE :
 *   CAN_BAUD_125K
 *   CAN_BAUD_250K
 *   CAN_BAUD_500K
 *   CAN_BAUD_1000K
 * 
 * FORMAT :
 *   CAN_STD_FORMAT
 *   CAN_EXT_FORMAT
*/

void setup()
{
  Serial.begin(115200);

  Serial.println("============================");
  Serial.println("=== CAN Message Example! ===");

  if (CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT) == false)
  {
    Serial.println("CAN open fail!!");
  }
  else
  {
    id = 0x123;
    
    CanBus.configFilter(id, 0, CAN_EXT_FORMAT);
  }
}

void loop()
{
  if (CanBus.avaliableMessage())
  {
    if(CanBus.readMessage(&rx_msg))
    {
      Serial.print("ID : ");
      Serial.print(rx_msg.id, HEX);
      Serial.print(", Length : ");
      Serial.print(rx_msg.length);
      Serial.print(", Data : ");
      for (i = 0; i < rx_msg.length; i++)
      {
        Serial.print(rx_msg.data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }

  if (millis() - t_time >= SEND_INTERVAL_MS)
  {
    t_time = millis();

    tx_msg.id = 0x123;
    tx_msg.format = CAN_EXT_FORMAT;
    for(i = 0; i < 8; i++)
    {
      tx_msg.data[i] = (uint8_t)micros() + i;
    }
    tx_msg.length = i;

    CanBus.writeMessage(&tx_msg);
  }
}
