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

/* Author: OpusK */
/* For API doc, please refer to https://github.com/ROBOTIS-GIT/OpenCR/wiki/arduino_examples_can */

uint32_t id, i;
can_msg_t rx_msg;
/*
 *  typedef struct 
 *  {
 *    uint32_t id      : Identifier of received message
 *    uint32_t length  : Length of received message data
 *    uint8_t  data[8] : Data of received message
 *  } can_msg_t;
*/

void setup()
{
  Serial.begin(115200);

  Serial.println("=================================");
  Serial.println("=== CAN RX Interrupt Example! ===");

  if (canOpen(_DEF_CAN_BAUD_125K, _DEF_CAN_EXT) == false)
  {
    Serial.println("CAN open fail!!");
  }
  else
  {
    id = 0x123;
    canConfigFilter(id, 0);
    canAttachRxInterrupt(canRxHandlerTemplate);
  }
}

void loop()
{
}

void canRxHandlerTemplate(can_msg_t *arg)
{
  if(canReadMsg(&rx_msg))
  {
    Serial.print("ID : ");
    Serial.print(arg->id, HEX);
    Serial.print(", Length : ");
    Serial.print(arg->length);
    Serial.print(", Data : ");
    for (i = 0; i < arg->length; i++)
    {
      Serial.print(arg->data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}
