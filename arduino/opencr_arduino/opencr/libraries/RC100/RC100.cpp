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

/* Author: Taehoon Lim (Darby), HanCheol Cho, Ashe Kim */

#include "RC100.h"

RC100::RC100()
{
}

RC100::~RC100()
{
}

void RC100::begin(int num)
{
  if(num == 1)
  {
    Serial2.begin(57600);
    number = num;
  }
  else if(num == 2)
  {
    Serial4.begin(57600);
    number = num;
  }
  else
  {
    Serial2.begin(57600);
    number = 1;
  }
  rc100_rx.state = 0;
  rc100_rx.index = 0;
  rc100_rx.received = false;
}

int RC100::available(void)
{
  if (number == 1)
  {
    if(Serial2.available())
    {
      return rc100Update(Serial2.read());
    }
  }
  else if (number == 2)
  {
    if(Serial4.available())
    {
      return rc100Update(Serial4.read());
    }
  }

  return false;
}

uint16_t RC100::readData(void)
{
  return rc100_rx.data;
}

bool RC100::rc100Update(uint8_t data)
{
  bool ret = false;
  static uint8_t save_data;
  static uint8_t inv_data;
  static uint32_t time_t;

  inv_data = ~data;

  if (millis()-time_t > 100)
  {
    rc100_rx.state = 0;
  }

  switch(rc100_rx.state)
  {
    case 0:
      if (data == 0xFF)
      {
        rc100_rx.state = 1;
        time_t = millis();
      }
      break;

    case 1:
      if (data == 0x55)
      {
        rc100_rx.state    = 2;
        rc100_rx.received = false;
        rc100_rx.data     = 0;
      }
      else
      {
        rc100_rx.state = 0;
      }
      break;

    case 2:
      rc100_rx.data  = data;
      save_data      = data;
      rc100_rx.state = 3;
      break;

    case 3:
      if (save_data == inv_data)
      {
        rc100_rx.state = 4;
      }
      else
      {
        rc100_rx.state = 0;
      }
      break;

    case 4:
      rc100_rx.data |= data<<8;
      save_data      = data;
      rc100_rx.state = 5;
      break;

    case 5:
      if (save_data == inv_data)
      {
        rc100_rx.received = true;
        ret = true;
      }
      rc100_rx.state = 0;
      break;

    default:
      rc100_rx.state = 0;
      break;
  }

  return ret;
}
