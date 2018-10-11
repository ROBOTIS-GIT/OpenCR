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

/* Author: Ryu Woon Jung (Leon) */

#if defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__)

#include <Arduino.h>


#include "../../include/dynamixel_sdk/port_handler_arduino.h"

#if defined (__OPENCR__)
#define DYNAMIXEL_SERIAL  Serial3
#endif

#define LATENCY_TIMER     4  // msec (USB latency timer)

using namespace dynamixel;

PortHandlerArduino::PortHandlerArduino(const char *port_name)
  : baudrate_(DEFAULT_BAUDRATE_),
    packet_start_time_(0.0),
    packet_timeout_(0.0),
    tx_time_per_byte(0.0)
{
  is_using_ = false;
  setPortName(port_name);

#if defined(__OPENCR__)
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);

  setPowerOff();

  // Setup to use our automatic Half duplex IO pin
  DYNAMIXEL_SERIAL.transmitterEnable(BDPIN_DXL_DIR);

#elif defined(__OPENCM904__)
  if (port_name[0] == '1')
  {
    socket_fd_ = 0;
    p_dxl_serial = &Serial1;
  }
  else if (port_name[0] == '2')
  {
    socket_fd_ = 1;
    p_dxl_serial = &Serial2;
  }
  else if (port_name[0] == '3')
  {
    socket_fd_ = 2;
    p_dxl_serial = &Serial3;
  }
  else
  {
    socket_fd_ = 0;
    p_dxl_serial = &Serial1;
  }

  drv_dxl_begin(socket_fd_);
#endif

  setTxDisable();
}

bool PortHandlerArduino::openPort()
{
#if defined(__OPENCR__)
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);

  setPowerOn();
  delay(1000);
#endif

  return setBaudRate(baudrate_);
}

void PortHandlerArduino::closePort()
{
  setPowerOff();
}

void PortHandlerArduino::clearPort()
{
#if defined(__OPENCR__)
  DYNAMIXEL_SERIAL.flush();
#elif defined(__OPENCM904__)
  p_dxl_serial->flush();
#endif
}

void PortHandlerArduino::flushPort()
{
#if defined(__OPENCR__)
  DYNAMIXEL_SERIAL.flush();
#elif defined(__OPENCM904__)
  p_dxl_serial->flush();   
#endif
}

void PortHandlerArduino::setPortName(const char *port_name)
{
  strcpy(port_name_, port_name);
}

char *PortHandlerArduino::getPortName()
{
  return port_name_;
}

bool PortHandlerArduino::setBaudRate(const int baudrate)
{
  baudrate_ = checkBaudrateAvailable(baudrate);

  if (baudrate_ == -1)
    return false;

  setupPort(baudrate_);

  return true;
}

int PortHandlerArduino::getBaudRate()
{
  return baudrate_;
}

int PortHandlerArduino::getBytesAvailable()
{
  int bytes_available;

#if defined(__OPENCR__)
  bytes_available = DYNAMIXEL_SERIAL.available();
#elif defined(__OPENCM904__)
  bytes_available = p_dxl_serial->available();
#endif

  return bytes_available;
}

int PortHandlerArduino::readPort(uint8_t *packet, int length)
{
  int rx_length;

#if defined(__OPENCR__)
  flushPort();  // Make sure any pending outputs had previously completed. 
  rx_length = DYNAMIXEL_SERIAL.available();
#elif defined(__OPENCM904__)
  rx_length = p_dxl_serial->available();
#endif

  if (rx_length > length)
    rx_length = length;

  for (int i = 0; i < rx_length; i++)
  {
#if defined(__OPENCR__)
    packet[i] = DYNAMIXEL_SERIAL.read();
#elif defined(__OPENCM904__)
    packet[i] = p_dxl_serial->read();
#endif
  }

  return rx_length;
}

int PortHandlerArduino::writePort(uint8_t *packet, int length)
{
  int length_written;

  setTxEnable();

#if defined(__OPENCR__)
  length_written = DYNAMIXEL_SERIAL.write(packet, length);
#elif defined(__OPENCM904__)
  length_written = p_dxl_serial->write(packet, length);
#endif

  setTxDisable();

  return length_written;
}

void PortHandlerArduino::setPacketTimeout(uint16_t packet_length)
{
  packet_start_time_  = getCurrentTime();
  packet_timeout_     = (tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void PortHandlerArduino::setPacketTimeout(double msec)
{
  packet_start_time_  = getCurrentTime();
  packet_timeout_     = msec;
}

bool PortHandlerArduino::isPacketTimeout()
{
  if (getTimeSinceStart() > packet_timeout_)
  {
    packet_timeout_ = 0;
    return true;
  }

  return false;
}

double PortHandlerArduino::getCurrentTime()
{
	return (double)millis();
}

double PortHandlerArduino::getTimeSinceStart()
{
  double elapsed_time;

  elapsed_time = getCurrentTime() - packet_start_time_;
  if (elapsed_time < 0.0)
    packet_start_time_ = getCurrentTime();

  return elapsed_time;
}

bool PortHandlerArduino::setupPort(int baudrate)
{
#if defined(__OPENCR__)
  DYNAMIXEL_SERIAL.begin(baudrate);
#elif defined(__OPENCM904__)
  p_dxl_serial->setDxlMode(true);
  p_dxl_serial->begin(baudrate);
#endif

  delay(100);

  tx_time_per_byte = (1000.0 / (double)baudrate) * 10.0;
  return true;
}

int PortHandlerArduino::checkBaudrateAvailable(int baudrate)
{
  switch(baudrate)
  {
    case 9600:
      return 9600;
    case 57600:
      return 57600;
    case 115200:
      return 115200;
    case 1000000:
      return 1000000;
    case 2000000:
      return 2000000;
    case 3000000:
      return 3000000;
    case 4000000:
      return 4000000;
    case 4500000:
      return 4500000;
    default:
      return -1;
  }
}

void PortHandlerArduino::setPowerOn()
{
#if defined(__OPENCR__)
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
#endif
}

void PortHandlerArduino::setPowerOff()
{
#if defined(__OPENCR__)
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
#endif
}

void PortHandlerArduino::setTxEnable()
{
#if defined(__OPENCR__)
//  digitalWriteFast(2, HIGH);
//  drv_dxl_tx_enable(TRUE);
#elif defined(__OPENCM904__)
  drv_dxl_tx_enable(socket_fd_, TRUE);
#endif
}

void PortHandlerArduino::setTxDisable()
{
#if defined(__OPENCR__)
  DYNAMIXEL_SERIAL.flush(); // make sure it completes before we disable... 
//  drv_dxl_tx_enable(FALSE);
//  digitalWriteFast(2, LOW);
#elif defined(__OPENCM904__)
  drv_dxl_tx_enable(socket_fd_, FALSE);
#endif
}

#endif
