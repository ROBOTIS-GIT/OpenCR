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
/// @file The file for port control
/// @author Zerom, Leon (RyuWoon Jung)
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLER_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLER_H_

#if defined(__linux__)
#define WINDECLSPEC
#elif defined(__APPLE__)
#define WINDECLSPEC
#elif defined(_WIN32) || defined(_WIN64)
  #ifdef WINDLLEXPORT
  #define WINDECLSPEC __declspec(dllexport)
  #else
  #define WINDECLSPEC __declspec(dllimport)
  #endif
#elif defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__)
#define WINDECLSPEC
#endif

#include <stdint.h>

namespace dynamixel
{

////////////////////////////////////////////////////////////////////////////////
/// @brief The class for port control that inherits PortHandlerLinux, PortHandlerWindows, PortHandlerMac, or PortHandlerArduino
////////////////////////////////////////////////////////////////////////////////
class WINDECLSPEC PortHandler
{
 public:
  static const int DEFAULT_BAUDRATE_ = 57600; ///< Default Baudrate

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that gets PortHandler class inheritance
  /// @description The function gets class inheritance (PortHandlerLinux / PortHandlerWindows / PortHandlerMac / PortHandlerArduino.
  ////////////////////////////////////////////////////////////////////////////////
  static PortHandler *getPortHandler(const char *port_name);

  bool   is_using_; ///< shows whether the port is in use

  virtual ~PortHandler() { }

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that opens the port
  /// @description The function calls PortHandlerLinux::setBaudRate() to open the port.
  /// @return communication results which come from PortHandlerLinux::setBaudRate()
  ////////////////////////////////////////////////////////////////////////////////
  virtual bool    openPort() = 0;

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that closes the port
  /// @description The function closes the port.
  ////////////////////////////////////////////////////////////////////////////////
  virtual void    closePort() = 0;

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that clears the port
  /// @description The function clears the port.
  ////////////////////////////////////////////////////////////////////////////////
  virtual void    clearPort() = 0;

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that flush the port
  /// @description The function waits until writes complete
  ////////////////////////////////////////////////////////////////////////////////
  virtual void    flushPort() {}

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that sets port name into the port handler
  /// @description The function sets port name into the port handler.
  /// @param port_name Port name
  ////////////////////////////////////////////////////////////////////////////////
  virtual void    setPortName(const char* port_name) = 0;

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that returns port name set into the port handler
  /// @description The function returns current port name set into the port handler.
  /// @return Port name
  ////////////////////////////////////////////////////////////////////////////////
  virtual char   *getPortName() = 0;

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that sets baudrate into the port handler
  /// @description The function sets baudrate into the port handler.
  /// @param baudrate Baudrate
  /// @return false
  /// @return   when error was occurred during port opening
  /// @return or true
  ////////////////////////////////////////////////////////////////////////////////
  virtual bool    setBaudRate(const int baudrate) = 0;

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that returns current baudrate set into the port handler
  /// @description The function returns current baudrate set into the port handler.
  /// @return Baudrate
  ////////////////////////////////////////////////////////////////////////////////
  virtual int     getBaudRate() = 0;

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that checks how much bytes are able to be read from the port buffer
  /// @description The function checks how much bytes are able to be read from the port buffer
  /// @description and returns the number.
  /// @return Length of read-able bytes in the port buffer
  ////////////////////////////////////////////////////////////////////////////////
  virtual int     getBytesAvailable() = 0;

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that reads bytes from the port buffer
  /// @description The function gets bytes from the port buffer,
  /// @description and returns a number of bytes read.
  /// @param packet Buffer for the packet received
  /// @param length Length of the buffer for read
  /// @return -1
  /// @return   when error was occurred
  /// @return or Length of bytes read
  ////////////////////////////////////////////////////////////////////////////////
  virtual int     readPort(uint8_t *packet, int length) = 0;

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that writes bytes on the port buffer
  /// @description The function writes bytes on the port buffer,
  /// @description and returns a number of bytes which are successfully written.
  /// @param packet Buffer which would be written on the port buffer
  /// @param length Length of the buffer for write
  /// @return -1
  /// @return   when error was occurred
  /// @return or Length of bytes written
  ////////////////////////////////////////////////////////////////////////////////
  virtual int     writePort(uint8_t *packet, int length) = 0;

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that sets and starts stopwatch for watching packet timeout
  /// @description The function sets the stopwatch by getting current time and the time of packet timeout with packet_length.
  /// @param packet_length Length of the packet expected to be received
  ////////////////////////////////////////////////////////////////////////////////
  virtual void    setPacketTimeout(uint16_t packet_length) = 0;

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that sets and starts stopwatch for watching packet timeout
  /// @description The function sets the stopwatch by getting current time and the time of packet timeout with msec.
  /// @param packet_length Length of the packet expected to be received
  ////////////////////////////////////////////////////////////////////////////////
  virtual void    setPacketTimeout(double msec) = 0;

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that checks whether packet timeout is occurred
  /// @description The function checks whether current time is passed by the time of packet timeout from the time set by PortHandlerLinux::setPacketTimeout().
  ////////////////////////////////////////////////////////////////////////////////
  virtual bool    isPacketTimeout() = 0;
};

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLER_H_ */
