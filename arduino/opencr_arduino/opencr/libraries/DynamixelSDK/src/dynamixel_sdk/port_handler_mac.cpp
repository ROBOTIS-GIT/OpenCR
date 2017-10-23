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

#if defined(__APPLE__)

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#include "port_handler_mac.h"

#define LATENCY_TIMER   8  // msec (USB latency timer)

using namespace dynamixel;

PortHandlerMac::PortHandlerMac(const char *port_name)
  : socket_fd_(-1),
    baudrate_(DEFAULT_BAUDRATE_),
    packet_start_time_(0.0),
    packet_timeout_(0.0),
    tx_time_per_byte(0.0)
{
  is_using_ = false;
  setPortName(port_name);
}

bool PortHandlerMac::openPort()
{
  return setBaudRate(baudrate_);
}

void PortHandlerMac::closePort()
{
  if(socket_fd_ != -1)
    close(socket_fd_);
  socket_fd_ = -1;
}

void PortHandlerMac::clearPort()
{
  tcflush(socket_fd_, TCIOFLUSH);
}

void PortHandlerMac::setPortName(const char *port_name)
{
  strcpy(port_name_, port_name);
}

char *PortHandlerMac::getPortName()
{
  return port_name_;
}

// TODO: baud number ??
bool PortHandlerMac::setBaudRate(const int baudrate)
{
  int baud = getCFlagBaud(baudrate);

  closePort();

  if(baud <= 0)   // custom baudrate
  {
    setupPort(B38400);
    baudrate_ = baudrate;
    return setCustomBaudrate(baudrate);
  }
  else
  {
    baudrate_ = baudrate;
    return setupPort(baud);
  }
}

int PortHandlerMac::getBaudRate()
{
  return baudrate_;
}

int PortHandlerMac::getBytesAvailable()
{
  int bytes_available;
  ioctl(socket_fd_, FIONREAD, &bytes_available);
  return bytes_available;
}

int PortHandlerMac::readPort(uint8_t *packet, int length)
{
  return read(socket_fd_, packet, length);
}

int PortHandlerMac::writePort(uint8_t *packet, int length)
{
  return write(socket_fd_, packet, length);
}

void PortHandlerMac::setPacketTimeout(uint16_t packet_length)
{
  packet_start_time_  = getCurrentTime();
  packet_timeout_     = (tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void PortHandlerMac::setPacketTimeout(double msec)
{
  packet_start_time_  = getCurrentTime();
  packet_timeout_     = msec;
}

bool PortHandlerMac::isPacketTimeout()
{
  if(getTimeSinceStart() > packet_timeout_)
  {
    packet_timeout_ = 0;
    return true;
  }
  return false;
}

double PortHandlerMac::getCurrentTime()
{
	struct timespec tv;
	clock_gettime( CLOCK_REALTIME, &tv);
	return ((double)tv.tv_sec*1000.0 + (double)tv.tv_nsec*0.001*0.001);
}

double PortHandlerMac::getTimeSinceStart()
{
  double time;

  time = getCurrentTime() - packet_start_time_;
  if(time < 0.0)
    packet_start_time_ = getCurrentTime();

  return time;
}

bool PortHandlerMac::setupPort(int cflag_baud)
{
  struct termios newtio;

  socket_fd_ = open(port_name_, O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(socket_fd_ < 0)
  {
    printf("[PortHandlerMac::SetupPort] Error opening serial port!\n");
    return false;
  }

  bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

  newtio.c_cflag = CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag      = 0;
  newtio.c_lflag      = 0;
  newtio.c_cc[VTIME]  = 0;
  newtio.c_cc[VMIN]   = 0;
  cfsetispeed(&newtio, cflag_baud);
  cfsetospeed(&newtio, cflag_baud);

  // clean the buffer and activate the settings for the port
  tcflush(socket_fd_, TCIFLUSH);
  tcsetattr(socket_fd_, TCSANOW, &newtio);

  tx_time_per_byte = (1000.0 / (double)baudrate_) * 10.0;
  return true;
}

bool PortHandlerMac::setCustomBaudrate(int speed)
{
  printf("[PortHandlerMac::SetCustomBaudrate] Not supported on Mac!\n");
  return false;
}

int PortHandlerMac::getCFlagBaud(int baudrate)
{
  switch(baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    // Mac OS doesn't support over B230400
    // case 460800:
    //   return B460800;
    // case 500000:
    //   return B500000;
    // case 576000:
    //   return B576000;
    // case 921600:
    //   return B921600;
    // case 1000000:
    //   return B1000000;
    // case 1152000:
    //   return B1152000;
    // case 1500000:
    //   return B1500000;
    // case 2000000:
    //   return B2000000;
    // case 2500000:
    //   return B2500000;
    // case 3000000:
    //   return B3000000;
    // case 3500000:
    //   return B3500000;
    // case 4000000:
    //   return B4000000;
    default:
      return -1;
  }
}

#endif
