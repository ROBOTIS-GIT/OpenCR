


#include "port_handler.h"

using namespace DYNAMIXEL;

PortHandler::PortHandler()
 : open_state_(false)
{}

/* PortHandler */
bool PortHandler::getOpenState()
{
  return open_state_;
}

void PortHandler::setOpenState(bool state)
{
  open_state_ = state;
}


/* SerialPortHandler */
SerialPortHandler::SerialPortHandler(HardwareSerial& port, const int dir_pin)
 : PortHandler(), port_(port), dir_pin_(dir_pin), baud_(57600)
{}

void SerialPortHandler::begin()
{
  begin(baud_);
}

void SerialPortHandler::begin(unsigned long baud)
{
  baud_ = baud;
  port_.begin(baud_);
  
  if(dir_pin_ != -1){
    pinMode(dir_pin_, OUTPUT);
    digitalWrite(dir_pin_, LOW);
    while(digitalRead(dir_pin_) != LOW);
  }

  setOpenState(true);
}

void SerialPortHandler::end(void)
{
  port_.end();
  setOpenState(false);
}

int SerialPortHandler::available(void)
{
  return port_.available();
}

int SerialPortHandler::read()
{
  return port_.read();
}

size_t SerialPortHandler::write(uint8_t c)
{
  size_t ret = 0;
  if(dir_pin_ != -1){
    digitalWrite(dir_pin_, HIGH);
    while(digitalRead(dir_pin_) != HIGH);
  }

  ret = port_.write(c);

  if(dir_pin_ != -1){
    port_.flush();
    digitalWrite(dir_pin_, LOW);
    while(digitalRead(dir_pin_) != LOW);
  }

  return ret;
}

size_t SerialPortHandler::write(uint8_t *buf, size_t len)
{
  size_t ret;
  if(dir_pin_ != -1){
    digitalWrite(dir_pin_, HIGH);
    while(digitalRead(dir_pin_) != HIGH);
  }

  ret = port_.write(buf, len);

  if(dir_pin_ != -1){
    port_.flush();
    digitalWrite(dir_pin_, LOW);
    while(digitalRead(dir_pin_) != LOW);
  }

  return ret;      
}

unsigned long SerialPortHandler::getBaud() const
{
  return baud_;
}


/* USBSerialPortHandler */
USBSerialPortHandler::USBSerialPortHandler(USBSerial& port)
 : PortHandler(), port_(port)
{}

void USBSerialPortHandler::begin()
{
  port_.begin(1000000);
  setOpenState(true);
}

void USBSerialPortHandler::end(void)
{
  port_.end();
  setOpenState(false);
}

int USBSerialPortHandler::available(void)
{
  return port_.available();
}

int USBSerialPortHandler::read()
{
  return port_.read();
}

size_t USBSerialPortHandler::write(uint8_t c)
{
  size_t ret = 0;

  ret = port_.write(c);

  return ret;
}

size_t USBSerialPortHandler::write(uint8_t *buf, size_t len)
{
  size_t ret;

  ret = port_.write(buf, len);

  return ret;      
}