/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef DYNAMIXEL_PORT_HANDLER_H_
#define DYNAMIXEL_PORT_HANDLER_H_


#include <Arduino.h>

namespace DYNAMIXEL{

  class PortHandler
  {
    public:
      PortHandler();
      virtual void begin() = 0;
      virtual void end() = 0;
      virtual int available(void) = 0;
      virtual int read() = 0;
      //virtual size_t read(uint8_t *buf, size_t len);
      virtual size_t write(uint8_t) = 0;
      virtual size_t write(uint8_t *buf, size_t len) = 0;
      bool getOpenState();
      void setOpenState(bool);

    private:
      bool open_state_;
  };

  class SerialPortHandler : public PortHandler
  {
    public:
      SerialPortHandler(HardwareSerial& port, const int dir_pin = -1);

      virtual void begin() override;
      virtual void end() override;
      virtual int available(void) override;
      virtual int read() override;
      virtual size_t write(uint8_t) override;
      virtual size_t write(uint8_t *buf, size_t len) override;

      void begin(unsigned long baud);
      unsigned long getBaud() const;

    private:
      HardwareSerial& port_;
      const int dir_pin_;
      unsigned long baud_;
  };

  class USBSerialPortHandler : public PortHandler
  {
    public:
      USBSerialPortHandler(USBSerial& port);

      virtual void begin() override;
      virtual void end() override;
      virtual int available(void) override;
      virtual int read() override;
      virtual size_t write(uint8_t) override;
      virtual size_t write(uint8_t *buf, size_t len) override;

    private:
      USBSerial& port_;
  };


}

#endif /* DYNAMIXEL_PORT_HANDLER_H_ */