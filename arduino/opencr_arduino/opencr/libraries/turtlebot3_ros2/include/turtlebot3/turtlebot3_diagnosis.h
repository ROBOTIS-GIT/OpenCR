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

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#ifndef TURTLEBOT3_DIAGNOSIS_H_
#define TURTLEBOT3_DIAGNOSIS_H_

#include <Arduino.h>

#define LED_TXD                          0
#define LED_RXD                          1
#define LED_LOW_BATTERY                  2
#define LED_ROS_CONNECT                  3
#define LED_WORKING_CHECK                13

#define BATTERY_POWER_OFF                0
#define BATTERY_POWER_STARTUP            1
#define BATTERY_POWER_NORMAL             2
#define BATTERY_POWER_CHECK              3
#define BATTERY_POWER_WARNNING           4

#define WAIT_FOR_BUTTON_PRESS            0
#define WAIT_SECOND                      1
#define CHECK_BUTTON_RELEASED            2

#define DEBUG_SERIAL  SerialBT2

class Turtlebot3Diagnosis
{
 public:
  Turtlebot3Diagnosis();
  ~Turtlebot3Diagnosis();

  bool init();

  void showLedStatus(bool isConnected);
  void updateRxTxLed(void);

  void setPowerOn(void);
  void setPowerOff(void);

  uint8_t updateVoltageCheck(bool check_setup);

  uint8_t getButtonPress(uint16_t time_to_press);  

 private:

};

#endif // TURTLEBOT3_DIAGNOSIS_H_