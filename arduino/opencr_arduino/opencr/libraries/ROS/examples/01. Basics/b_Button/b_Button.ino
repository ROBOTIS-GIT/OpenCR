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

#include <ros.h>
#include <std_msgs/Byte.h>


ros::NodeHandle nh;

std_msgs::Byte button_msg;
ros::Publisher pub_button("button", &button_msg);



void setup()
{
  nh.initNode();
  nh.advertise(pub_button);

  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);
}

void loop()
{
  uint8_t reading = 0;
  static uint32_t pre_time;


  if (digitalRead(BDPIN_PUSH_SW_1) == HIGH)
  {
    reading |= 0x01;
  }
  if (digitalRead(BDPIN_PUSH_SW_2) == HIGH)
  {
    reading |= 0x02;
  }

  if (millis()-pre_time >= 50)
  {
    button_msg.data = reading;
    pub_button.publish(&button_msg);
    pre_time = millis();
  }

  nh.spinOnce();
}
