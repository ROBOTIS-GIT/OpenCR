/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, Gilbert, HanCheol Cho */

#include <ros.h>
#include <std_msgs/Byte.h>
#include <OLLO.h>

OLLO touchOLLO;
ros::NodeHandle nh;

std_msgs::Byte bumper_msg;
ros::Publisher pub_bumper("bumper", &bumper_msg);



void setup()
{
  nh.initNode();
  nh.advertise(pub_bumper);
  
  touchOLLO.begin(3,TOUCH_SENSOR);//OLLO Touch Module must be connected at port 3.
  touchOLLO.begin(4,TOUCH_SENSOR);//OLLO Touch Module must be connected at port 4.
}

void loop()
{
  uint8_t reading = 0;
  static uint32_t pre_time;
 
  if (touchOLLO.read(3, TOUCH_SENSOR) == HIGH)
  {
    reading |= 0x02; // front side touch_sensor
  }
  
  if (touchOLLO.read(4, TOUCH_SENSOR) == HIGH)
  {
    reading |= 0x01; // back side touch_sensor
  }
  
  if (millis()-pre_time >= 50)
  {
    bumper_msg.data = reading;
    pub_bumper.publish(&bumper_msg);
    pre_time = millis();
  }
  
  nh.spinOnce();
}
