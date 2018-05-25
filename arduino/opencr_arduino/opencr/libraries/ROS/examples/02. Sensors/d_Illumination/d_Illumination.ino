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

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho, Gilbert */

#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;
std_msgs::Float32 illumination_msg;
ros::Publisher pub_illumination("illumination", &illumination_msg);

void setup() 
{
  nh.initNode();
  nh.advertise(pub_illumination);
  
  Serial.begin(9600);
}
 
void loop()
{
  float reading = 0;
  static uint32_t pre_time;

  reading = analogRead(A1);
  
  if (millis()-pre_time >= 50)
  {
    illumination_msg.data = reading;
    pub_illumination.publish(&illumination_msg);
    pre_time = millis();
    Serial.println(illumination_msg.data);
  }
  nh.spinOnce();
}

