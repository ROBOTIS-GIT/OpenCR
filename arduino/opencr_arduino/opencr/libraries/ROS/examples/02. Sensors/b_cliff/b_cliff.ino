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
#include <std_msgs/UInt16.h>
#include <OLLO.h>

OLLO IROLLO;
ros::NodeHandle nh;
std_msgs::UInt16 cliff_msg;
ros::Publisher pub_cliff("cliff", &cliff_msg);

void setup()
{
  nh.initNode();
  nh.advertise(pub_cliff);

  Serial.begin(9600);
  
  IROLLO.begin(2, IR_SENSOR);//IR Module must be connected at port 1.
}

void loop()
{
  cliff_msg.data = IROLLO.read(2, IR_SENSOR);
  pub_cliff.publish(&cliff_msg);
    
  Serial.println(cliff_msg.data);
  delay(10);
  nh.spinOnce();
}
