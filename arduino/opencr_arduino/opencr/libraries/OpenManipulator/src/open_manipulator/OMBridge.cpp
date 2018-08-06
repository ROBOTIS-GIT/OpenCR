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

/* Authors: Hye-Jong KIM, Darby Lim */

#include "../../include/open_manipulator/OMBridge.h"

void PROCESSING::split(String data, char separator, String* temp)
{
  int cnt = 0;
  int get_index = 0;

  String copy = data;
  
  while(true)
  {
    get_index = copy.indexOf(separator);

	if(-1 != get_index)
	{
	  temp[cnt] = copy.substring(0, get_index);
  	  copy = copy.substring(get_index + 1);
	}
	else
	{
      temp[cnt] = copy.substring(0, copy.length());
	  break;
	}
	  ++cnt;
  }
}

  void PROCESSING::initProcessing(int8_t joint_angle_vector_size)
  {  
    for (int i = 0; i < joint_angle_vector_size; i++)
    {
      Serial.print(0.0);
      Serial.print(",");
    }

    Serial.println(0.0);
    delay(300);

    Serial.println("Init Processing");
  }

  void PROCESSING::sendAngle2Processing(std::vector<float> joint_angle_vector)
  {
    Serial.print("angle");

    for (int i = 0; i < joint_angle_vector.size(); i++)
    {
      Serial.print(",");
      Serial.print(joint_angle_vector.at(i));
    }
  Serial.println(" ");
  }


