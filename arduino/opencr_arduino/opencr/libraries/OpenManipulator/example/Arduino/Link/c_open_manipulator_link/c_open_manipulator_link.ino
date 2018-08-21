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

/* Authors: Hye-Jong KIM */

#include "OMLink.h"
//#include "Processing.h"

float present_time = 0.0;
float previous_time[3] = {0.0, 0.0, 0.0};

std::vector<float> init_joint_angle;
std::vector<float> target_angle;

std::vector<float> ac_angle_temp;
std::vector<float> ac_angle_min;
std::vector<float> ac_angle_max;
bool error_flug[3];
int8_t i_check = 0;

void setup() 
{
  Serial.begin(57600);
  DEBUG.begin(57600);

  while (!Serial)
    ;

  USB.println("Setup done");

  OM_PROCESSING::initProcessing(12);

  initOMLink();
  USB.println("Setup done");
  manipulator.actuatorEnable();
  manipulator.sendAllActuatorAngle(OMLINK, manipulator.getAllActiveJointAngle(OMLINK));

  previous_time[0] = (float)(millis()/1000.0f);
  previous_time[1] = (float)(millis()/1000.0f);
  previous_time[2] = (float)(millis()/1000.0f);
  USB.println("Setup done");
}

void loop() 
{
  present_time = (float)(millis()/1000.0f);
  
  MyFunction::getData(8);
  MyFunction::setMotion();

  if(error_flug[0])
  {
    if(present_time-previous_time[1] >= CONTROL_PERIOD)
    {
      MyFunction::jointMove(present_time);  
      previous_time[1] = (float)(millis()/1000.0f);
    } 
  }  

  if(present_time-previous_time[0] >= CONTROL_PERIOD)
  {
    manipulator.setAllActiveJointAngle(OMLINK, manipulator.receiveAllActuatorAngle(OMLINK));

    MyFunction::setPassiveJointAngle();
    manipulator.forward(OMLINK);

    if(send_processing_flug)
      {
        OM_PROCESSING::sendAngle2Processing(manipulator.getAllJointAngle(OMLINK));
        DEBUG.print(" angle : ");
        for(i_check=0; i_check < 3; i_check++)
        {
          if(manipulator.getAllJointAngle(OMLINK).at(i_check)<-2*M_PI)
          {
            error_flug[0] = false;
          }
          else if(manipulator.getAllJointAngle(OMLINK).at(i_check)>2*M_PI)
          {
            error_flug[0] = false;
          }
          else
          {
            error_flug[0] = true;
          }
          DEBUG.print(manipulator.getAllJointAngle(OMLINK).at(i_check+1));
          DEBUG.print(" , ");
        }
        DEBUG.println(" ");
      }
    previous_time[0] = (float)(millis()/1000.0f);
  } 
}


