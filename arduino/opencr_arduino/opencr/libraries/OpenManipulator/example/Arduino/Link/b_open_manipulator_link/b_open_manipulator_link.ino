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
#include "RemoteController.h"
//#include "Processing.h"

float present_time = 0.0;
float previous_time[3] = {0.0, 0.0, 0.0};

std::vector<float> init_joint_angle;

std::vector<float> ac_angle_temp;
std::vector<float> ac_angle_min;
std::vector<float> ac_angle_max;
bool error_flug[3] = {true, true, true};
int8_t i_check = 0;

void setup() 
{
  Serial.begin(57600);
  DEBUG.begin(57600);

  // while (!Serial)
  //   ;
  
  ac_angle_min.push_back(-180.0*DEG2RAD);
  ac_angle_max.push_back(180.0*DEG2RAD);
  ac_angle_min.push_back(-135.0*DEG2RAD);
  ac_angle_max.push_back(0.0*DEG2RAD);
  ac_angle_min.push_back(-180.0*DEG2RAD);
  ac_angle_max.push_back(-90.0*DEG2RAD);

  OM_PROCESSING::initProcessing(12);
  connectRC100();
  initOMLink();

  ///////////////first move//////////////////////
  init_joint_angle.push_back(0.0*DEG2RAD);
  init_joint_angle.push_back(-90.0*DEG2RAD);
  init_joint_angle.push_back(-160.0*DEG2RAD);

  previous_goal_angle = manipulator.getAllActiveJointAngle();
  MyFunction::updateJointTrajectory(init_joint_angle, 3.0f);
  init_joint_angle.clear();

  manipulator.actuatorEnable();
  present_time = (float)(millis()/1000.0f);
  previous_time[0] = (float)(millis()/1000.0f);
  previous_time[1] = (float)(millis()/1000.0f);
  previous_time[2] = (float)(millis()/1000.0f);

  

  while(present_time-previous_time[2]<3.0f)
  {
    present_time = (float)(millis()/1000.0f);
    MyFunction::jointMove(present_time); 
  }
  ///////////////////////////////////////////////
}

void loop() 
{
  present_time = (float)(millis()/1000.0f);
  MyFunction::getData(8);
  MyFunction::setMotion();
  
  if(!error_flug[0]){manipulator.actuatorDisable();}
  else if(!error_flug[1]){manipulator.actuatorDisable();}
  else if(!error_flug[2]){manipulator.actuatorDisable();}
  else
  {
    if(present_time-previous_time[1] >= CONTROL_PERIOD)
    {
      MyFunction::jointMove(present_time);  
      previous_time[1] = (float)(millis()/1000.0f);
    }
    // DEBUG.println("jointMove");
  }  
  
  if(present_time-previous_time[0] >= CONTROL_PERIOD)
  {
    manipulator.setAllActiveJointAngle(manipulator.receiveAllActuatorAngle());
    MyFunction::setPassiveJointAngle();
    manipulator.forward();

    if(send_processing_flug)
      {
        OM_PROCESSING::sendAngle2Processing(manipulator.getAllJointAngle());
        //DEBUG.print(" angle : ");
        for(i_check=0; i_check < 3; i_check++)
        {
          if(manipulator.getAllActiveJointAngle().at(i_check)<ac_angle_min.at(i_check))
          {
            error_flug[i_check] = false;
            DEBUG.print(i_check);
            DEBUG.print(" : range over");
          }
          else if(manipulator.getAllActiveJointAngle().at(i_check)>ac_angle_max.at(i_check))
          {
            error_flug[i_check] = false;
            DEBUG.print(i_check);
            DEBUG.print(" : range over");
          }
          else
          {
            error_flug[i_check] = true;
            // DEBUG.print(i_check);
            // DEBUG.println(" : move flug");
          }
          //DEBUG.print(manipulator.getAllActiveJointAngle(OMLINK).at(i_check));
          //DEBUG.print(" , ");
        }
        //DEBUG.println(" ");
      }
    previous_time[0] = (float)(millis()/1000.0f);
  }
}


