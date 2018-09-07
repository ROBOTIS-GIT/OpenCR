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

#ifndef MOTION_H_
#define MOTION_H_

#include "Chain.h"

#define MAX_MOTION_NUM 1

bool motion    = false;
uint8_t motion_cnt = 0;
uint8_t sub_motion_cnt = 0;

void gripOnOff(bool data);
void moveJS(float j1, float j2, float j3, float j4, float t);
bool pickStick1();
bool pickStick2();
bool pickStick3();
bool placeStick1();
bool placeStick2();
bool placeStick3();
bool shakeMotion1();
bool shakeMotion2();
bool shakeMotion3();
bool motion1();
bool motion2();

void setMotion()
{
  if(motion && !chain.moving() && !chain.drawing())
  {
    DEBUG.print(motion_cnt); DEBUG.print(", ");  DEBUG.println(sub_motion_cnt);
////////////////////////MOTION SETTING//////////////////////////////        
    switch(motion_cnt)
    {
    case 0:
      if(pickStick1())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 1:
      if(placeStick2())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 2:
      if(pickStick2())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 3:
      if(placeStick3())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 4:
      if(pickStick3())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 5:
      if(placeStick1())
      { sub_motion_cnt = 0; motion_cnt =0; }
      else 
        sub_motion_cnt ++;
    break;
    /*case 1:
      if(shakeMotion2())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 2:
      if(shakeMotion1())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 3:
      if(shakeMotion3())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 4:
      if(motion2())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 5:
      if(placeStick2())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 6:
      if(pickStick2())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 7:
      if(shakeMotion3())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 8:
      if(shakeMotion1())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 9:
      if(shakeMotion2())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 10:
      if(motion1())
      { sub_motion_cnt = 0; motion_cnt ++; }
      else 
        sub_motion_cnt ++;
    break;
    case 11:
      if(placeStick1())
      { sub_motion_cnt = 0; motion_cnt =0; }
      else 
        sub_motion_cnt ++;
    break;*/
    }
////////////////////////MOTION SETTING////////////////////////////// 
  }
}

void gripOnOff(bool data)
{
  if(data)
    chain.toolMove(TOOL, -1.0f); // gripper on
  else
    chain.toolMove(TOOL, 0.05f); // gripper off
}
void moveJS(float j1, float j2, float j3, float j4, float t)
{
  static std::vector <float> target_angle;
  target_angle.clear();
  target_angle.push_back(j1);
  target_angle.push_back(j2);
  target_angle.push_back(j3);
  target_angle.push_back(j4);
  chain.jointMove(target_angle,t);     
}
void moveCS(float x, float y, float z, float t)
{
  Pose target_pose = chain.getComponentPoseToWorld(TOOL);
  target_pose.position(0) = x;
  target_pose.position(1) = y;
  target_pose.position(2) = z;
  chain.setPose(TOOL, target_pose, t);
}

void motionStart()
{
  motion_cnt = 0;          
  sub_motion_cnt = 0;
  motion = true;
}

void motionStop()
{
  motion_cnt = 0;
  sub_motion_cnt = 0;
  motion     = false;
}

bool pickStick1()
{
  switch(sub_motion_cnt)
  {
  case 0: moveJS(0.0, -1.05, 0.35, 0.7, 2.0); gripOnOff(true); break;
  case 1: moveCS(-0.038, -0.01, 0.15, 1.5); break;
  case 2: chain.drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.100f), 1.0f); break;
  case 3: gripOnOff(false); chain.wait(1.5); break;
  case 4: chain.drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.050f), 0.6f);
          return 1; break;
  }
  return 0;
}
bool pickStick2()
{
  switch(sub_motion_cnt)
  {
  case 0: moveJS(-0.35, -1.05, 0.35, 0.7, 2.0); gripOnOff(true); break;
  case 1: moveCS(-0.04, -0.08, 0.15, 1.5); break;
  case 2: chain.drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.100f), 1.0f); break;
  case 3: gripOnOff(false); chain.wait(1.5); break;
  case 4: chain.drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.050f), 0.6f);
          return 1; break;
  }
  return 0;
}
bool pickStick3()
{
  switch(sub_motion_cnt)
  {
  case 0: moveJS(0.35, -1.05, 0.35, 0.7, 2.0); gripOnOff(true); break;
  case 1: moveCS(-0.035, 0.0725, 0.15, 1.5); break;
  case 2: chain.drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.100f), 1.0f); break;
  case 3: gripOnOff(false); chain.wait(1.5); break;
  case 4: chain.drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.050f), 0.6f);
          return 1; break;
  }
  return 0;
}

bool placeStick1()
{
  switch(sub_motion_cnt)
  {
  case 0: moveJS(0.0, -1.05, 0.35, 0.7, 2.0); break;
  case 1: moveCS(-0.038, -0.01, 0.15, 1.5); break;
  case 2: chain.drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.100f), 1.2f); break;
  case 3: gripOnOff(true); chain.wait(1.0); break;
  case 4: chain.drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.100f), 1.2f);
          return 1; break;
  }
  return 0;
}
bool placeStick2()
{
  switch(sub_motion_cnt)
  {
  case 0: moveJS(-0.35, -1.05, 0.35, 0.7, 2.0); break;
  case 1: moveCS(-0.04, -0.08, 0.15, 1.5); break;
  case 2: chain.drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.100f), 1.2f); break;
  case 3: gripOnOff(true); chain.wait(1.0); break;
  case 4: chain.drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.100f), 1.2f);
          return 1; break;
  }
  return 0;
}
bool placeStick3()
{
  switch(sub_motion_cnt)
  {
  case 0: moveJS(0.35, -1.05, 0.35, 0.7, 2.0); break;
  case 1: moveCS(-0.035, 0.0725, 0.15, 1.5); break;
  case 2: chain.drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.100f), 1.2f); break;
  case 3: gripOnOff(true); chain.wait(1.0); break;
  case 4: chain.drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.100f), 1.2f);
          return 1; break;
  }
  return 0;
}
bool shakeMotion1()
{
  switch(sub_motion_cnt)
  {
  case 0: moveJS(0.0, -1.05, 0.35,  0.7, 2.0);  break;
  case 1: moveJS(0.0, -0.92, 0.08, -0.64, 0.8); break;
  case 2: moveJS(0.0, -1.35, 0.81,  0.34, 0.8); break;
  case 3: moveJS(0.0, -0.92, 0.08, -0.64, 0.8); break;
  case 4: moveJS(0.0, -1.35, 0.81,  0.34, 0.8);
          return 1; break;
  }
  return 0;
}
bool shakeMotion3()
{
  switch(sub_motion_cnt)
  {
  case 0: moveJS(1.57, -1.05, 0.35,  0.7, 2.0);  break;
  case 1: moveJS(1.57, -0.92, 0.08, -0.64, 0.8); break;
  case 2: moveJS(1.57, -1.35, 0.81,  0.34, 0.8); break;
  case 3: moveJS(1.57, -0.92, 0.08, -0.64, 0.8); break;
  case 4: moveJS(1.57, -1.35, 0.81,  0.34, 0.8);
          return 1; break;
  }
  return 0;
}
bool shakeMotion2()
{
  switch(sub_motion_cnt)
  {
  case 0: moveJS(-1.57, -1.05, 0.35,  0.7, 2.0);  break;
  case 1: moveJS(-1.57, -0.92, 0.08, -0.64, 0.8); break;
  case 2: moveJS(-1.57, -1.35, 0.81,  0.34, 0.8); break;
  case 3: moveJS(-1.57, -0.92, 0.08, -0.64, 0.8); break;
  case 4: moveJS(-1.57, -1.35, 0.81,  0.34, 0.8);
          return 1; break;
  }
  return 0;
}
bool motion1()
{
  switch(sub_motion_cnt)
  {
  case 0: moveJS( 0.00, -1.05, 0.35, 0.7, 2.0);  break;
  case 1: moveJS(-2.00, -0.96, 1.12, -0.76, 2.0); break;
  case 2: moveJS(-0.52, -0.82, 0.21,  1.06, 2.0); break;
  case 3: moveJS( 0.52, -0.96, 1.12, -0.76, 2.0); break;
  case 4: moveJS( 2.00, -0.82, 0.21,  1.06, 2.0); 
          return 1; break;
  }
  return 0;
}
bool motion2()
{
  switch(sub_motion_cnt)
  {
  case 0: moveJS( 0.00, -1.05, 0.35, 0.7, 2.0);  break;
  case 1: moveJS( 2.00, -0.82, 0.21,  1.06, 2.0); break;
  case 2: moveJS( 0.52, -0.96, 1.12, -0.76, 2.0); break;
  case 3: moveJS(-0.52, -0.82, 0.21,  1.06, 2.0); break;
  case 4: moveJS(-2.00, -0.96, 1.12, -0.76, 2.0); 
          return 1; break;
  }
  return 0;
}


#endif
