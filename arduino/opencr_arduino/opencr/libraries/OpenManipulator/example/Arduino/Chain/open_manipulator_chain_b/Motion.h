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

using namespace OPEN_MANIPULATOR;

bool motion[2]    = {false, false};
bool continue_flag = false;
uint8_t motion_cnt[2] = {0,0};
uint8_t sub_motion_cnt[2] = {0, 0};

void gripOnOff(OpenManipulator* manipulator, bool data)
{
  if(data)
    manipulator->toolMove(TOOL, -0.8f); // gripper open
  else
    manipulator->toolMove(TOOL, 0.02f); // gripper close
}
void moveJS(OpenManipulator* manipulator, float j1, float j2, float j3, float j4, float t)
{
  static std::vector <float> target_angle;
  target_angle.clear();
  target_angle.push_back(j1);
  target_angle.push_back(j2);
  target_angle.push_back(j3);
  target_angle.push_back(j4);
  manipulator->jointMove(target_angle,t);     
}
void moveCS(OpenManipulator* manipulator, float x, float y, float z, float t)
{
  Pose target_pose = manipulator->getComponentPoseToWorld(TOOL);
  target_pose.position(0) = x;
  target_pose.position(1) = y;
  target_pose.position(2) = z;
  manipulator->setPose(TOOL, target_pose, t);
}

void motionStart()
{
  motion_cnt[0] = 0;          
  motion_cnt[1] = 0;    
  sub_motion_cnt[0] = 0;
  sub_motion_cnt[1] = 0;
  motion[0] = true;
  motion[1] = true;
  continue_flag = false;
}

void motionStop()
{
  motion_cnt[0] = 0;          
  motion_cnt[1] = 0;          
  sub_motion_cnt[1] = 0;
  sub_motion_cnt[0] = 0;
  motion[0] = false;
  motion[1] = false;
}

bool pickStick1(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: gripOnOff(manipulator, true); moveJS(manipulator, 0.0, -1.05, 0.35, 0.7, 2.0);  break;
  case 1: 
    if(index == 0) moveCS(manipulator, -0.038, -0.005, 0.15, 1.5);
    else if(index == 1) moveCS(manipulator, -0.038, -0.00, 0.15, 1.5);
    break;
  case 2: 
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.105f), 1.0f); 
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.120f), 1.0f); 
    break;
  case 3: gripOnOff(manipulator, false); manipulator->wait(1.5); break;
  case 4: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.100f), 0.6f);
          return 1; break;
  }
  return 0;
}
bool pickStick2(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0:  gripOnOff(manipulator, true); moveJS(manipulator, -0.35, -1.05, 0.35, 0.7, 2.0); break;
  case 1: 
    if(index == 0) moveCS(manipulator, -0.035, -0.075, 0.15, 1.5); 
    else if(index == 1) moveCS(manipulator, -0.04, -0.078, 0.15, 1.5); 
    break;
  case 2: 
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.105f), 1.0f); 
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.120f), 1.0f);
    break;
  case 3: gripOnOff(manipulator, false); manipulator->wait(1.5); break;
  case 4: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.100f), 0.6f);
          return 1; break;
  }
  return 0;
}
bool pickStick3(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: gripOnOff(manipulator, true); moveJS(manipulator, 0.35, -1.05, 0.35, 0.7, 2.0); break;
  case 1: 
    if(index == 0) moveCS(manipulator, -0.035, 0.0725, 0.15, 1.5);
    else if(index == 1) moveCS(manipulator, -0.037, 0.0775, 0.15, 1.5);
    break;
  case 2: 
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.105f), 1.0f); 
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.120f), 1.0f); 
    break;
  case 3: gripOnOff(manipulator, false); manipulator->wait(1.5); break;
  case 4: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.100f), 0.6f);
          return 1; break;
  }
  return 0;
}

bool placeStick1(OpenManipulator* manipulator, int index, int opt)
{
  float depth = -0.105f;
  if(opt == 1)
    depth = -0.07f;
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, 0.0, -1.05, 0.35, 0.7, 2.0); break;
  case 1: 
    if(index == 0) moveCS(manipulator, -0.038, -0.005, 0.15, 1.5);
    else if(index == 1) moveCS(manipulator, -0.038, -0.00, 0.15, 1.5);
    break;
  case 2: 
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, depth), 1.0f); 
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, depth - 0.015f), 1.0f); 
    break;
  case 3: gripOnOff(manipulator, true); manipulator->wait(1.5); break;
  case 4: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.100f), 0.6f);
          return 1; break;
  }
  return 0;
}
bool placeStick2(OpenManipulator* manipulator, int index, int opt)
{
  float depth = -0.105f;
  if(opt == 1)
    depth = -0.07f;
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, -0.35, -1.05, 0.35, 0.7, 2.0); break;
  case 1: 
    if(index == 0) moveCS(manipulator, -0.035, -0.075, 0.15, 1.5); 
    else if(index == 1) moveCS(manipulator, -0.04, -0.078, 0.15, 1.5); 
    break;
  case 2: 
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, depth), 1.0f); 
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, depth - 0.015f), 1.0f); 
    break;
  case 3: gripOnOff(manipulator, true); manipulator->wait(1.5); break;
  case 4: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.100f), 0.6f);
          return 1; break;
  }
  return 0;
}
bool placeStick3(OpenManipulator* manipulator, int index, int opt)
{
  float depth = -0.105f;
  if(opt == 1)
    depth = -0.07f;

  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, 0.35, -1.05, 0.35, 0.7, 2.0); break;
  case 1: 
    if(index == 0) moveCS(manipulator, -0.035, 0.0725, 0.15, 1.5);
    else if(index == 1) moveCS(manipulator, -0.037, 0.0775, 0.15, 1.5);
    break;
  case 2: 
    if(index == 0) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, depth), 1.0f); 
    else if(index == 1) manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, depth - 0.015f), 1.0f); 
    break;
  case 3: gripOnOff(manipulator, true); manipulator->wait(1.5); break;
  case 4: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.100f), 0.6f);
          return 1; break;
  }
  return 0;
}
bool shakeMotion1(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, 0.0, -1.05, 0.35,  0.7, 2.0);  break;
  case 1: moveJS(manipulator, 0.0, -0.92, 0.08, -0.64, 0.8); break;
  case 2: moveJS(manipulator, 0.0, -1.35, 0.81,  0.34, 0.8); break;
  case 3: moveJS(manipulator, 0.0, -0.92, 0.08, -0.64, 0.8); break;
  case 4: moveJS(manipulator, 0.0, -1.35, 0.81,  0.34, 0.8); break;
  case 5: moveJS(manipulator, 0.0, -0.92, 0.08, -0.64, 0.8); 
          return 1; break;
  }
  return 0;
}
bool shakeMotion3(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, 0,    -1.05, 0.35,  0.7, 2.0);  break;
  case 1: moveJS(manipulator, 1.57, -0.92, 0.08, -0.64, 1.5); break;
  case 2: moveJS(manipulator, 1.57, -1.35, 0.81,  0.34, 0.8); break;
  case 3: moveJS(manipulator, 1.57, -0.92, 0.08, -0.64, 0.8); break;
  case 4: moveJS(manipulator, 1.57, -1.35, 0.81,  0.34, 0.8); break;
  case 5: moveJS(manipulator, 1.57, -0.92, 0.08, -0.64, 0.8); 
          return 1; break;
  }
  return 0;
}
bool shakeMotion2(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, 0,     -1.05, 0.35,  0.7, 2.0);  break;
  case 1: moveJS(manipulator, -1.57, -0.92, 0.08, -0.64, 1.5); break;
  case 2: moveJS(manipulator, -1.57, -1.35, 0.81,  0.34, 0.8); break;
  case 3: moveJS(manipulator, -1.57, -0.92, 0.08, -0.64, 0.8); break;
  case 4: moveJS(manipulator, -1.57, -1.35, 0.81,  0.34, 0.8); break;
  case 5: moveJS(manipulator, -1.57, -0.92, 0.08, -0.64, 0.8); 
          return 1; break;
  }
  return 0;
}
bool motion1(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator,  0.00, -1.05, 0.35, 0.7, 2.0);  break;
  case 1: moveJS(manipulator, -2.00, -1.22, 0.75, -0.76, 2.0); break;
  case 2: moveJS(manipulator, -0.52, -1.20, 0.05,  1.63, 2.0); break;
  case 3: moveJS(manipulator,  0.52, -1.22, 0.75, -0.76, 2.0); break;
  case 4: moveJS(manipulator,  2.00, -1.20, 0.05,  1.63, 2.0); 
          return 1; break;
  }
  return 0;
}
bool motion2(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator,  0.00, -1.05, 0.35, 0.7, 2.0);  break;
  case 1: moveJS(manipulator,  2.00, -1.20, 0.05,  1.63, 2.0); break;
  case 2: moveJS(manipulator,  0.52, -1.22, 0.75, -0.76, 2.0); break;
  case 3: moveJS(manipulator, -0.52, -1.20, 0.05,  1.63, 2.0); break;
  case 4: moveJS(manipulator, -2.00, -1.22, 0.75, -0.76, 2.0); 
          return 1; break;
  }
  return 0;
}


bool send1to2(OpenManipulator* manipulator, int index)
{
  if(index == 0)
  {
    switch(sub_motion_cnt[index])
    {
    case 0: moveJS(manipulator, 0.0, -1.05, 0.35, 0.7, 2.5); break; // init
    case 1: moveJS(manipulator, 1.57, -1.00, 1.44, -0.47, 1.5); break; // ready
    case 2: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.05f), 1.0f); break;
    case 3: manipulator->wait(1.5); break;
    case 4: gripOnOff(manipulator, true); manipulator->wait(1.5); break;
    case 5: moveJS(manipulator, 1.57, -1.00, 1.44, -0.47, 1.5); 
            return 1; break;
    }
  }
  else if(index == 1)
  {
    switch(sub_motion_cnt[index])
    {
    case 0: moveJS(manipulator, 0.0, -1.05, 0.35, 0.7, 2.5); gripOnOff(manipulator, true); break; // init
    case 1: moveJS(manipulator, -1.53, -0.71, -0.14, 0.82, 1.5); break; // ready
    case 2: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.09f), 1.0f); break;
    case 3: gripOnOff(manipulator, false); manipulator->wait(1.5); break;
    case 4: manipulator->wait(1.5); break;
    case 5: moveJS(manipulator, -1.53, -0.71, -0.14, 0.82, 1.5); 
            return 1; break;
    }
  }
  
  return 0;
}


bool send2to1(OpenManipulator* manipulator, int index)
{
  if(index == 0)  //recv
  {
    switch(sub_motion_cnt[index])
    {
    case 0: moveJS(manipulator, 0.0, -1.05, 0.35, 0.7, 2.5); gripOnOff(manipulator, true); break; // init
    case 1: moveJS(manipulator, 1.57, -0.71, -0.14, 0.82, 1.5); break; // ready
    case 2: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.09f), 1.0f); break;
    case 3: gripOnOff(manipulator, false); manipulator->wait(1.5); break;
    case 4: manipulator->wait(1.5); break;
    case 5: moveJS(manipulator, 1.57,  -0.71, -0.14, 0.82, 1.5); 
            return 1; break;
    }
  }
  else if(index == 1) //send
  {
    switch(sub_motion_cnt[index])
    {
    case 0: moveJS(manipulator, 0.0, -1.05, 0.35, 0.7, 2.5); break; // init
    case 1: moveJS(manipulator, -1.53, -1.00, 1.44, -0.47, 1.5); break; // ready
    case 2: manipulator->drawLine(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.05f), 1.0f); break;
    case 3: manipulator->wait(1.5); break;
    case 4: gripOnOff(manipulator, true); manipulator->wait(1.5); break;
    case 5: moveJS(manipulator, -1.53, -1.00, 1.44, -0.47, 1.5); 
            return 1; break;
    }
  }
  
  return 0;
}

bool motion_place_6_7(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator, -0.70, -0.61, 0.80,  -0.13, 2.0); gripOnOff(manipulator, false); break;
  case 1: moveJS(manipulator, -0.35,  0.01, 1.28,  -1.28, 1.6); break;
  case 2: moveJS(manipulator,  0.35,  0.01, 1.28,  -1.28, 1.7); break;
  case 3: moveJS(manipulator,  0.70, -0.61, 0.80,  -0.13, 1.7); break;
  case 4: moveJS(manipulator,  0.00, -0.97, 0.03,   0.96, 1.6); 
          return 1; break;
  }
  return 0;
}


bool motion_pick_6_6(OpenManipulator* manipulator, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: moveJS(manipulator,  0.00, -1.05,  0.35, 0.70, 2.0); gripOnOff(manipulator, false); break;
  case 1: moveJS(manipulator, -0.80, -0.95,  0.53, 0.42, 1.6); break;
  case 2: moveJS(manipulator,  0.00, -1.53,  1.40, 0.13, 1.7); break;
  case 3: moveJS(manipulator,  0.80, -0.95,  0.53, 0.42, 1.7); break;
  case 4: moveJS(manipulator,  0.00, -0.60, -0.57, 1.17, 1.6); 
          return 1; break;
  }
  return 0;
}

void setMotion1()
{
  if(motion[0] && !chain.checkManipulatorMoving())
  {
////////////////////////MOTION SETTING//////////////////////////////        
    switch(motion_cnt[0])
    {
    ////////// O X O 
    case 0:
      if(pickStick2(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 1:
      if(shakeMotion2(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 2:
      if(shakeMotion1(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 3:
      if(placeStick1(&chain, 0, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    ////////// X O O
    case 4:
      if(pickStick1(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 5:
      if(send1to2(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 6:
      if(motion_place_6_7(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    ////////// X X O 
    case 7:
      if(motion_pick_6_6(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 8:
      if(send2to1(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 9:
      if(placeStick2(&chain, 0, 1))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    ////////// O X O 
    case 10:
      if(pickStick3(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 11:
      if(motion1(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 12:
      if(placeStick1(&chain, 0, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    ////////// O O X 
    case 13:
      if(pickStick2(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 14:
      if(shakeMotion2(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 15:
      if(shakeMotion1(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 16:
      if(shakeMotion3(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    case 17:
      if(placeStick3(&chain, 0, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    ////////// X O O 
    case 18:
      if(motion_pick_6_6(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break; 
    case 19:
      if(send2to1(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break; 
    case 20:
      if(placeStick2(&chain, 0, 1))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break; 
    ////////// O O O 
    case 21:
      if(pickStick1(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break; 
    case 22:
      if(send1to2(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break; 
    case 23:
      if(motion_place_6_7(&chain, 0))
      { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
      else 
        sub_motion_cnt[0] ++;
    break;
    ////////// O X O
    case 24:
      moveJS(&chain, 0.0, -1.05, 0.35, 0.7, 2.0);
      motion_cnt[0] ++; 
      continue_flag = true;
    break;
    }
////////////////////////MOTION SETTING////////////////////////////// 
  }
}


void setMotion2()
{
  if(motion[1] && !chain2.checkManipulatorMoving())
  {
////////////////////////MOTION SETTING//////////////////////////////        
    switch(motion_cnt[1])
    {
    ////////// O X O 
    case 0:
      if(pickStick3(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 1:
      if(shakeMotion3(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 2:
      if(shakeMotion1(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 3:
      if(placeStick1(&chain2, 1, 0))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    ////////// O O X
    case 4:
      if(motion_pick_6_6(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 5:
      if(send1to2(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 6:
      if(placeStick3(&chain2, 1, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    ////////// O O O
    case 7:
      if(pickStick2(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 8:
      if(send2to1(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 9:
      if(motion_place_6_7(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    ////////// O X O
    case 10:
      if(pickStick2(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 11:
      if(motion1(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 12:
      if(placeStick1(&chain2, 1, 0))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    ////////// X O O
    case 13:
      if(pickStick3(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 14:
      if(shakeMotion2(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 15:
      if(shakeMotion1(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 16:
      if(shakeMotion3(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 17:
      if(placeStick2(&chain2, 1, 0))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    ////////// O O X
    case 18:
      if(pickStick1(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 19:
      if(send2to1(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 20:
      if(motion_place_6_7(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    ////////// O X X
    case 21:
      if(motion_pick_6_6(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 22:
      if(send1to2(&chain2, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    case 23:
      if(placeStick3(&chain2, 1, 1))
      { sub_motion_cnt[1] = 0; motion_cnt[1] ++; }
      else 
        sub_motion_cnt[1] ++;
    break;
    ////////// O X O
    case 24:
      moveJS(&chain2, 0.0, -1.05, 0.35, 0.7, 2.0);
      motion_cnt[1] ++; 
      continue_flag = true;
    break;
    }
////////////////////////MOTION SETTING////////////////////////////// 
  }
}

#endif
