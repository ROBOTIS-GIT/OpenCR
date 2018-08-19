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

/* Authors: Darby Lim */

#ifndef REMOTE_CONTROLLER_H_
#define REMOTE_CONTROLLER_H_

#include "Chain.h"
#include <RC100.h>

RC100 rc100;

void connectRC100()
{
  rc100.begin(1);
}

void fromRC100()
{
  if (rc100.available())
  {
    uint16_t data = rc100.readData();

    if (data & RC100_BTN_U)
      chain.setMove(CHAIN, TOOL, OM_MATH::makeVector3(0.005, 0.0, 0.0), 0.1f);
    else if (data & RC100_BTN_D)
      chain.setMove(CHAIN, TOOL, OM_MATH::makeVector3(-0.005, 0.0, 0.0), 0.1f);
    else if (data & RC100_BTN_L)
      chain.setMove(CHAIN, TOOL, OM_MATH::makeVector3(0.0, -0.005, 0.0), 0.1f);
    else if (data & RC100_BTN_R)
      chain.setMove(CHAIN, TOOL, OM_MATH::makeVector3(0.0, 0.005, 0.0), 0.1f);
    else if (data & RC100_BTN_1)
      chain.setMove(CHAIN, TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.005), 0.1f);
    else if (data & RC100_BTN_3)
      chain.setMove(CHAIN, TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.005), 0.1f);
    else if (data & RC100_BTN_2)
    {
      float grip_pose = chain.getComponentToolValue(CHAIN, TOOL) + 0.010;
      chain.toolMove(CHAIN, TOOL, OM_MATH::map(grip_pose, 0.020f, 0.070f, 0.907f, -1.13f));
    }
    else if (data & RC100_BTN_4)
    {
      float grip_pose = chain.getComponentToolValue(CHAIN, TOOL) - 0.010;
      chain.toolMove(CHAIN, TOOL, OM_MATH::map(grip_pose, 0.020f, 0.070f, 0.907f, -1.13f));
    }
    else
      chain.setMove(CHAIN, TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.0), 0.1f);

    // else if (receive_data & RC100_BTN_5)
    // {
    //   // target_pos[1] = 0.0;
    //   // target_pos[2] = 60.0  * DEG2RAD;
    //   // target_pos[3] = -20.0 * DEG2RAD;
    //   // target_pos[4] = -40.0 * DEG2RAD;

    //   // setJointAngle(target_pos);
    //   // move();

    //   motion_cnt = 0;
    //   motion     = false;
    //   repeat     = false;
    // }
    // else if (receive_data & RC100_BTN_6)
    // {
    //   // target_pos[1] = 0.0;
    //   // target_pos[2] = 0.0;
    //   // target_pos[3] = 0.0;
    //   // target_pos[4] = 0.0;

    //   // setJointAngle(target_pos);
    //   // move();

    //   if (DYNAMIXEL)
    //       sendAngle2Processing(getAngle()); 

    //   if (PROCESSING)
    //     sendAngle2Processing(getState()); 

    //   for (int i = 0; i < MOTION_NUM; i++)
    //   {
    //     for (int j = 0; j < 6; j++)
    //     {
    //       motion_storage[i][j] = motion_set[i][j];
    //     }
    //   }

    //   motion_num = MOTION_NUM;  
    //   motion_cnt = 0;          
    //   motion = true;
    //   repeat = true;
    // }
  }
}

#endif