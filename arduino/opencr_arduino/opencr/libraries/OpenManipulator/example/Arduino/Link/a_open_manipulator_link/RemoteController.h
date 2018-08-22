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

int availableRC100()
{
  return rc100.available();
}

uint16_t readRC100Data()
{
  return rc100.readData();
}

void fromRC100(uint16_t data)
{
  if (data & RC100_BTN_U)
    chain.setMove(TOOL, OM_MATH::makeVector3(0.005f, 0.0, 0.0), 0.1f);
  else if (data & RC100_BTN_D)
    chain.setMove(TOOL, OM_MATH::makeVector3(-0.005f, 0.0, 0.0), 0.1f);
  else if (data & RC100_BTN_L)
    chain.setMove(TOOL, OM_MATH::makeVector3(0.0, 0.005f, 0.0), 0.1f);
  else if (data & RC100_BTN_R)
    chain.setMove(TOOL, OM_MATH::makeVector3(0.0, -0.005f, 0.0), 0.1f);
  else if (data & RC100_BTN_1)
    chain.setMove(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.005f), 0.1f);
  else if (data & RC100_BTN_3)
    chain.setMove(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.005f), 0.1f);
  else if (data & RC100_BTN_2)
  {
    float grip_value = chain.getComponentToolValue(TOOL) + 0.030f;
    if (grip_value >= 0.907f)
      grip_value = 0.907f;

    chain.toolMove(TOOL, grip_value);
  }
  else if (data & RC100_BTN_4)
  {
    float grip_value = chain.getComponentToolValue(TOOL) - 0.030f;
    if (grip_value <= -1.130f)
      grip_value = -1.130f;

    chain.toolMove(TOOL, grip_value);
  }
  else if (data & RC100_BTN_5)
  {
    std::vector<float> goal_position;

    goal_position.push_back(0.0f);
    goal_position.push_back(-60.0f * DEG2RAD);
    goal_position.push_back(20.0f * DEG2RAD);
    goal_position.push_back(40.0f * DEG2RAD);

    chain.jointMove(goal_position, 1.0f);
  }
  else if (data & RC100_BTN_6)
  {
    std::vector<float> goal_position;

    goal_position.push_back(0.0f);
    goal_position.push_back(0.0f);
    goal_position.push_back(0.0f);
    goal_position.push_back(0.0f);

    chain.jointMove(goal_position, 1.0f);
  }

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
#endif