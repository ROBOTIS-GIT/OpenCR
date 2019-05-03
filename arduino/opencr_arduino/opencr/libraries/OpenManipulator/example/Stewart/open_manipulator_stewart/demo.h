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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef DEMO_H_
#define DEMO_H_

#include <stewart_libs.h>

bool start_demo_flag;
uint8_t motion_cnt[] = {0};

/*****************************************************************************
** Start or Stop Demo
*****************************************************************************/
void startDemo()
{
  // Start the demo
  start_demo_flag = true;
}

void stopDemo(Stewart *stewart)
{
  // Stop the demo
  start_demo_flag = false;

  // Move to the default pose.
  stewart->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, 0.0), 2.0);

  // Reset the count variables
  motion_cnt[0] = 0;
}

/*****************************************************************************
** Initialize Demo
*****************************************************************************/
void initDemo()
{
  start_demo_flag = false;
  motion_cnt[0] = 0;
}

/*****************************************************************************
** Run Demo
*****************************************************************************/
void runDemo(Stewart *stewart)
{
  if(digitalRead(BDPIN_PUSH_SW_1))
  {
    startDemo();
  }
  if(digitalRead(BDPIN_PUSH_SW_2))
  {
    stopDemo(stewart);    
  }

  if (stewart->getMovingState()) 
  {
    return;
  }
  else 
  {
    if (start_demo_flag)
    {
      switch(motion_cnt[0])
      {
        case 0:
          stewart->makeTaskTrajectory("tool", math::vector3( 0.025, 0.0, 0.0), 1.0);
          motion_cnt[0] ++; 
        break;
        case 1:
          stewart->makeTaskTrajectory("tool", math::vector3(-0.025, 0.0, 0.0), 1.0/2);
          motion_cnt[0] ++; 
        break;
        case 2:
          stewart->makeTaskTrajectory("tool", math::vector3( 0.025, 0.0, 0.0), 1.0/2);
          motion_cnt[0] ++; 
        break;
        case 3:
          stewart->makeTaskTrajectory("tool", math::vector3(-0.025, 0.0, 0.0), 1.0/2);
          motion_cnt[0] ++; 
        break;
        case 4:
          stewart->makeTaskTrajectory("tool", math::vector3(0.0, 0.025, 0.0), 1.0/2);
          motion_cnt[0] ++; 
        break;
        case 5:
          stewart->makeTaskTrajectory("tool", math::vector3(0.0, -0.025, 0.0), 1.0/2);
          motion_cnt[0] ++; 
        break;
        case 6:
          stewart->makeTaskTrajectory("tool", math::vector3(0.0,  0.025, 0.0), 1.0/2);
          motion_cnt[0] ++; 
        break;
        case 7:
          stewart->makeTaskTrajectory("tool", math::vector3(0.0, -0.025, 0.0), 1.0/2);
          motion_cnt[0] ++; 
        break;
        case 8:
          stewart->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, 0.020), 1.0/2);
          motion_cnt[0] ++; 
        break;
        case 9:
          stewart->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, -0.0), 1.0/2);
          motion_cnt[0] ++; 
        break;
        case 10:
          stewart->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, 0.020), 1.0/2);
          motion_cnt[0] ++; 
        break;
        case 11:
          stewart->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, -0.0), 1.0/2);
          motion_cnt[0] ++; 
        break;
        case 12:
        {
          Pose goal_pose;
          goal_pose.kinematic.position = math::vector3(0.0, 0.0, 0.0);
          goal_pose.kinematic.orientation = math::convertRollAngleToRotationMatrix(PI/18.0);
          stewart->makeTaskTrajectory("tool", goal_pose.kinematic, 1.0);
          motion_cnt[0] ++; 
        }
        break;
        case 13:
        {
          Pose goal_pose;
          goal_pose.kinematic.position = math::vector3(0.0, 0.0, 0.0);
          goal_pose.kinematic.orientation = math::convertRollAngleToRotationMatrix(-PI/18.0);
          stewart->makeTaskTrajectory("tool", goal_pose.kinematic, 1.0/2);
          motion_cnt[0] ++; 
        }
        break;
        case 14:
        {
          Pose goal_pose;
          goal_pose.kinematic.position = math::vector3(0.0, 0.0, 0.0);
          goal_pose.kinematic.orientation = math::convertPitchAngleToRotationMatrix(PI/18.0);
          stewart->makeTaskTrajectory("tool", goal_pose.kinematic, 1.0/2);
          motion_cnt[0] ++; 
        }
        break;
        case 15:
        {
          Pose goal_pose;
          goal_pose.kinematic.position = math::vector3(0.0, 0.0, 0.0);
          goal_pose.kinematic.orientation = math::convertPitchAngleToRotationMatrix(-PI/18.0);
          stewart->makeTaskTrajectory("tool", goal_pose.kinematic, 1.0/2);
          motion_cnt[0] ++; 
        }
        break;
        case 16:
        {
          Pose goal_pose;
          goal_pose.kinematic.position = math::vector3(0.0, 0.0, 0.0);
          goal_pose.kinematic.orientation = math::convertYawAngleToRotationMatrix(PI/6.0);
          stewart->makeTaskTrajectory("tool", goal_pose.kinematic, 1.0/2);
          motion_cnt[0] ++; 
        }
        break;
        case 17:
        {
          Pose goal_pose;
          goal_pose.kinematic.position = math::vector3(0.0, 0.0, 0.0);
          goal_pose.kinematic.orientation = math::convertYawAngleToRotationMatrix(-PI/6.0);
          stewart->makeTaskTrajectory("tool", goal_pose.kinematic, 1.0/2);
          motion_cnt[0] ++; 
        }
        break;
        case 18:
        {
          Pose goal_pose;
          goal_pose.kinematic.position = math::vector3(0.0, 0.0, 0.0);
          goal_pose.kinematic.orientation = math::convertRollAngleToRotationMatrix(0.0);
          stewart->makeTaskTrajectory("tool", goal_pose.kinematic, 1.0/2);
          motion_cnt[0] ++; 
        }
        break;
        case 19:
          // stewart->sleepTrajectory(10.0);
          // motion_cnt[0] = 0; 
        break;
      }
    }
  }
}

#endif // DEMO_H_

