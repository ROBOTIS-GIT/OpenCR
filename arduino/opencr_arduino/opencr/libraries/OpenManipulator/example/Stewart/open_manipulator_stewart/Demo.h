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

#include <Stewart.h>

uint8_t motion_cnt[] = {0};

//---------------------------------------------------------------------------------------------------- 
void initDemo(){}

//---------------------------------------------------------------------------------------------------- 
void runDemo(Stewart *stewart)
{
  if (stewart->isMoving())
    return;
  else
  {
    switch(motion_cnt[0])
    {
      case 0:
        stewart->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.025, 0.0, 0.0), 1.0);
        motion_cnt[0] ++; 
      break;
      case 1:
        stewart->taskTrajectoryMove("tool", RM_MATH::makeVector3(-0.025, 0.0, 0.0), 1.0/2);
        motion_cnt[0] ++; 
      break;
      case 2:
        stewart->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.025, 0.0, 0.0), 1.0/2);
        motion_cnt[0] ++; 
      break;
      case 3:
        stewart->taskTrajectoryMove("tool", RM_MATH::makeVector3(-0.025, 0.0, 0.0), 1.0/2);
        motion_cnt[0] ++; 
      break;
      case 4:
        stewart->taskTrajectoryMove("tool", RM_MATH::makeVector3(0.0, 0.025, 0.0), 1.0/2);
        motion_cnt[0] ++; 
      break;
      case 5:
        stewart->taskTrajectoryMove("tool", RM_MATH::makeVector3(0.0, -0.025, 0.0), 1.0/2);
        motion_cnt[0] ++; 
      break;
      case 6:
        stewart->taskTrajectoryMove("tool", RM_MATH::makeVector3(0.0,  0.025, 0.0), 1.0/2);
        motion_cnt[0] ++; 
      break;
      case 7:
        stewart->taskTrajectoryMove("tool", RM_MATH::makeVector3(0.0, -0.025, 0.0), 1.0/2);
        motion_cnt[0] ++; 
      break;
      case 8:
        stewart->taskTrajectoryMove("tool", RM_MATH::makeVector3(0.0, 0.0, 0.020), 1.0/2);
        motion_cnt[0] ++; 
      break;
      case 9:
        stewart->taskTrajectoryMove("tool", RM_MATH::makeVector3(0.0, 0.0, -0.0), 1.0/2);
        motion_cnt[0] ++; 
      break;
      case 10:
        stewart->taskTrajectoryMove("tool", RM_MATH::makeVector3(0.0, 0.0, 0.020), 1.0/2);
        motion_cnt[0] ++; 
      break;
      case 11:
        stewart->taskTrajectoryMove("tool", RM_MATH::makeVector3(0.0, 0.0, -0.0), 1.0/2);
        motion_cnt[0] ++; 
      break;
      case 12:
      {
        Pose goal_pose;
        goal_pose.position = RM_MATH::makeVector3(0.0, 0.0, 0.0);
        goal_pose.orientation = RM_MATH::getRotationX(PI/18.0);
        stewart->taskTrajectoryMove("tool", goal_pose, 1.0/2);
        motion_cnt[0] ++; 
      }
      break;
      case 13:
      {
        Pose goal_pose;
        goal_pose.position = RM_MATH::makeVector3(0.0, 0.0, 0.0);
        goal_pose.orientation = RM_MATH::getRotationX(-PI/18.0);
        stewart->taskTrajectoryMove("tool", goal_pose, 1.0/2);
        motion_cnt[0] ++; 
      }
      break;
      case 14:
      {
        Pose goal_pose;
        goal_pose.position = RM_MATH::makeVector3(0.0, 0.0, 0.0);
        goal_pose.orientation = RM_MATH::getRotationY(PI/18.0);
        stewart->taskTrajectoryMove("tool", goal_pose, 1.0/2);
        motion_cnt[0] ++; 
      }
      break;
      case 15:
      {
        Pose goal_pose;
        goal_pose.position = RM_MATH::makeVector3(0.0, 0.0, 0.0);
        goal_pose.orientation = RM_MATH::getRotationY(-PI/18.0);
        stewart->taskTrajectoryMove("tool", goal_pose, 1.0/2);
        motion_cnt[0] ++; 
      }
      break;
      case 16:
      {
        Pose goal_pose;
        goal_pose.position = RM_MATH::makeVector3(0.0, 0.0, 0.0);
        goal_pose.orientation = RM_MATH::getRotationZ(PI/6.0);
        stewart->taskTrajectoryMove("tool", goal_pose, 1.0/2);
        motion_cnt[0] ++; 
      }
      break;
      case 17:
      {
        Pose goal_pose;
        goal_pose.position = RM_MATH::makeVector3(0.0, 0.0, 0.0);
        goal_pose.orientation = RM_MATH::getRotationZ(-PI/6.0);
        stewart->taskTrajectoryMove("tool", goal_pose, 1.0/2);
        motion_cnt[0] ++; 
      }
      break;
      case 18:
      {
        Pose goal_pose;
        goal_pose.position = RM_MATH::makeVector3(0.0, 0.0, 0.0);
        goal_pose.orientation = RM_MATH::getRotationX(0.0);
        stewart->taskTrajectoryMove("tool", goal_pose, 1.0/2);
        motion_cnt[0] ++; 
      }
      break;
      case 19:
        stewart->TrajectoryWait(10.0);
        motion_cnt[0] = 0; 
      break;
    }
  }
}

#endif // DEMO_H_

