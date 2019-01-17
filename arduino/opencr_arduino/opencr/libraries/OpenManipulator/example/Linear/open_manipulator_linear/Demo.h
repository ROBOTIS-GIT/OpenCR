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

#include <Linear.h>

uint8_t motion_cnt[] = {0};
uint8_t sub_motion_cnt[] = {0};

//---------------------------------------------------------------------------------------------------- 
void initDemo(){}

//---------------------------------------------------------------------------------------------------- 
void runDemo(Linear *linear)
{
  if (linear->isMoving()) 
  {
    return;
  }
  else 
  {  
    switch(motion_cnt[0])
    {
      case 0:
        linear->toolMove("tool", 0.007);
        linear->TrajectoryWait(2.0); 
        motion_cnt[0] ++;
        break;
      case 1:
        {
        double joint_angle[2];
        joint_angle[0] = linear->getJointValue("joint1").value;
        joint_angle[1] = linear->getJointValue("joint2").value;
        std::vector<double> goal_position = {joint_angle[0], joint_angle[1], 0};
        linear->jointTrajectoryMove(goal_position, 1.0);
        motion_cnt[0] ++;
        }
        break;
      case 2:
        linear->toolMove("tool", -0.007);        
        linear->TrajectoryWait(2.0); 
        motion_cnt[0] ++;
        break;
      case 3:
        {
        double joint_angle[2];
        joint_angle[0] = linear->getJointValue("joint1").value;
        joint_angle[1] = linear->getJointValue("joint2").value;
        std::vector<double> goal_position = {joint_angle[0], joint_angle[1], -2*PI};
        linear->jointTrajectoryMove(goal_position, 1.0);
        motion_cnt[0] ++;
        }
        break;
      case 4:
        {
        std::vector<double> goal_position = {-4.899, -4.5, -2*PI};
        linear->jointTrajectoryMove(goal_position, 0.3);
        motion_cnt[0] ++;
        }
        break;
      case 5:
        linear->toolMove("tool", 0.007);        
        linear->TrajectoryWait(2.0); 
        motion_cnt[0] ++;
        break;
      case 6:
        linear->toolMove("tool", -0.007);        
        linear->TrajectoryWait(2.0); 
        motion_cnt[0] ++;
        break;
      case 7:
        {
        std::vector<double> goal_position = {0.0, 0.0, -2*PI};
        linear->jointTrajectoryMove(goal_position, 0.3);
        linear->setRunDemoFlag(false);
        motion_cnt[0] = 0;
        }
        break;
    }
  }
}
#endif // DEMO_H_
