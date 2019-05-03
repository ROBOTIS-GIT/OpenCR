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

#include <linear_libs.h>

bool start_demo_flag;
uint8_t motion_cnt[] = {0};
uint8_t sub_motion_cnt[] = {0};

/*****************************************************************************
** Start or Stop Demo
*****************************************************************************/
void startDemo()
{
  // Start the demo
  start_demo_flag = true;
}

void stopDemo(Linear *linear)
{
  // Stop the demo
  start_demo_flag = false;

  // Move to the default pose.
  std::vector<double> target_angle;
  target_angle.push_back(0.0);
  target_angle.push_back(0.0);
  target_angle.push_back(0.0);
  linear->makeJointTrajectory(target_angle, 1);
  linear->makeToolTrajectory("tool", -0.007);

  // Reset the count variables
  motion_cnt[0] = 0;
  sub_motion_cnt[0] = 0;
}

/*****************************************************************************
** Initialize Demo
*****************************************************************************/
void initDemo()
{
  start_demo_flag = false;
  motion_cnt[0] = 0;
  sub_motion_cnt[0] = 0;  
}

/*****************************************************************************
** Run Demo
*****************************************************************************/
void runDemo(Linear *linear)
{
  if(digitalRead(BDPIN_PUSH_SW_1))
  {
    startDemo();
  }
  if(digitalRead(BDPIN_PUSH_SW_2))
  {
    stopDemo(linear);    
  }

  if (linear->getMovingState())
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
          linear->makeToolTrajectory("tool", -0.49);
          linear->sleepTrajectory(2.0);
          motion_cnt[0] ++;
        break;
        case 1:
          {
          double joint_angle[2];
          joint_angle[0] = linear->getJointValue("joint1").position;
          joint_angle[1] = linear->getJointValue("joint2").position;
          std::vector<double> target_angle;
          target_angle.push_back(joint_angle[0]);
          target_angle.push_back(joint_angle[1]);
          target_angle.push_back(0.0);
          linear->makeJointTrajectory(target_angle, 1.0);
          motion_cnt[0] ++;
          }
        break;
        case 2:
          linear->makeToolTrajectory("tool", 0.49);        
          linear->sleepTrajectory(2.0); 
          motion_cnt[0] ++;
        break;
        case 3:
          {
          double joint_angle[2];
          joint_angle[0] = linear->getJointValue("joint1").position;
          joint_angle[1] = linear->getJointValue("joint2").position;
          std::vector<double> target_angle;
          target_angle.push_back(joint_angle[0]);
          target_angle.push_back(joint_angle[1]);
          target_angle.push_back(-2*PI);
          linear->makeJointTrajectory(target_angle, 1.0);
          motion_cnt[0] ++;
          }
        break;
        case 4:
          {
          std::vector<double> target_angle;
          target_angle.push_back(-4.899);
          target_angle.push_back(-4.5);
          target_angle.push_back(-2*PI);
          linear->makeJointTrajectory(target_angle, 0.3);
          motion_cnt[0] ++;
          }
        break;
        case 5:
          linear->sleepTrajectory(1.0); 
          motion_cnt[0] ++;
        break;
        case 6:
          linear->makeToolTrajectory("tool", -0.49);        
          linear->sleepTrajectory(2.0); 
          motion_cnt[0] ++;
        break;
        case 7:
          linear->makeToolTrajectory("tool", 0.49);        
          linear->sleepTrajectory(2.0); 
          motion_cnt[0] ++;
        break;
        case 8:
          {
          std::vector<double> target_angle;
          target_angle.push_back(0.0);
          target_angle.push_back(0.0);
          target_angle.push_back(-2*PI);
          linear->makeJointTrajectory(target_angle, 0.3);
          motion_cnt[0] ++;
          }
        break;
      }
    }
  }
}

#endif // DEMO_H_
