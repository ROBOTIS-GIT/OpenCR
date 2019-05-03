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

#include <open_manipulator_libs.h>

bool start_demo_flag;
bool erasing_flag;
uint8_t motion_cnt[] = {0};
uint8_t sub_motion_cnt[] = {0};

/*****************************************************************************
** Functions used in runDemo()
*****************************************************************************/
// Move in Joint Space 
void moveJS(OpenManipulator *open_manipulator, double j1, double j2, double j3, double j4, double t)
{
  static std::vector <double> target_angle;
  target_angle.clear();
  target_angle.push_back(j1);
  target_angle.push_back(j2);
  target_angle.push_back(j3);
  target_angle.push_back(j4);
  open_manipulator->makeJointTrajectory(target_angle,t);
}

/*****************************************************************************
** Start or Stop Demo
*****************************************************************************/
void startDemo()
{
  // Start the demo
  start_demo_flag = true;
}

void stopDemo(OpenManipulator *open_manipulator)
{
  // Stop the demo
  start_demo_flag = false;

  // Move to the default pose.
  moveJS(open_manipulator, 0.0, 0.0, 0.0, 0.0, 1.0); 
  open_manipulator->makeToolTrajectory("tool", 0.0);

  // Reset the count variables
  motion_cnt[0] = 0;
  sub_motion_cnt[0] = 0;
  erasing_flag = false;
}

/*****************************************************************************
** Initialize Demo
*****************************************************************************/
void initDemo()
{
  start_demo_flag = false;
  motion_cnt[0] = 0;
  sub_motion_cnt[0] = 0;
  erasing_flag = false;

  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);
}

/*****************************************************************************
** Run Demo
*****************************************************************************/
void runDemo(OpenManipulator *open_manipulator)
{
  if(digitalRead(BDPIN_PUSH_SW_1))
  {
    startDemo();
  }
  if(digitalRead(BDPIN_PUSH_SW_2))
  {
    stopDemo(open_manipulator);
  }

  if (open_manipulator->getMovingState())
  {
    return;
  }
  else 
  {
    if (start_demo_flag)
    // if (1)
    {
      // Draw Objects
      if (!erasing_flag)
      {
        switch(motion_cnt[0])
        {

          case 0:
            moveJS(open_manipulator, 0.0, -1.0, 0.2, 0.8, 2.0); 
            motion_cnt[0] ++; 
          break;
          case 1:
            moveJS(open_manipulator, -0.5, 0.0, 1.0, -1.0, 2.0); 
            motion_cnt[0] ++; 
          break;
          case 2:
            moveJS(open_manipulator, -0.5, 0.1, 0.75, -0.85, 2.0); 
            motion_cnt[0] ++; 
          break;
          case 3:
            open_manipulator->sleepTrajectory(2.0);
            motion_cnt[0] ++; 
          break;
          case 4:
            moveJS(open_manipulator, -0.5, -0.05, 1.05, -1.0, 2.0); 
            motion_cnt[0] = 0; 
          break;
        }
      }
    }
  }
}

#endif // DEMO_H_

