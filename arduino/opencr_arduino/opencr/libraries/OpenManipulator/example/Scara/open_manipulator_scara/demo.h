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

#include <scara_libs.h>

bool start_demo_flag;
bool erasing_flag;
uint8_t motion_cnt[] = {0};
uint8_t sub_motion_cnt[] = {0};

/*****************************************************************************
** Functions used in runDemo()
*****************************************************************************/
// Move in Joint Space 
void moveJS(Scara* scara, double j1, double j2, double j3, double t) 
{
  static std::vector <double> target_angle;
  target_angle.clear();
  target_angle.push_back(j1);
  target_angle.push_back(j2);
  target_angle.push_back(j3);
  scara->makeJointTrajectory(target_angle,t);     
}

// Draw an Object 
void drawObj(Scara* scara, STRING object, double radius, int num_revolution, double start_angular_position, double move_time)
{
  double draw_arg[3]; 
  draw_arg[0] = radius;                 // Radius (m)
  draw_arg[1] = num_revolution;         // Number of revolution
  draw_arg[2] = start_angular_position; // Starting angular position (rad)
  void* p_draw_arg = &draw_arg;

  scara->makeCustomTrajectory(object, "tool", p_draw_arg, move_time);
}

// Erase an Object
bool eraseObj(Scara *scara, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: scara->makeToolTrajectory("tool", -0.5); break;
  case 1: moveJS(scara, -2.17, 0.82, 2.05, 3.0); break;
  case 2: scara->sleepTrajectory(2.0); scara->makeToolTrajectory("tool", -0.0); break;
  case 3: scara->sleepTrajectory(2.0); scara->makeToolTrajectory("tool", -0.5); return 1; 
  }
  return 0;
}

bool drawCircle(Scara *scara, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: scara->makeToolTrajectory("tool", -0.5); break;
  case 1: moveJS(scara, -1.05, 0.9, 0.9, 2.0); break;
  case 2: scara->makeToolTrajectory("tool", -0.0); break;
  case 3: drawObj(scara, CUSTOM_TRAJECTORY_CIRCLE, 0.030, 1, 0.0, 10.0); return 1;
  }
  return 0;
}

bool drawCircle2(Scara *scara, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: scara->makeToolTrajectory("tool", -0.5); break;
  case 1: moveJS(scara, -1.45, 1.2, 1.2, 2.0); break;
  case 2: scara->makeToolTrajectory("tool", -0.0); break;
  case 3: drawObj(scara, CUSTOM_TRAJECTORY_CIRCLE, 0.020, 1, 0, 10.0); break; 
  case 4: drawObj(scara, CUSTOM_TRAJECTORY_CIRCLE, 0.020, 1, PI/3, 10.0); break; 
  case 5: drawObj(scara, CUSTOM_TRAJECTORY_CIRCLE, 0.020, 1, PI*2/3, 10.0); break; 
  case 6: drawObj(scara, CUSTOM_TRAJECTORY_CIRCLE, 0.020, 1, PI, 10.0); break; 
  case 7: drawObj(scara, CUSTOM_TRAJECTORY_CIRCLE, 0.020, 1, PI*4/3, 10.0); break; 
  case 8: drawObj(scara, CUSTOM_TRAJECTORY_CIRCLE, 0.020, 1, PI*5/3, 10.0); return 1;
  }
  return 0;
}

bool drawRhombus(Scara *scara, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: scara->makeToolTrajectory("tool", -0.5); break;
  case 1: moveJS(scara, -1.8, 1.43, 1.43, 2.0); break;
  case 2: scara->makeToolTrajectory("tool", -0.0); break;
  case 3: drawObj(scara, CUSTOM_TRAJECTORY_RHOMBUS, 0.035, 1, PI, 10.0); return 1;
  }
  return 0;
}

bool drawRhombus2(Scara *scara, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: scara->makeToolTrajectory("tool", -0.5); break;
  case 1: moveJS(scara, -1.80, 1.43, 1.43, 2.0); break;
  case 2: scara->makeToolTrajectory("tool", -0.0); break;
  case 3: drawObj(scara, CUSTOM_TRAJECTORY_RHOMBUS, 0.020, 1, PI, 10.0); break;
  case 4: drawObj(scara, CUSTOM_TRAJECTORY_RHOMBUS, 0.027, 1, PI, 10.0); break;
  case 5: drawObj(scara, CUSTOM_TRAJECTORY_RHOMBUS, 0.034, 1, PI, 10.0); break;
  case 6: drawObj(scara, CUSTOM_TRAJECTORY_RHOMBUS, 0.041, 1, PI, 10.0); return 1;
  }
  return 0;
}

bool drawHeart(Scara *scara, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: scara->makeToolTrajectory("tool", -0.5); break;
  case 1: moveJS(scara, -1.6, 1.3, 1.3, 2.0); break;
  case 2: scara->makeToolTrajectory("tool", -0.0); break;
  case 3: drawObj(scara, CUSTOM_TRAJECTORY_HEART, 0.045, 1, PI, 10.0); return 1;
  }
  return 0;
}

/*****************************************************************************
** Start or Stop Demo
*****************************************************************************/
void startDemo()
{
  // Start the demo
  start_demo_flag = true;
}

void stopDemo(Scara *scara)
{
  // Stop the demo
  start_demo_flag = false;

  // Move to the default pose.
  moveJS(scara, 0.0, 0.0, 0.0, 2.0); 
  scara->makeToolTrajectory("tool", -0.5);

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
void runDemo(Scara *scara)
{
  if(digitalRead(BDPIN_PUSH_SW_1))
  {
    startDemo();
  }
  if(digitalRead(BDPIN_PUSH_SW_2))
  {
    stopDemo(scara);
  }

  if (scara->getMovingState())
  {
    return;
  }
  else 
  {
    if (start_demo_flag)
    {
      // Draw Objects
      if (!erasing_flag)
      {
        switch(motion_cnt[0])
        {
          case 0:
            if(drawCircle(scara, 0))
            { sub_motion_cnt[0] = 0; motion_cnt[0] ++; erasing_flag = true; }
            else
              sub_motion_cnt[0] ++;
          break;
          case 1:
            if(drawCircle2(scara, 0))
            { sub_motion_cnt[0] = 0; motion_cnt[0] ++; erasing_flag = true; }
            else 
              sub_motion_cnt[0] ++;
          break;
          case 2:
            if(drawRhombus(scara, 0))
            { sub_motion_cnt[0] = 0; motion_cnt[0] ++; erasing_flag = true; }
            else 
              sub_motion_cnt[0] ++;
          break;
          case 3:
            if(drawRhombus2(scara, 0))
            { sub_motion_cnt[0] = 0; motion_cnt[0] ++; erasing_flag = true; }
            else
              sub_motion_cnt[0] ++;
          break;
          case 4:
            if(drawHeart(scara, 0))
            { sub_motion_cnt[0] = 0; motion_cnt[0] = 0; erasing_flag = true; }
            else 
              sub_motion_cnt[0] ++;
          break;
        }
      }

      // Erase Objects
      else
      {
        if(eraseObj(scara, 0))
        { sub_motion_cnt[0] = 0; erasing_flag = false; }
        else 
          sub_motion_cnt[0] ++;
      }
    }
  }
}

#endif // DEMO_H_

