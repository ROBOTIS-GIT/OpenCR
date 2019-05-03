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

#include <delta_libs.h>

bool start_demo_flag;
uint8_t motion_cnt[] = {0};
uint8_t sub_motion_cnt[] = {0};

const double hole[10][3] = {{ 0.0580, -0.0340, 0.023},  // Ball 1 
                            { 0.0000, -0.0690, 0.023},  // Ball 2
                            {-0.0563, -0.0360, 0.023},  // Ball 3
                            {-0.0570,  0.0315, 0.023},  // Ball 4
                            {-0.0000,  0.0660, 0.023},  // Ball 5
                            { 0.0580,  0.0335, 0.023},  // Ball 6
                            { 0.0000,  0.0173, 0.023},  // Ball 7
                            {-0.0150, -0.0087, 0.023},  // Ball 8
                            { 0.0150, -0.0087, 0.023},  // Ball 9
                            { 0.0000,  0.0000, 0.042}}; // Centre
                                  
/*****************************************************************************
** Functions used in runDemo()
*****************************************************************************/
// Move in Joint Space 
bool touchBall(Delta *delta, int num, double t, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: delta->makeTaskTrajectory("tool", math::vector3(hole[num-1][0], hole[num-1][1], hole[num-1][2]+0.015), t); break;
  case 1: delta->makeTaskTrajectory("tool", math::vector3(hole[num-1][0], hole[num-1][1], hole[num-1][2]), t); break;
  case 2: delta->makeTaskTrajectory("tool", math::vector3(hole[num-1][0], hole[num-1][1], hole[num-1][2]+0.015), t); return 1; 
  }
  return 0;
}

void suctionOnOff(bool onoff)
{
  if (onoff)
  {
    pinMode(4, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(12, OUTPUT);
    digitalWrite(8, HIGH);      //suction on
    digitalWrite(12, HIGH);     //suction on
  } 
  else 
  {
    pinMode(4, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(12, OUTPUT);
    digitalWrite(8, LOW);      //suction off
    digitalWrite(12, LOW);     //suction off
  }
}

bool moveBall(Delta *delta, int num1, int num2, double t, int index)
{
  switch(sub_motion_cnt[index])
  {
  case 0: delta->makeTaskTrajectory("tool", math::vector3(hole[num1-1][0], hole[num1-1][1], hole[num2-1][2]+0.015), t); break;
  case 1: delta->makeTaskTrajectory("tool", math::vector3(hole[num1-1][0], hole[num1-1][1], hole[num1-1][2]), t); suctionOnOff(true); break;
  case 2: delta->sleepTrajectory(1.0); break;
  case 3: delta->makeTaskTrajectory("tool", math::vector3(hole[num1-1][0], hole[num1-1][1], hole[num2-1][2]+0.015), t); break;
  case 4: delta->makeTaskTrajectory("tool", math::vector3(hole[num2-1][0], hole[num2-1][1], hole[num2-1][2]+0.015), t); break;
  case 5: delta->makeTaskTrajectory("tool", math::vector3(hole[num2-1][0], hole[num2-1][1], hole[num2-1][2]), t); break;
  case 6: delta->sleepTrajectory(1.0); suctionOnOff(false); break;
  case 7: delta->makeTaskTrajectory("tool", math::vector3(hole[num2-1][0], hole[num2-1][1], hole[num2-1][2]+0.015), t); return 1;
  }
  return 0;
}

// Draw an Object 
void drawObj(Delta *delta, STRING object, double radius, int num_revolution, double start_angular_position, double move_time)
{
  double draw_arg[3]; 
  draw_arg[0] = radius;                 // Radius (m)
  draw_arg[1] = num_revolution;         // Number of revolution
  draw_arg[2] = start_angular_position; // Starting angular position (rad)
  void *p_draw_arg = &draw_arg;

  delta->makeCustomTrajectory(object, "tool", p_draw_arg, move_time);
}

/*****************************************************************************
** Start or Stop Demo
*****************************************************************************/
void startDemo()
{
  // Start the demo
  start_demo_flag = true;
}

void stopDemo(Delta *delta)
{
  // Stop the demo
  start_demo_flag = false;

  // Move to the default pose.
  delta->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, 0.0), 2.0);

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
void runDemo(Delta *delta)
{
  if(digitalRead(BDPIN_PUSH_SW_1))
  {
    startDemo();
  }
  if(digitalRead(BDPIN_PUSH_SW_2))
  {
    stopDemo(delta);    
  }

  if (delta->getMovingState()) 
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
          if(touchBall(delta, 1, 0.07, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 1:
          if(touchBall(delta, 2, 0.07, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 2:
          if(touchBall(delta, 3, 0.07, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 3:
          if(touchBall(delta, 4, 0.07, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 4:
          if(touchBall(delta, 5, 0.07, 0))
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 5:
          if(touchBall(delta, 6, 0.07, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 6:
          if(touchBall(delta, 5, 0.07, 0))
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 7:
          if(touchBall(delta, 4, 0.07, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 8:
          if(touchBall(delta, 3, 0.07, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 9:
          if(touchBall(delta, 2, 0.07, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 10:
          if(touchBall(delta, 1, 0.07, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 11:
          if(moveBall(delta, 1, 9, 0.15, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 12:
          if(moveBall(delta, 3, 8, 0.15, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 13:
          if(moveBall(delta, 5, 7, 0.15, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 14:
          if(moveBall(delta, 6, 10, 0.15, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 15:
          if(moveBall(delta, 10, 6, 0.15, 0))
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 16:
          if(moveBall(delta, 7, 5, 0.15, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 17:
          if(moveBall(delta, 8, 3, 0.15, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 18:
          if(moveBall(delta, 9, 1, 0.15, 0)) 
          { sub_motion_cnt[0] = 0; motion_cnt[0] ++; }
          else 
            sub_motion_cnt[0] ++;
        break;
        case 19:
          delta->makeTaskTrajectory("tool", math::vector3(0,0,0), 1.0);
        break;        
      }
    }
  }
}

#endif // DEMO_H_
