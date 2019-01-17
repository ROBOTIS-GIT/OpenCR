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

#include <Planar.h>

uint8_t motion_cnt[] = {0};
uint8_t sub_motion_cnt[] = {0};

/* Draw an Object */
void drawObj(Planar* planar, STRING object, double radius, int num_revolution, double start_angular_position, double move_time)
{
  double draw_arg[3]; 
  draw_arg[0] = radius;                 // Radius (m)
  draw_arg[1] = num_revolution;         // Number of revolution
  draw_arg[2] = start_angular_position; // Starting angular position (rad)
  void* p_draw_arg = &draw_arg;
  planar->drawingTrajectoryMove(object, "tool", draw_arg, move_time);
}

//---------------------------------------------------------------------------------------------------- 
void initDemo(){}

//---------------------------------------------------------------------------------------------------- 
void runDemo(Planar *planar)
{
  if (planar->isMoving()) 
    return;
  else 
  {
    switch(motion_cnt[0])
    {
      case 0:
        // planar->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.045, 0.0, 0.0), 1);
        motion_cnt[0] ++; 
      break;
      case 1:
        planar->TrajectoryWait(0.05);
        motion_cnt[0] ++; 
      break;
      case 2:
        // planar->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.0, 0.045, 0.0), 1);
        motion_cnt[0] ++; 
      break;
      case 3:
        planar->TrajectoryWait(0.05);
        motion_cnt[0] ++; 
      break;
      case 4:
        // planar->taskTrajectoryMove("tool", RM_MATH::makeVector3(-0.045, 0.0, 0.0), 1);
        motion_cnt[0] ++; 
      break;
      case 5:
        planar->TrajectoryWait(0.05);
        motion_cnt[0] ++; 
      break;
      case 6:
        // planar->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.0,-0.045, 0.0), 1);
        motion_cnt[0] ++; 
      break;
      case 7:
        planar->TrajectoryWait(0.05);
        motion_cnt[0] ++; 
      break;
      case 8:
        // planar->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.045, 0.0, 0.0), 1);
        motion_cnt[0] ++; 
      break;
      case 9:
        planar->TrajectoryWait(0.5);
        motion_cnt[0] ++; 
      break;
      case 10:
        planar->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.045, 0.0, 0.0), 0.08);
        motion_cnt[0] ++; 
      break;
      case 11:
        planar->TrajectoryWait(0.05);
        motion_cnt[0] ++; 
      break;
      case 12:
        planar->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.0,0.045, 0.0), 0.08);
        motion_cnt[0] ++; 
      break;
      case 13:
        planar->TrajectoryWait(0.05);
        motion_cnt[0] ++; 
      break;
      case 14:
        planar->taskTrajectoryMove("tool", RM_MATH::makeVector3(-0.045, 0.0, 0.0), 0.08);
        motion_cnt[0] ++; 
      break;
      case 15:
        planar->TrajectoryWait(0.05);
        motion_cnt[0] ++; 
      break;
      case 16:
        planar->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.0,-0.045, 0.0), 0.08);
        motion_cnt[0] ++; 
      break;
      case 17:
        planar->TrajectoryWait(0.05);
        motion_cnt[0] ++; 
      break;
      case 18:
        planar->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.045, 0.0, 0.0), 0.08);
        motion_cnt[0] ++; 
      break;
      case 19:
        // planar->TrajectoryWait(0.05);
        motion_cnt[0] ++; 
      break;
      case 20:
        drawObj(planar, DRAWING_CIRCLE, 0.045, 1, 0.0, 0.8); // drawObj(manipulator, blahblah)
        motion_cnt[0] ++; 
      break;
      case 21:
        planar->TrajectoryWait(0.05);
        motion_cnt[0] ++; 
      break;
      case 22:
        // planar->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.0, 0.0, 0.0), 0.08);
        motion_cnt[0] ++; 
      break;
      case 23:
        // planar->TrajectoryWait(0.3);
        motion_cnt[0] ++; 
      break;
      case 24:
        planar->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.04, 0.02, 0.0), 0.08);
        motion_cnt[0] ++; 
      break;
      case 25:
        planar->TrajectoryWait(0.3);
        motion_cnt[0] ++; 
      break;
      case 26:
        planar->taskTrajectoryMove("tool", RM_MATH::makeVector3(-0.045, 0.0, 0.0), 0.08);
        motion_cnt[0] ++; 
      break;
      case 27:
        planar->TrajectoryWait(0.3);
        motion_cnt[0] ++; 
      break;
      case 28:
        planar->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.04, -0.02, 0.0), 0.08);
        motion_cnt[0] ++; 
      break;
      case 29:
        planar->TrajectoryWait(0.3);
        motion_cnt[0] ++; 
      break;    
      case 30:
        planar->taskTrajectoryMove("tool", RM_MATH::makeVector3(-0.02, 0.04, 0.0), 0.08);
        motion_cnt[0] ++; 
      break;
      case 31:
        planar->TrajectoryWait(0.3);
        motion_cnt[0] ++; 
      break;    
      case 32:
        planar->taskTrajectoryMove("tool", RM_MATH::makeVector3(-0.02,-0.04, 0.0), 0.08);
        motion_cnt[0] ++; 
      break;
      case 33:
        planar->TrajectoryWait(0.3);
        motion_cnt[0] ++; 
      break;    
      case 34:
        planar->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.04, 0.02, 0.0), 0.08);
        motion_cnt[0] ++; 
      break;
      case 35:
        planar->TrajectoryWait(1);
        motion_cnt[0] ++; 
      break;
      case 36:
        planar->taskTrajectoryMove("tool", RM_MATH::makeVector3( 0.0, 0.0, 0.0), 0.08);
        motion_cnt[0] ++; 
      break;
    }
  }
}

#endif // DEMO_H_
