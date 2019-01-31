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

#ifndef OPEN_MANIPULATOR_PEN_MOTION_H_
#define OPEN_MANIPULATOR_PEN_MOTION_H_

#include <RobotisManipulator.h>
#include "open_manipulator_pen.h"

#define BDPIN_PUSH_SW_1         34
#define BDPIN_PUSH_SW_2         35

#define PEN_DEPTH  0.0436

bool demo_motion_state = false;
int  demo_motion_type = 0;
char demo_motion_cnt = 0;

void demo_motion_robotis_mm(OpenManipulatorPen *open_manipulator, int cnt)
{
  std::vector<double> goal_joint_position;
  char draw_alphabet_arg[1];
  void* p_draw_alphabet_arg = &draw_alphabet_arg;
  Eigen::Vector3d goal_pose;

  Eigen::Vector3d robotis_start_position(0.147, -0.07, PEN_DEPTH);
  Eigen::Vector3d mm_start_position(0.280, -0.06, PEN_DEPTH);
  Eigen::Vector3d ict_start_position(0.220, -0.06, PEN_DEPTH);
  
  switch(cnt) 
  {
    case 0: // home
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back(-1.05);
      goal_joint_position.push_back( 0.35);
      goal_joint_position.push_back( 0.69);
      open_manipulator->makeJointTrajectory(goal_joint_position, 1.0); 
    break;
    case 1: // init
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back( 0.00);
      open_manipulator->makeJointTrajectory(goal_joint_position, 1.0); 
    break;

    case 2: // R start pose 0, 0
      goal_pose = robotis_start_position;
      goal_pose(2) += 0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.0);
    break;
    case 3: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 4: // drawing R
      draw_alphabet_arg[0] = 'R'; // drawing alphabet
      draw_alphabet_arg[1] = 20; // drawing scale 2cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 5: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;

    case 6: // O start pose 0, 1
      goal_pose = robotis_start_position;
      goal_pose(0) -=0.010;
      goal_pose(1) +=0.02; 
      goal_pose(2) +=0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.0);
    break;
    case 7: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 8: // drawing O
      draw_alphabet_arg[0] = 'O'; // drawing alphabet
      draw_alphabet_arg[1] = 20; // drawing scale 2cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 9: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;

    case 10: // B start pose 0, 0
      goal_pose = robotis_start_position;
      goal_pose(0) -=0.00;
      goal_pose(1) +=0.04; 
      goal_pose(2) +=0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.0);
    break;
    case 11: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 12: // drawing O
      draw_alphabet_arg[0] = 'B'; // drawing alphabet
      draw_alphabet_arg[1] = 20; // drawing scale 2cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 13: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;

    case 14: // O start pose 0, 1
      goal_pose = robotis_start_position;
      goal_pose(0) -=0.01;
      goal_pose(1) +=0.06; 
      goal_pose(2) +=0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.0);
    break;
    case 15: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 16: // drawing O
      draw_alphabet_arg[0] = 'O'; // drawing alphabet
      draw_alphabet_arg[1] = 20; // drawing scale 2cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 17: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;

    case 18: // T start pose 1, 0
      goal_pose = robotis_start_position;
      goal_pose(0) -=0.0;
      goal_pose(1) +=0.09 - 0.001; 
      goal_pose(2) +=0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.0);
    break;
    case 19: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 20: // drawing O
      draw_alphabet_arg[0] = 'T'; // drawing alphabet
      draw_alphabet_arg[1] = 20; // drawing scale 2cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 21: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;

    case 22: // I start pose 0, 0
      goal_pose = robotis_start_position;
      goal_pose(0) -=0.0;
      goal_pose(1) +=0.10 - 0.001;  
      goal_pose(2) +=0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.0);
    break;
    case 23: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 24: // drawing O
      draw_alphabet_arg[0] = 'I'; // drawing alphabet
      draw_alphabet_arg[1] = 20; // drawing scale 2cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 25: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;
    
    case 26: // S start pose 0, 0
      goal_pose = robotis_start_position;
      goal_pose(0) -=0.0;
      goal_pose(1) +=0.12 - 0.001; 
      goal_pose(2) +=0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.0);
    break;
    case 27: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 28: // drawing O
      draw_alphabet_arg[0] = 'S'; // drawing alphabet
      draw_alphabet_arg[1] = 20; // drawing scale 2cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 29: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;

    case 30: // mobile manipulator start pose 0, 0
      goal_pose = mm_start_position;
      goal_pose(2) += 0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.5);
    break;
    case 31: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 32: // drawing mm
      draw_alphabet_arg[0] = '!'; // drawing alphabet
      draw_alphabet_arg[1] = 120; // drawing scale 12cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg,10.0);
    break;
    case 33: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.020), 0.5);
    break;

    case 34: // I start pose 0, 0
      goal_pose = ict_start_position;
      goal_pose(2) += 0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.5);
    break;
    case 35: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 36: // drawing mm
      draw_alphabet_arg[0] = 'I'; // drawing alphabet
      draw_alphabet_arg[1] = 15; // drawing scale 1.5cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 37: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;

    case 38: // C start pose 0, 1
      goal_pose = ict_start_position;
      goal_pose(0) -= 0.0075;
      goal_pose(1) += 0.015; 
      goal_pose(2) += 0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.5);
    break;
    case 39: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 40: // drawing mm
      draw_alphabet_arg[0] = 'C'; // drawing alphabet
      draw_alphabet_arg[1] = 15; // drawing scale 1.5cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 41: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;

    case 42: // T start pose 0, 0
      goal_pose = ict_start_position;
      goal_pose(0) -= 0.0;
      goal_pose(1) += 0.0375; 
      goal_pose(2) += 0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.5);
    break;
    case 43: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 44: // drawing mm
      draw_alphabet_arg[0] = 'T'; // drawing alphabet
      draw_alphabet_arg[1] = 15; // drawing scale 1.5cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 45: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;
    
    case 46: // home
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back(-1.05);
      goal_joint_position.push_back( 0.35);
      goal_joint_position.push_back( 0.69);
      open_manipulator->makeJointTrajectory(goal_joint_position, 1.5); 
    break;
  }
}

void demo_motion_mm(OpenManipulatorPen *open_manipulator, int cnt)
{
  std::vector<double> goal_joint_position;
  char draw_alphabet_arg[1];
  void* p_draw_alphabet_arg = &draw_alphabet_arg;
  Eigen::Vector3d goal_pose;
  
  Eigen::Vector3d mm_start_position(0.260, -0.075, PEN_DEPTH);
  
  switch(cnt) 
  {
    case 0: // home
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back(-1.05);
      goal_joint_position.push_back( 0.35);
      goal_joint_position.push_back( 0.69);
      open_manipulator->makeJointTrajectory(goal_joint_position, 1.0); 
    break;
    case 1: // init
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back( 0.00);
      open_manipulator->makeJointTrajectory(goal_joint_position, 1.0); 
    break;

    case 2: // mobile manipulator start pose 0, 0
      goal_pose = mm_start_position;
      goal_pose(2) += 0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.5);
    break;
    case 3: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 4: // drawing mm
      draw_alphabet_arg[0] = '!'; // drawing alphabet
      draw_alphabet_arg[1] = 120; // drawing scale 12cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg,25.0);
    break;
    case 5: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.020), 0.5);
    break;

    case 6: // home
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back(-1.05);
      goal_joint_position.push_back( 0.35);
      goal_joint_position.push_back( 0.69);
      open_manipulator->makeJointTrajectory(goal_joint_position, 1.5); 
    break;
  }
}

void demo_motion_robotis(OpenManipulatorPen *open_manipulator, int cnt)
{
  std::vector<double> goal_joint_position;
  char draw_alphabet_arg[1];
  void* p_draw_alphabet_arg = &draw_alphabet_arg;
  Eigen::Vector3d goal_pose;

  Eigen::Vector3d robotis_start_position(0.190, -0.105, PEN_DEPTH);
  
  switch(cnt) 
  {
    case 0: // home
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back(-1.05);
      goal_joint_position.push_back( 0.35);
      goal_joint_position.push_back( 0.69);
      open_manipulator->makeJointTrajectory(goal_joint_position, 1.0); 
    break;
    case 1: // init
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back( 0.00);
      open_manipulator->makeJointTrajectory(goal_joint_position, 1.0); 
    break;

    case 2: // R start pose 0, 0
      goal_pose = robotis_start_position;
      goal_pose(2) += 0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.0);
    break;
    case 3: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 4: // drawing R
      draw_alphabet_arg[0] = 'R'; // drawing alphabet
      draw_alphabet_arg[1] = 30; // drawing scale 3cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 5: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;

    case 6: // O start pose 0, 1
      goal_pose = robotis_start_position;
      goal_pose(0) -=0.015;
      goal_pose(1) +=0.03; 
      goal_pose(2) +=0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.0);
    break;
    case 7: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 8: // drawing O
      draw_alphabet_arg[0] = 'O'; // drawing alphabet
      draw_alphabet_arg[1] = 30; // drawing scale 3cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 9: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;

    case 10: // B start pose 0, 0
      goal_pose = robotis_start_position;
      goal_pose(0) -=0.00;
      goal_pose(1) +=0.06; 
      goal_pose(2) +=0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.0);
    break;
    case 11: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 12: // drawing O
      draw_alphabet_arg[0] = 'B'; // drawing alphabet
      draw_alphabet_arg[1] = 30; // drawing scale 3cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 13: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;

    case 14: // O start pose 0, 1
      goal_pose = robotis_start_position;
      goal_pose(0) -=0.015;
      goal_pose(1) +=0.09; 
      goal_pose(2) +=0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.0);
    break;
    case 15: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 16: // drawing O
      draw_alphabet_arg[0] = 'O'; // drawing alphabet
      draw_alphabet_arg[1] = 30; // drawing scale 3cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 17: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;

    case 18: // T start pose 1, 0
      goal_pose = robotis_start_position;
      goal_pose(0) -=0.0;
      goal_pose(1) +=0.12 + 0.015 - 0.003; 
      goal_pose(2) +=0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.0);
    break;
    case 19: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 20: // drawing O
      draw_alphabet_arg[0] = 'T'; // drawing alphabet
      draw_alphabet_arg[1] = 30; // drawing scale 3cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 21: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;

    case 22: // I start pose 0, 0
      goal_pose = robotis_start_position;
      goal_pose(0) -=0.0;
      goal_pose(1) +=0.15 - 0.003;  
      goal_pose(2) +=0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.0);
    break;
    case 23: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 24: // drawing O
      draw_alphabet_arg[0] = 'I'; // drawing alphabet
      draw_alphabet_arg[1] = 30; // drawing scale 3cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 25: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;
    
    case 26: // S start pose 0, 0
      goal_pose = robotis_start_position;
      goal_pose(0) -=0.0;
      goal_pose(1) +=0.18 - 0.003; 
      goal_pose(2) +=0.01;
      open_manipulator->makeTaskTrajectory("pen", goal_pose, 1.0);
    break;
    case 27: // z down
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.5);
    break;
    case 28: // drawing O
      draw_alphabet_arg[0] = 'S'; // drawing alphabet
      draw_alphabet_arg[1] = 30; // drawing scale 3cm
      open_manipulator->makeCustomTrajectory(DRAWING_ALPHABET, "pen", p_draw_alphabet_arg, 5.0);
    break;
    case 29: // z up
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.5);
    break;

    case 30: // home
      goal_joint_position.push_back( 0.00);
      goal_joint_position.push_back(-1.05);
      goal_joint_position.push_back( 0.35);
      goal_joint_position.push_back( 0.69);
      open_manipulator->makeJointTrajectory(goal_joint_position, 1.5); 
    break;
  }
}

void playMotion(OpenManipulatorPen *open_manipulator)
{
  if(!open_manipulator->getMovingState() && demo_motion_state)
  {
    log::println("Demo cnt ", demo_motion_cnt);
    if(demo_motion_type == 1) // robotis
      demo_motion_robotis(open_manipulator, demo_motion_cnt);
    else if(demo_motion_type == 2) // mobile manipulator
      demo_motion_mm(open_manipulator, demo_motion_cnt);
    else if(demo_motion_type == 3) // robotis + mobile manipulator  
      demo_motion_robotis_mm(open_manipulator, demo_motion_cnt);
    demo_motion_cnt ++;
    if(demo_motion_cnt > 46)
    {
      demo_motion_cnt = 0;
      demo_motion_state = false;
    }
  }
}

void switchInit()
{
  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);
}

void switchRead(OpenManipulatorPen *open_manipulator)
{
  if(digitalRead(BDPIN_PUSH_SW_1))
  {
    demo_motion_state = true;
    demo_motion_type = 1; // ROBOTIS
  }
  else if(digitalRead(BDPIN_PUSH_SW_2))
  {
    demo_motion_state = true;
    demo_motion_type = 2; // mobile manipulator
  }
}
#endif // OPEN_MANIPULATOR_PEN_MOTION_H_



