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

#ifndef OPEN_MANIPULATOR_VACUUM_MOTION_H_
#define OPEN_MANIPULATOR_VACUUM_MOTION_H_

#include <RobotisManipulator.h>
#include "open_manipulator_vacuum.h"

#define BDPIN_PUSH_SW_1         34
#define BDPIN_PUSH_SW_2         35

bool demo_motion_state = false;
char demo_motion_cnt = 0;

#define NUM_OF_MOTION 42
#define NUM_OF_JOINT 4
double demo_motion_way_point_buf[NUM_OF_MOTION][6] = 
{
 // j1       j2       j3       j4       time    tool 
    0.000,   0.000,   0.000,   0.000,   1.5,    0.0, // init
    0.000,  -1.050,   0.350,   0.690,   1.0,    0.0, // home

    0.433,   0.827,   0.031,  -0.827,   1.5,    0.0, // pose 1 up
    0.433,   1.019,  -0.018,  -1.045,   0.5,    0.0, // pose 1
    0.433,   1.019,  -0.018,  -1.045,   0.5,    1.0, // pick
    0.433,   0.827,   0.031,  -0.827,   0.5,    1.0, // pose 1 up
   -1.557,  -0.110,   0.360,   1.318,   1.5,    1.0, // save up
   -1.557,   0.029,   0.594,   0.919,   1.0,    1.0, // save pose
   -1.557,   0.029,   0.594,   0.919,   0.5,    0.0, // place
    0.000,  -1.050,   0.350,   0.690,   1.0,    0.0, // home

    0.540,   0.721,   0.804,  -1.522,   1.5,    0.0, // pose 2 up
    0.540,   0.844,   0.752,  -1.598,   0.5,    0.0, // pose 2
    0.540,   0.844,   0.752,  -1.598,   0.5,    1.0, // pick
    0.540,   0.721,   0.804,  -1.522,   0.5,    1.0, // pose 2 up
   -1.557,  -0.110,   0.360,   1.318,   1.5,    1.0, // save up
   -1.557,   0.029,   0.594,   0.919,   1.0,    1.0, // save pose
   -1.557,   0.029,   0.594,   0.919,   0.5,    0.0, // place
    0.000,  -1.050,   0.350,   0.690,   1.0,    0.0, // home

    0.249,   0.755,   0.575,  -1.328,   1.5,    0.0, // pose 3 up
    0.249,   0.857,   0.537,  -1.385,   0.5,    0.0, // pose 3
    0.249,   0.857,   0.537,  -1.385,   0.5,    1.0, // pick
    0.249,   0.755,   0.575,  -1.328,   0.5,    1.0, // pose 3 up
   -1.557,  -0.110,   0.360,   1.318,   1.5,    1.0, // save up
   -1.557,   0.029,   0.594,   0.919,   1.0,    1.0, // save pose
   -1.557,   0.029,   0.594,   0.919,   0.5,    0.0, // place
    0.000,  -1.050,   0.350,   0.690,   1.0,    0.0, // home

    0.046,   0.851,   0.138,  -1.000,   1.5,    0.0, // pose 4 up
    0.046,   0.966,   0.090,  -1.022,   0.5,    0.0, // pose 4
    0.046,   0.966,   0.090,  -1.022,   0.5,    1.0, // pick
    0.046,   0.851,   0.138,  -1.000,   0.5,    1.0, // pose 4 up
   -1.557,  -0.110,   0.360,   1.318,   1.5,    1.0, // save up
   -1.557,   0.029,   0.594,   0.919,   1.0,    1.0, // save pose
   -1.557,   0.029,   0.594,   0.919,   0.5,    0.0, // place
    0.000,  -1.050,   0.350,   0.690,   1.0,    0.0, // home

   -0.229,   0.724,   0.555,  -1.335,   1.5,    0.0, // pose 5 up
   -0.229,   0.879,   0.509,  -1.437,   0.5,    0.0, // pose 5
   -0.229,   0.879,   0.509,  -1.437,   0.5,    1.0, // pick
   -0.229,   0.724,   0.555,  -1.335,   0.5,    1.0, // pose 5 up
   -1.557,  -0.110,   0.360,   1.318,   1.5,    1.0, // save up
   -1.557,   0.029,   0.594,   0.919,   1.0,    1.0, // save pose
   -1.557,   0.029,   0.594,   0.919,   0.5,    0.0, // place
    0.000,  -0.077,   0.538,  -0.446,   1.0,    0.0  // ceremony
  
};


void playMotion(OpenManipulatorVacuum *open_manipulator)
{
  if(!open_manipulator->getMovingState() && demo_motion_state)
  {
    std::vector<double> joint_angle;
    for(int i = 0; i < NUM_OF_JOINT; i ++)
      joint_angle.push_back(demo_motion_way_point_buf[demo_motion_cnt][i]);
    open_manipulator->makeJointTrajectory(joint_angle, demo_motion_way_point_buf[demo_motion_cnt][4]); 
    open_manipulator->makeToolTrajectory("vacuum", demo_motion_way_point_buf[demo_motion_cnt][5]);
    
    demo_motion_cnt ++;
    if(demo_motion_cnt > NUM_OF_MOTION)
    {
      double draw_circle_arg[3];
      draw_circle_arg[0] = 0.2; // radius (m)
      draw_circle_arg[1] = 2;    // revolution
      draw_circle_arg[2] = 0.0;  // start angle position (rad)
      void* p_draw_circle_arg = &draw_circle_arg;
      open_manipulator->makeCustomTrajectory(CUSTOM_TRAJECTORY_HEART, "vacuum", p_draw_circle_arg, 4.0);

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

void switchRead()
{
  if(digitalRead(BDPIN_PUSH_SW_1))
    demo_motion_state = true;
  else if(digitalRead(BDPIN_PUSH_SW_2))
    demo_motion_state = false;
}
#endif // OPEN_MANIPULATOR_VACUUM_MOTION_H_




