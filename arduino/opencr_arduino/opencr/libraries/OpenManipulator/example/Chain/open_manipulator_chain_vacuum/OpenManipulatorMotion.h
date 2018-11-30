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

#ifndef OPEN_MANIPULATOR_MOTION_H_
#define OPEN_MANIPULATOR_MOTION_H_

#if defined(__OPENCR__)
  #include <RobotisManipulator.h>
  #include "OpenManipulatorVacuum.h"
#else
  #include <robotis_manipulator/robotis_manipulator.h>
#endif

#define BDPIN_PUSH_SW_1         34
#define BDPIN_PUSH_SW_2         35

bool demo_motion_state = false;
char demo_motion_cnt = 0;

#define NUM_OF_MOTION 32
double demo_motion_way_point_buf[NUM_OF_MOTION][6] = 
{
 // j1       j2       j3       j4       time    tool 
    0.000,   0.000,   0.000,   0.000,   2.0,    0.0, // init
    0.000,  -1.050,   0.350,   0.690,   2.0,    0.0, // home
    0.431,   1.019,   0.020,  -1.049,   2.0,    0.0, // pose 1
    0.431,   1.019,   0.020,  -1.049,   1.0,    1.0, // pick
    0.000,  -1.050,   0.350,   0.690,   2.0,    1.0, // home
   -1.170,   1.190,  -0.988,  -1.652,   2.0,    1.0, // save pose
   -1.170,   1.190,  -0.988,  -1.652,   1.0,    0.0, // place
    0.000,  -1.050,   0.350,   0.690,   2.0,    0.0, // home
    0.535,   0.885,   0.772,  -1.706,   2.0,    0.0, // pose 2
    0.535,   0.885,   0.772,  -1.706,   1.0,    1.0, // pick
    0.000,  -1.050,   0.350,   0.690,   2.0,    1.0, // home
   -1.170,   1.190,  -0.988,  -1.652,   2.0,    1.0, // save pose
   -1.170,   1.190,  -0.988,  -1.652,   1.0,    0.0, // place
    0.000,  -1.050,   0.350,   0.690,   2.0,    0.0, // home
    0.247,   0.887,   0.535,  -1.445,   2.0,    0.0, // pose 3
    0.247,   0.887,   0.535,  -1.445,   1.0,    1.0, // pick
    0.000,  -1.050,   0.350,   0.690,   2.0,    1.0, // home
   -1.170,   1.190,  -0.988,  -1.652,   2.0,    1.0, // save pose
   -1.170,   1.190,  -0.988,  -1.652,   1.0,    0.0, // place
    0.000,  -1.050,   0.350,   0.690,   2.0,    0.0, // home
    0.052,   0.988,   0.097,  -1.058,   2.0,    0.0, // pose 4
    0.052,   0.988,   0.097,  -1.058,   1.0,    1.0, // pick
    0.000,  -1.050,   0.350,   0.690,   2.0,    1.0, // home
   -1.170,   1.190,  -0.988,  -1.652,   2.0,    1.0, // save pose
   -1.170,   1.190,  -0.988,  -1.652,   1.0,    0.0, // place
    0.000,  -1.050,   0.350,   0.690,   2.0,    0.0, // home
   -0.216,   0.876,   0.511,  -1.344,   2.0,    0.0, // pose 5
   -0.216,   0.876,   0.511,  -1.344,   1.0,    1.0, // pick
    0.000,  -1.050,   0.350,   0.690,   2.0,    1.0, // home
   -1.170,   1.190,  -0.988,  -1.652,   2.0,    1.0, // save pose
   -1.170,   1.190,  -0.988,  -1.652,   1.0,    0.0, // place
    0.000,  -1.050,   0.350,   0.690,   2.0,    0.0  // home
};


void playMotion(OPEN_MANIPULATOR_VACUUM *open_manipulator)
{
  if(!open_manipulator->isMoving() && demo_motion_state)
  {
    std::vector<double> joint_angle;
    for(int i = 0; i < NUM_OF_JOINT; i ++)
      joint_angle.push_back(demo_motion_way_point_buf[demo_motion_cnt][i]);
    open_manipulator->jointTrajectoryMove(joint_angle, demo_motion_way_point_buf[demo_motion_cnt][4]); 
    open_manipulator->toolMove("tool", demo_motion_way_point_buf[demo_motion_cnt][5]);
    
    demo_motion_cnt ++;
    if(demo_motion_cnt >= NUM_OF_MOTION)
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

void switchRead()
{
  if(digitalRead(BDPIN_PUSH_SW_1))
    demo_motion_state = true;
  else if(digitalRead(BDPIN_PUSH_SW_2))
    demo_motion_state = false;
}
#endif // OPEN_MANIPULATOR_MOTION_H_




