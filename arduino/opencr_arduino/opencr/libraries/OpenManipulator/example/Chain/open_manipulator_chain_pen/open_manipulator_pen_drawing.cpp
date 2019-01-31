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

#include "open_manipulator_pen_drawing.h"

using namespace open_manipulator_pen_drawing;

Alphabet::Alphabet() {}
Alphabet::~Alphabet() {}

void Alphabet::initAlphabet(double move_time, TaskWaypoint start, char alphabet, char scale)
{
  start_pose_ = start;
  alphabet_ = alphabet;
  move_time_ = move_time;
  scale_ = (double)scale * 0.001 *0.5;

  Point drawingStart, drawingGoal;

  drawingStart.position = 0.0;
  drawingStart.velocity = 0.0;
  drawingStart.acceleration = 0.0;
  drawingStart.effort = 0.0;

  if(alphabet_ == 'B')  drawingGoal.position = 10.0; 
  else if(alphabet_ == 'R')  drawingGoal.position = 7.5; 
  else if(alphabet_ == 'O')  drawingGoal.position = 2.0; 
  else if(alphabet_ == 'T')  drawingGoal.position = 5.0; 
  else if(alphabet_ == 'I')  drawingGoal.position = 8.0; 
  else if(alphabet_ == 'S')  drawingGoal.position = 7.0; 
  else if(alphabet_ == 'C')  drawingGoal.position = 3.0; 
  else if(alphabet_ == '!')  
  {
    drawingGoal.position = 55.75; 
    scale_ *= 0.2;
  }
  else                      drawingGoal.position = 2.0; 

  drawingGoal.velocity = 0.0;
  drawingGoal.acceleration = 0.0;
  drawingGoal.effort = 0.0;

  path_generator_.calcCoefficient(drawingStart, drawingGoal, move_time);
  coefficient_ = path_generator_.getCoefficient();
}

TaskWaypoint Alphabet::drawAlphabet(double time_var)
{
  // get time variable
  double get_time_var = 0.0;

  get_time_var = coefficient_(0) +
                 coefficient_(1) * pow(time_var, 1) +
                 coefficient_(2) * pow(time_var, 2) +
                 coefficient_(3) * pow(time_var, 3) +
                 coefficient_(4) * pow(time_var, 4) +
                 coefficient_(5) * pow(time_var, 5);

  if(alphabet_ == 'B')      return drawing_B(get_time_var);
  else if(alphabet_ == 'R') return drawing_R(get_time_var);
  else if(alphabet_ == 'O') return drawing_O(get_time_var);
  else if(alphabet_ == 'T') return drawing_T(get_time_var);
  else if(alphabet_ == 'I') return drawing_I(get_time_var);
  else if(alphabet_ == 'S') return drawing_S(get_time_var);
  else if(alphabet_ == 'C') return drawing_C(get_time_var);
  else if(alphabet_ == '!') return drawing_MM(get_time_var);
  else return {};
}

void Alphabet::setOption(const void *arg){}
void Alphabet::makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg)
{
  char *c_arg = (char *)arg;
  initAlphabet(move_time, start, c_arg[0], c_arg[1]);
}
TaskWaypoint Alphabet::getTaskWaypoint(double tick){  return drawAlphabet(tick); }

TaskWaypoint Alphabet::drawing_B(double t)
{
  // set drawing trajectory
  TaskWaypoint pose;
  double diff_pose[2] = {0.0, 0.0};
  
  if(t <= 2.0)
  {
    double x = t;         
    diff_pose[0] = 0; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 3.5)
  {
    double x = t-2.0;
    diff_pose[0] = x; // x
    diff_pose[1] = 2.0; // y
  }
  else if(t <= 5.0) // up half circle
  {
    double x = (t - 3.5) * (3.14/1.5);
    diff_pose[0] = 1.5 + 0.5 * sin(x);// x
    diff_pose[1] = 1.5 + 0.5 * cos(x);// y
  }
  else if(t <= 6.0)
  {
    double x = 1.5 - (t-5.0);
    diff_pose[0] = x; // x
    diff_pose[1] = 1.0; // y
  }
  else if(t <= 7.0)
  {
    double x = (t-6.0) + 0.5;
    diff_pose[0] = x; // x
    diff_pose[1] = 1.0; // y
  }
  else if(t <= 8.5) // up half circle
  {
    double x = (t - 7.0) * (3.14/1.5);
    diff_pose[0] = 1.5 + 0.5 * sin(x);// x
    diff_pose[1] = 0.5 + 0.5 * cos(x);// y
  }
  else// if(t <= 10.0)
  {
    double x = 1.5 - (t-8.5);
    diff_pose[0] = x; // x
    diff_pose[1] = 0.0; // y
  }

  pose.kinematic.position[0] = start_pose_.kinematic.position[0] - diff_pose[1]*scale_;
  pose.kinematic.position[1] = start_pose_.kinematic.position[1] + diff_pose[0]*scale_;
  pose.kinematic.position[2] = start_pose_.kinematic.position[2];
  pose.kinematic.orientation = start_pose_.kinematic.orientation;

  return pose;
}

TaskWaypoint Alphabet::drawing_R(double t)
{
  // set drawing trajectory
  TaskWaypoint pose;
  double diff_pose[2] = {0.0, 0.0};

  if(t <= 2.0)
  {
    double x = t;         
    diff_pose[0] = 0; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 3.5)
  {
    double x = t-2.0;
    diff_pose[0] = x; // x
    diff_pose[1] = 2.0; // y
  }
  else if(t <= 5.0) // up half circle
  {
    double x = (t - 3.5) * (3.14/1.5);
    diff_pose[0] = 1.5 + 0.5 * sin(x);// x
    diff_pose[1] = 1.5 + 0.5 * cos(x);// y
  }
  else if(t <= 6.0)
  {
    double x = 1.5 - (t-5.0);
    diff_pose[0] = x; // x
    diff_pose[1] = 1.0; // y
  }
  else //if(t <= 7.5)
  {
    double x = (t-6.0) + 0.5;
    diff_pose[0] = x; // x
    diff_pose[1] = -(2.0/3.0)*x + 4.0/3.0; // y
  }

  pose.kinematic.position[0] = start_pose_.kinematic.position[0] - diff_pose[1]*scale_;
  pose.kinematic.position[1] = start_pose_.kinematic.position[1] + diff_pose[0]*scale_;
  pose.kinematic.position[2] = start_pose_.kinematic.position[2];
  pose.kinematic.orientation = start_pose_.kinematic.orientation;

  return pose;
}

TaskWaypoint Alphabet::drawing_C(double t)
{
  // set drawing trajectory
  TaskWaypoint pose;
  double diff_pose[2] = {0.0, 0.0};

  double x = (t) * (9.0/4.0*PI/3.0);

  if(x <= 3.0/4.0 * PI)
  {
    diff_pose[0] = 1.0 + cos(x + PI); // x
    diff_pose[1] = sin(x); // y
  }
  else //if(t <= 3.0)
  {
    diff_pose[0] = 1.0 +sin(x); // x
    diff_pose[1] = cos(x+PI); // y
  }

  pose.kinematic.position[0] = start_pose_.kinematic.position[0] - diff_pose[1]*scale_;
  pose.kinematic.position[1] = start_pose_.kinematic.position[1] + diff_pose[0]*scale_;
  pose.kinematic.position[2] = start_pose_.kinematic.position[2];
  pose.kinematic.orientation = start_pose_.kinematic.orientation;

  return pose;
}
TaskWaypoint Alphabet::drawing_O(double t)
{
  // set drawing trajectory
  TaskWaypoint pose;
  double diff_pose[2] = {0.0, 0.0};

  double x = (t) * (6.28/2.0);
  diff_pose[0] = 1.0 + 1.0 * cos(x+3.14);// y
  diff_pose[1] = 1.0 * sin(x);// x

  pose.kinematic.position[0] = start_pose_.kinematic.position[0] - diff_pose[1]*scale_;
  pose.kinematic.position[1] = start_pose_.kinematic.position[1] + diff_pose[0]*scale_;
  pose.kinematic.position[2] = start_pose_.kinematic.position[2];
  pose.kinematic.orientation = start_pose_.kinematic.orientation;

  return pose;
}

TaskWaypoint Alphabet::drawing_T(double t)
{
  // set drawing trajectory
  TaskWaypoint pose;
  double diff_pose[2] = {0.0, 0.0};

  if(t <= 2.0)
  {
    double x = t;         
    diff_pose[0] = 0; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 3.0)
  {
    double x = -(t-2.0);         
    diff_pose[0] = x+0.1; // x
    diff_pose[1] = 2.0; // y
  }
  else// if(t <= 4.0)
  {
    double x = -1.0 + (t-3.0);         
    diff_pose[0] = x; // x
    diff_pose[1] = 2.0; // y
  }

  pose.kinematic.position[0] = start_pose_.kinematic.position[0] - diff_pose[1]*scale_;
  pose.kinematic.position[1] = start_pose_.kinematic.position[1] + diff_pose[0]*scale_;
  pose.kinematic.position[2] = start_pose_.kinematic.position[2];
  pose.kinematic.orientation = start_pose_.kinematic.orientation;
  
  return pose;
}

TaskWaypoint Alphabet::drawing_I(double t)
{
  // set drawing trajectory
  TaskWaypoint pose;
  double diff_pose[2] = {0.0, 0.0};

  if(t <= 2.0)
  {
    double x = (t);         
    diff_pose[0] = x; // x
    diff_pose[1] = 0; // y
  }
  else if(t <= 3.0)
  {
    double x = 2.0 - (t-2.0);         
    diff_pose[0] = x-0.2; // x
    diff_pose[1] = 0; // y
  }
  else if(t <= 5.0)
  {
    double x = (t-3.0);         
    diff_pose[0] = 1.0; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 6.0)
  {
    double x = 1.0 - (t-5.0);         
    diff_pose[0] = x; // x
    diff_pose[1] = 2.0; // y
  }
  else //if(t <= 8.0)
  {
    double x = (t-6.0);         
    diff_pose[0] = x; // x
    diff_pose[1] = 2.0; // y
  }

  pose.kinematic.position[0] = start_pose_.kinematic.position[0] - diff_pose[1]*scale_;
  pose.kinematic.position[1] = start_pose_.kinematic.position[1] + diff_pose[0]*scale_;
  pose.kinematic.position[2] = start_pose_.kinematic.position[2];
  pose.kinematic.orientation = start_pose_.kinematic.orientation;

  return pose;
}

TaskWaypoint Alphabet::drawing_S(double t)
{
  // set drawing trajectory
  TaskWaypoint pose;
  double diff_pose[2] = {0.0, 0.0};

  if(t <= 1.5)
  {
    double x = t;         
    diff_pose[0] = x; // x
    diff_pose[1] = 0; // y
  }
  else if(t <= 3.0)
  {
    double x = (t - 1.5) * (3.14/1.5);
    diff_pose[0] = 1.5 + 0.5 * sin(x);// x
    diff_pose[1] = 0.5 + 0.5 * cos(x + PI);// y
  }
  else if(t <= 4.0) 
  {
    double x = 1.5 - (t - 3.0);
    diff_pose[0] = x;// x
    diff_pose[1] = 1.0;// y
  }
  else if(t <= 5.5)
  {
    double x = (t - 4.0) * (3.14/1.5);
    diff_pose[0] = 0.5 + 0.5 * sin(x+ PI);// x
    diff_pose[1] = 1.5 + 0.5 * cos(x+ PI);// y
  }
  else// if(t <= 7.0)
  {
    double x = 0.5 + (t-5.5);
    diff_pose[0] = x; // x
    diff_pose[1] = 2.0; // y
  }

  pose.kinematic.position[0] = start_pose_.kinematic.position[0] - diff_pose[1]*scale_;
  pose.kinematic.position[1] = start_pose_.kinematic.position[1] + diff_pose[0]*scale_;
  pose.kinematic.position[2] = start_pose_.kinematic.position[2];
  pose.kinematic.orientation = start_pose_.kinematic.orientation;
  
  return pose;
}

TaskWaypoint Alphabet::drawing_MM(double t)
{
  // set drawing trajectory
  TaskWaypoint pose;
  double diff_pose[2] = {0.0, 0.0};
  
  if(t <= 1.134)
  {
    double x = t;         
    diff_pose[0] = x; // x
    diff_pose[1] = 0; // y
  }
  else if(t <= 5.0)
  {
    double x = (t - 1.134) * ((8.0/3.0)*PI/3.866);
    diff_pose[0] = 2.0 + 1 * sin(-x - 2.0/3.0*PI);// x
    diff_pose[1] = 0.5 + 1 * cos(-x - 2.0/3.0*PI);// y
  }
  else if(t <= 10.634)
  {
    double x = 2.866 + (t-5.0);
    diff_pose[0] = x; // x
    diff_pose[1] = 0; // y
  }
  else if(t <= 12.0)
  {
    double x = (t - 10.634) * ((PI)/1.366);
    diff_pose[0] = 9.0 + 0.5 * cos(x+PI);// x
    diff_pose[1] = 0.0 + 0.5 * sin(x+PI);// y
  }
  else if(t <= 12.5) 
  {
    double x = 9.5 + (t-12.0);
    diff_pose[0] = x; // x
    diff_pose[1] = 0; // y
  }
  else if(t <= 15.5) 
  {
    double x = (t-12.5);
    diff_pose[0] = 10.0; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 18.5) 
  {
    double x = 10.0 - (t-15.5);
    diff_pose[0] = x; // x
    diff_pose[1] = 3.0; // y
  }
  else if(t <= 19.5) 
  {
    double x = (t-18.5);
    diff_pose[0] = 7.0; // x
    diff_pose[1] = x + 3.0; // y
  }
  else if(t <= 20.5) 
  {
    double x = 7.0 - (t-19.5);
    diff_pose[0] = x; // x
    diff_pose[1] = 4.0; // y
  }
  else if(t <= 21.0) 
  {
    double x = (t-20.5) + 4.0;
    diff_pose[0] = 6.0; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 23.5) // 2.5 
  {
    double x = (t-21.0) + 6.0;
    diff_pose[0] = x; // x
    diff_pose[1] = x - 1.5; // y
  }
  else if(t <= 24.25) // 0.75 
  {
    double x = 8.5 - (t-23.5);
    diff_pose[0] = x; // x
    diff_pose[1] = -x+15.5 ; // y
  }
  else if(t <= 24.5) // 0.25 
  {
    double x = 7.75 - (t-24.25);
    diff_pose[0] = x; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 27.5) // 3.0
  {
    double x = 7.5 - (t-24.5);
    diff_pose[0] = x; // x
    diff_pose[1] = -0.5*x+11.25 ; // y
  }
  else if(t <= 28.25) // 0.75 
  {
    double x = (t-27.5)+9.0; 
    diff_pose[0] = 4.5; // x
    diff_pose[1] = x ; // y
  }
  else if(t <= 28.75) // 0.5
  {
    double x = 4.5 - (t-28.25); 
    diff_pose[0] = x; // x
    diff_pose[1] = 9.75 ; // y
  }
  else if(t <= 29.0) // 0.25 
  {
    double x = (t-28.75)+9.75; 
    diff_pose[0] = 4.0; // x
    diff_pose[1] = x ; // y
  }
  else if(t <= 30.0) // 1.0 top
  {
    double x = 4.0 - (t-29.0); 
    diff_pose[0] = x; // x
    diff_pose[1] = 10.0; // y
  }
  else if(t <= 30.5) // 0.5 
  {
    double x = 10.0 - (t-30.0); 
    diff_pose[0] = 3.0; // x
    diff_pose[1] = x ; // y
  }
  else if(t <= 31.0) // 0.5 
  {
    double x = 3.0 - (t-30.5); 
    diff_pose[0] = x; // x
    diff_pose[1] = 9.5 ; // y
  }
  else if(t <= 31.25) // 0.25 
  {
    double x = 9.5 - (t-31.0); 
    diff_pose[0] = 2.5; // x
    diff_pose[1] = x ; // y
  }
  else if(t <= 31.75) // 0.5 
  {
    double x = 2.5 - (t-31.25); 
    diff_pose[0] = x; // x
    diff_pose[1] = 9.25 ; // y
  }
  else if(t <= 33.25) // 1.5 
  {
    double x = 2.0 - (t-31.75); 
    diff_pose[0] = x; // x
    diff_pose[1] = 0.25/1.5*x + (9-(0.25*0.5/1.5)); // y
  }
  else if(t <= 34.0) // 0.75 gripper
  {
    double x = 9.0 - (t-33.25); 
    diff_pose[0] = 0.5; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 35.5) // 1.5
  {
    double x = (t-34.0) + 0.5; 
    diff_pose[0] = x; // x
    diff_pose[1] = -0.25/1.5*x + (8.25 + (0.25*0.5/1.5)); // y
  }
  else if(t <= 36.0) // 0.5
  {
    double x = (t-35.5) + 2.0; 
    diff_pose[0] = x; // x
    diff_pose[1] = 8.0; // y
  }
  else if(t <= 36.25) // 0.25
  {
    double x = 8.0 - (t-36.0); 
    diff_pose[0] = 2.5; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 36.75) // 0.5
  {
    double x = 2.5 + (t-36.25); 
    diff_pose[0] = x; // x
    diff_pose[1] = 7.75; // y
  }
  else if(t <= 37.75) // 1.0
  {
    double x = (t-36.75) + 3.0; 
    diff_pose[0] = x; // x
    diff_pose[1] = 0.25*x + (7.0); // y
  }
  else if(t <= 38.25) // 0.5
  {
    double x = (t-37.75) + 4.0; 
    diff_pose[0] = x; // x
    diff_pose[1] = 8.0; // y
  }
  else if(t <= 40.75) // 1.5
  {
    double x = (t-38.25) + 4.5; 
    diff_pose[0] = x; // x
    diff_pose[1] = -0.5*x + (8.0 + (0.5*4.5)); // y
  }
  else if(t <= 41.0) // 0.25
  {
    double x = 6.75 - (t-40.75); 
    diff_pose[0] = 7.0; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 42.75) // 1.75
  {
    double x = 7.0 - (t-41.0); 
    diff_pose[0] = x; // x
    diff_pose[1] = x-0.5; // y
  }
  else if(t <= 43.0) // 0.25
  {
    double x = 5.25 - (t-42.75); 
    diff_pose[0] = x; // x
    diff_pose[1] = 4.75; // y
  }
  else if(t <= 44.75) // 1.75
  {
    double x = 4.75 - (t-43.00); 
    diff_pose[0] = 5.0; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 45.25) // 0.5
  {
    double x = 5.0 - (t-44.75); 
    diff_pose[0] = x; // x
    diff_pose[1] = 3.0; // y
  }
  else if(t <= 46.25) // 1.0
  {
    double x = (t-45.25) + 3.0; 
    diff_pose[0] = 4.5; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 48.25) // 2.0
  {
    double x = 4.5 - (t-46.25); 
    diff_pose[0] = x; // x
    diff_pose[1] = 4.0; // y
  }
  else if(t <= 48.75) // 0.5
  {
    double x = 4.0 - (t-48.25); 
    diff_pose[0] = 2.5; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 49.75) // 1.0
  {
    double x = 2.5 - (t-48.75); 
    diff_pose[0] = x; // x
    diff_pose[1] = 3.5; // y
  }
  else if(t <= 50.25) // 0.5
  {
    double x = 3.5 - (t-49.75); 
    diff_pose[0] = 1.5; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 51.5) // 1.25
  {
    double x = 1.5 - (t-50.25); 
    diff_pose[0] = x; // x
    diff_pose[1] = 3.0; // y
  }
  else if(t <= 52.0) // 0.5
  {
    double x = (t-51.5) + 3.0; 
    diff_pose[0] = 0.25; // x
    diff_pose[1] = x; // y
  }
  else if(t <= 52.25) // 0.25
  {
    double x = 0.25 - (t-52.0); 
    diff_pose[0] = x; // x
    diff_pose[1] = 3.5; // y
  }
  else// if(t <= 55.75) // 3.5
  {
    double x = 3.5 - (t-52.25); 
    diff_pose[0] = 0.0; // x
    diff_pose[1] = x; // y
  }

  pose.kinematic.position[0] = start_pose_.kinematic.position[0] - diff_pose[1]*scale_;
  pose.kinematic.position[1] = start_pose_.kinematic.position[1] + diff_pose[0]*scale_;
  pose.kinematic.position[2] = start_pose_.kinematic.position[2];
  pose.kinematic.orientation = start_pose_.kinematic.orientation;
  
  return pose;
}
