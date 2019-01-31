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

#ifndef OPEN_MANIPULATOR_PEN_DRAWING_H_
#define OPEN_MANIPULATOR_PEN_DRAWING_H_

#include <RobotisManipulator.h>

using namespace robotis_manipulator;
using namespace Eigen;

namespace open_manipulator_pen_drawing
{
//-------------------- Alphabet --------------------//

class Alphabet : public CustomTaskTrajectory
{
private:
  MinimumJerk path_generator_;
  VectorXd coefficient_;

  TaskWaypoint start_pose_;
  TaskWaypoint goal_pose_;

  char alphabet_;
  double move_time_;
  double scale_;

public:
  Alphabet();
  virtual ~Alphabet();

  void initAlphabet(double move_time, TaskWaypoint start, char alphabet, char scale);
  TaskWaypoint drawAlphabet(double time_var);
  TaskWaypoint drawing_B(double t);
  TaskWaypoint drawing_R(double t);
  TaskWaypoint drawing_C(double t);
  TaskWaypoint drawing_O(double t);
  TaskWaypoint drawing_T(double t);
  TaskWaypoint drawing_I(double t);
  TaskWaypoint drawing_S(double t);
  TaskWaypoint drawing_MM(double t);
  
  virtual void setOption(const void *arg);
  virtual void makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg);
  virtual TaskWaypoint getTaskWaypoint(double tick);
};

} // namespace open_manipulator_pen_drawing
#endif // OPEN_MANIPULATOR_PEN_DRAWING_H_




