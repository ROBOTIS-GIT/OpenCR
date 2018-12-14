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

using namespace ROBOTIS_MANIPULATOR;
using namespace Eigen;

namespace OPEN_MANIPULATOR_PEN_DRAWING
{
//-------------------- Alphabet --------------------//

class Alphabet : public ROBOTIS_MANIPULATOR::DrawingTrajectory
{
private:
  WayPointType output_way_point_type_;

  ROBOTIS_MANIPULATOR::MinimumJerk path_generator_;
  VectorXd coefficient_;

  std::vector<WayPoint> start_pose_;
  std::vector<WayPoint> goal_pose_;

  char alphabet_;
  double move_time_;
  double scale_;

public:
  Alphabet();
  virtual ~Alphabet();

  void initAlphabet(double move_time, double control_time, std::vector<WayPoint> start, char alphabet, char scale);
  std::vector<WayPoint> drawAlphabet(double time_var);
  std::vector<WayPoint> drawing_A(double t);
  std::vector<WayPoint> drawing_B(double t);
  std::vector<WayPoint> drawing_R(double t);
  std::vector<WayPoint> drawing_C(double t);
  std::vector<WayPoint> drawing_O(double t);
  std::vector<WayPoint> drawing_T(double t);
  std::vector<WayPoint> drawing_I(double t);
  std::vector<WayPoint> drawing_S(double t);
  std::vector<WayPoint> drawing_MM(double t);
  

  virtual void setOption(const void *arg);
  virtual void init(double move_time, double control_time, std::vector<WayPoint> start, const void *arg);
  virtual std::vector<WayPoint> getJointWayPoint(double tick);
  virtual std::vector<WayPoint> getTaskWayPoint(double tick);
};

} // namespace OPEN_MANIPULATOR_PEN_DRAWING
#endif // OPEN_MANIPULATOR_PEN_DRAWING_H_




