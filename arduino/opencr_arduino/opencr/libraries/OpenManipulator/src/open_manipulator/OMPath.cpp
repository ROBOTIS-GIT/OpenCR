/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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

/* Authors: Hye-Jong KIM, Darby Lim*/

#include "../../include/open_manipulator/OMPath.h"

MinimumJerk::MinimumJerk() {}

MinimumJerk::~MinimumJerk() {}

void MinimumJerk::setCoeffi(Trajectory start, Trajectory end, uint8_t active_joint_num, float move_time, float control_period)
{
  uint16_t step_time = uint16_t(floor(move_time / control_period) + 1.0);
  move_time = float(step_time - 1) * control_period;

  coeffi_.resize(6, link_num);

  Matrix3f A = Matrix3f::Identity(3, 3);
  Vector3f x = Vector3f::Zero();
  Vector3f b = Vector3f::Zero();

  A << pow(move_time, 3), pow(move_time, 4), pow(move_time, 5),
      3 * pow(move_time, 2), 4 * pow(move_time, 3), 5 * pow(move_time, 4),
      6 * pow(move_time, 1), 12 * pow(move_time, 2), 20 * pow(move_time, 3);

  for (int8_t num = 0; num < active_joint_num; num++)
  {
    VectorXf single_coeffi(6);

    single_coeffi(0) = start[num].pos;
    single_coeffi(1) = start[num].vel;
    single_coeffi(2) = 0.5 * start[num].acc;

    b << (end[num].pos - start[num].pos - (start[num].vel * move_time + 0.5 * start[num].acc * pow(move_time, 2))),
        (end[num].vel - start[num].vel - (start[num].acc * move_time)),
        (end[num].acc - start[num].acc);

    ColPivHouseholderQR<Matrix3f> dec(A);
    x = dec.solve(b);

    single_coeffi(3) = x(0);
    single_coeffi(4) = x(1);
    single_coeffi(5) = x(2);

    coeffi_.col(num) = single_coeffi;
  }
}

float MinimumJerk::getPosition(Trajectory *joint, uint8_t to, float tick)
{
  for (int num = 0; num <= to; num++)
  {
    joint[num].pos = coeffi_(0, num) +
                     coeffi_(1, num) * pow(tick, 1) +
                     coeffi_(2, num) * pow(tick, 2) +
                     coeffi_(3, num) * pow(tick, 3) +
                     coeffi_(4, num) * pow(tick, 4) +
                     coeffi_(5, num) * pow(tick, 5);
  }
}

float MinimumJerk::getVelocity(Trajectory *joint, uint8_t to, float tick)
{
  for (int num = 0; num <= to; num++)
  {
    joint[num].vel = coeffi_(1, num) +
                     2 * coeffi_(2, num) * pow(tick, 1) +
                     3 * coeffi_(3, num) * pow(tick, 2) +
                     4 * coeffi_(4, num) * pow(tick, 3) +
                     5 * coeffi_(5, num) * pow(tick, 4);
  }
}

float MinimumJerk::getAcceleration(Trajectory *joint, uint8_t to, float tick)
{
  for (int num = 0; num <= to; num++)
  {
    joint[num].acc = 2 * coeffi_(2, num) +
                     6 * coeffi_(3, num) * pow(tick, 1) +
                     12 * coeffi_(4, num) * pow(tick, 2) +
                     20 * coeffi_(5, num) * pow(tick, 3);
  }
}
