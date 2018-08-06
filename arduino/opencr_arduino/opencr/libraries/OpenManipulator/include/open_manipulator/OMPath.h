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

/* Authors: Darby Lim, Hye-Jong KIM */

#ifndef OMPATH_H_
#define OMPATH_H_

#include <Eigen.h>        // Calls main Eigen matrix class library
#include <Eigen/LU>       // Calls inverse, determinant, LU decomp., etc.
#include <Eigen/Dense>

#include "OMMath.h"
#include "OMDebug.h"

using namespace Eigen;

typedef struct
{
  float position;
  float velocity;
  float acceleration;
} Trajectory

class MinimumJerk
{
 private:
  MatrixXf coeffi_;

 public:
  MinimumJerk();
  ~MinimumJerk();

  void setCoeffi(Trajectory start, Trajectory end, uint8_t link_num, float mov_time, float control_period);

  float getPosition(uint8_t to, float tick);
  float getVelocity(uint8_t to, float tick);
  float getAcceleration(uint8_t to, float tick);
};

#endif // OMPATH_H_
