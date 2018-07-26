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

/* Authors: Hye-Jong KIM, Darby Lim */

#ifndef OMTRANSFORM_HPP_
#define OMTRANSFORM_HPP_

#include <unistd.h>
#include <WString.h>
#include <Eigen.h>

#include "OMMath.hpp"    
#include "OMDebug.hpp"

using namespace Eigen;

class tf
{
  Vector3f getRelativePosition(Vector3f from, Vector3f to);
  Matrix3f getRelativeOrientation(Matrix3f from, Matrix3f to);
};