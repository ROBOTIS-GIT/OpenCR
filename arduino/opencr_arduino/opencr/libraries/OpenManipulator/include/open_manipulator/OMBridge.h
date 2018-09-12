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

/* Authors: Hye-Jong KIM, Darby Lim, Ryan Shim, Yong-Ho Na */

#ifndef OMBRIDGE_H_
#define OMBRIDGE_H_

#include <unistd.h>
#include <vector>

#include "OMDebug.h"

namespace OM_PROCESSING
{
  void split(String data, char separator, String* temp);
  void initProcessing(int8_t joint_angle_vector_size);
  void sendAngle2Processing(std::vector<float> joint_angle_vector);
  void sendToolData2Processing(bool onoff);
  void sendToolData2Processing(float value);
} // namespace PROCESSING
#endif