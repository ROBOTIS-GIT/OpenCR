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

#ifndef OMDEBUG_H_
#define OMDEBUG_H_

#include <Eigen.h>

#define USB    Serial
#define DEBUG  SerialBT2

namespace LOG
{
  void init();
  void log(String form, String msg);
  void INFO(String msg);
  void WARN(String msg);
  void ERROR(String msg);
}

/*
Manager Error



*/

/*
Kinematics Error



*/

/*
Math Error



*/



#endif // OMDEBUG_HPP_

