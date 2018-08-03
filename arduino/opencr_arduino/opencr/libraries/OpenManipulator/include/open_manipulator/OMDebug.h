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
#include <WString.h>
#include "variant.h"

#define USB Serial
#define DEBUG SerialBT2

void OpenUSBSerialPort();
void OpenDebugSerialPort();

namespace LOG
{
void log(String form, String msg);
void INFO(String msg);
void WARN(String msg);
void ERROR(String msg);
} // namespace LOG

namespace VECTOR
{
template <typename vector>
void PRINT(vector &v)
{
  uint8_t i = 0;

  USB.print(" ");  
  for (i = 0; i < v.size(); i++)
  {
    USB.print(v(i), 3);
    USB.print(", ");
  }
  USB.println();
}
} // namespace VECTOR

namespace MATRIX
{
template <typename matrix>
void PRINT(matrix &m)
{
  uint8_t i = 0, j = 0;

  for (i = 0; i < m.rows(); i++)
  {
    USB.print(" ");
    for (j = 0; j < m.cols(); j++)
    {
      USB.print(m(i, j), 3); // print 6 decimal places
      USB.print(", ");
    }
    USB.println();
  }
  USB.println();
}
} // namespace MATRIX

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
