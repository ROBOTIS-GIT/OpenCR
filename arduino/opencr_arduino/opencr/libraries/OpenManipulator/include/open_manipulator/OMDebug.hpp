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

#ifndef OMDEBUG_HPP_
#define OMDEBUG_HPP_

#include <Eigen.h>

#define USB    Serial
#define DEBUG  SerialBT2

namespace LOG
{
void init()
{
  DEBUG.begin(57600);
}

void log(String form, String msg)
{
  DEBUG.println(form + msg);
}

void INFO(String msg)
{
  log("[INFO] ", msg);
}

void WARN(String msg)
{
  log("[WARN] ", msg);
}

void ERROR(String msg)
{
  log("[ERROR] ", msg);
}

// void print_mt3f(const Eigen::Matrix3f& m)
// {
//   uint8_t i = 0 , j = 0;

//   for (i=0; i<3; i++)
//   {
//       for (j=0; j<3; j++)
//       {
//           Serial.print(m(i,j), 3);   // print 6 decimal places
//           Serial.print(", ");
//       }
//       Serial.println();
//   }
//   Serial.println();
// }

// void print_vt3f(const Eigen::Vector3f& v)
// {
//   uint8_t i = 0;

//   for (i=0; i<3; i++)
//   {
//     Serial.print(v(i), 3);
//     Serial.print(", ");
//   }
//   Serial.println();
// }

// void print_mtXf(const Eigen::MatrixXf& m)
// {
//   uint8_t i = 0 , j = 0;

//   for (i=0; i<m.rows(); i++)
//   {
//       for (j=0; j<m.cols(); j++)
//       {
//           Serial.print(m(i,j), 3);   // print 6 decimal places
//           Serial.print(", ");
//       }
//       Serial.println();
//   }
//   Serial.println();
// }

// void print_vtXf(const Eigen::VectorXf& v)
// {
//   uint8_t i = 0;

//   for (i=0; i<v.size(); i++)
//   {
//     Serial.print(v(i), 3);
//     Serial.print(", ");
//   }
//   Serial.println();
// }

// void showJointAngle(String unit, OPMLink* link, int from, int to)
// {
//   int num = 0;

//   if (unit == "rad")
//   {
//     for (num = from; num <= to; num++)
//     {
//       Serial.print(link[num].joint_angle_); 
//       Serial.print(" ");
//     }
//     Serial.println("");
//   }
//   else if (unit == "deg")
//   {
//     for (num = from; num <= to; num++)
//     {
//       Serial.print(link[num].joint_angle_*RAD2DEG);
//       Serial.print(" ");
//     }
//     Serial.println("");
//   }
// }

// void showFKResult(OPMLink* link, int from, int to)
// {
//   int num = 0;

//   for (num = from; num <= to; num++)
//   {
//     Serial.print("link : "); Serial.println(link[num].name_);
//     Serial.println("p_ : "); print_vt3f(link[num].p_);
//     Serial.println("R_ : "); print_mt3f(link[num].R_);
//   }
// }

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



int8_t manager_error;
int8_t kinematics_error;
int8_t math_error;
int8_t api_error;
int8_t path_error;
int8_t dynamixel_error;




#endif // OMDEBUG_HPP_

