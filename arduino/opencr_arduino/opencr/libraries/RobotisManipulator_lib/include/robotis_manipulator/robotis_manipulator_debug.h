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

#ifndef ROBOTIS_MANIPULATOR_DEBUG_H
#define ROBOTIS_MANIPULATOR_DEBUG_H

#include <unistd.h>
#include <vector>

#if defined(__OPENCR__)
  #include <Eigen.h>
  #include <WString.h>
  #include "variant.h"

  #define DEBUG SerialBT2
#else
  #include <string>

  #define ANSI_COLOR_RED     "\x1b[31m"
  #define ANSI_COLOR_GREEN   "\x1b[32m"
  #define ANSI_COLOR_YELLOW  "\x1b[33m"
  #define ANSI_COLOR_BLUE    "\x1b[34m"
  #define ANSI_COLOR_MAGENTA "\x1b[35m"
  #define ANSI_COLOR_CYAN    "\x1b[36m"
  #define ANSI_COLOR_RESET   "\x1b[0m"
#endif

#if defined(__OPENCR__)
  typedef String		  STRING;
#else
  typedef std::string STRING;
#endif

namespace RM_LOG
{
  void PRINT(STRING str);
  void PRINT(STRING str, double data, uint8_t point = 3);
  void PRINT(const char* str);
  void PRINT(const char* str, double data, uint8_t point = 3);

  void PRINTLN(STRING str);
  void PRINTLN(STRING str, double data, uint8_t point = 3);
  void PRINTLN(const char* str);
  void PRINTLN(const char* str, double data, uint8_t point = 3);

  void INFO(STRING str);
  void INFO(STRING str, double data, uint8_t point = 3);
  void INFO(const char* str);
  void INFO(const char* str, double data, uint8_t point = 3);
  void WARN(STRING str);
  void WARN(STRING str, double data, uint8_t point = 3);
  void WARN(const char* str);
  void WARN(const char* str, double data, uint8_t point = 3);
  void ERROR(STRING str);
  void ERROR(STRING str, double data, uint8_t point = 3);
  void ERROR(const char* str);
  void ERROR(const char* str, double data, uint8_t point = 3);

  template <typename T>
  void PRINT_VECTOR(std::vector<T> &vec, uint8_t point = 3)
  {
  #if defined(__OPENCR__)
    DEBUG.print("(");
    for (uint8_t i = 0; i < vec.size(); i++)
    {
      DEBUG.print(vec.at(i), point);
      DEBUG.print(",\t");
    }
    DEBUG.println(")");
  #else
    printf("(");
    for (uint8_t i = 0; i < vec.size(); i++)
    {
      printf("%.*lf", point, vec.at(i));
      printf(",\t");
    }
    printf(")\n");
  #endif
  }

  template <typename vector>
  void PRINT_VECTOR(vector &vec, uint8_t point = 3)
  {
  #if defined(__OPENCR__)
    DEBUG.print("(");
    for (uint8_t i = 0; i < vec.size(); i++)
    {
      DEBUG.print(vec(i), point);
      DEBUG.print(",\t");
    }
    DEBUG.println(")");
  #else
    printf("(");
    for (uint8_t i = 0; i < vec.size(); i++)
    {
      printf("%.*lf", point, vec(i));
      printf(",\t");
    }
    printf(")\n");
  #endif
  }


  template <typename matrix>
  void PRINT_MATRIX(matrix &m, uint8_t point = 3)
  {
  #if defined(__OPENCR__)
    DEBUG.print("(");
    for (uint8_t i = 0; i < m.rows(); i++)
    {
      DEBUG.print(" ");
      for (uint8_t j = 0; j < m.cols(); j++)
      {
        DEBUG.print(m(i, j), point);
        DEBUG.print(",\t");
      }
      DEBUG.println();
    }
    DEBUG.println(")");
  #else
    printf("(");
    for (uint8_t i = 0; i < m.rows(); i++)
    {
      printf(" ");
      for (uint8_t j = 0; j < m.cols(); j++)
      {
        printf("%.*lf", point, m(i, j));
        printf(",\t");
      }
      printf("\n");
    }
    printf(")\n");
  #endif
  }
}

#endif // ROBOTIS_MANIPULATOR_DEBUG_H
