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

#include <Arduino.h>
#include <Eigen.h>

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

