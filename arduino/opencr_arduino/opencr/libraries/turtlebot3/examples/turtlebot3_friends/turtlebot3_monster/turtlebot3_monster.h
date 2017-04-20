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

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#include <math.h>

#include <RC100.h>

#include "turtlebot3_monster_motor_driver.h"

#define WHEEL_RADIUS                    0.033     // meter
#define WHEEL_SEPARATION                0.16      // meter (BURGER => 0.16, WAFFLE => 0.287)
// #define ROBOT_LENGTH                    0.165     // meter

#define WHEEL_POS_FROM_CENTER_X_1       -0.100    // meter
#define WHEEL_POS_FROM_CENTER_Y_1       -0.128    // meter
#define WHEEL_POS_FROM_CENTER_X_2       0.100     // meter
#define WHEEL_POS_FROM_CENTER_Y_2       -0.128    // meter
#define WHEEL_POS_FROM_CENTER_X_3       -0.100    // meter
#define WHEEL_POS_FROM_CENTER_Y_3       0.128     // meter
#define WHEEL_POS_FROM_CENTER_X_4       0.100     // meter
#define WHEEL_POS_FROM_CENTER_Y_4       0.128     // meter

#define ENCODER_MIN                     -2147483648     // raw
#define ENCODER_MAX                     2147483648      // raw

#define VELOCITY_CONSTANT_VAULE         1263.632956882  // V = r * w = r * RPM * 0.10472
                                                        //   = 0.033 * 0.229 * Goal RPM * 0.10472
                                                        // Goal RPM = V * 1263.632956882

#define CONTROL_PERIOD                  8000

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI

// Function prototypes
void receiveRemoteControlData(void);
void controlMotorSpeed(void);
