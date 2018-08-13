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

/* Authors: Darby Lim */

#include "OpenManipulator_Chain.h"

std::vector<Trajectory> start_trajectory_;
std::vector<Trajectory> goal_trajectory_;

void setup()
{
  Serial.begin(57600);
  while (!Serial)
    ;

  initManipulator();

  chain.setAllActiveJointAngle(CHAIN, chain.receiveAllActuatorAngle());
  chain.forward(CHAIN, COMP1);

  std::vector<float> present_position = chain.getAllActiveJointAngle(CHAIN);

  for (uint8_t index = 0; index < chain.getNumberOfActiveJoint(CHAIN); index++)
  {
    Trajectory start;
    Trajectory goal;

    start.position = present_position.at(index);
    Serial.println(start.position);
    start.velocity = 0.0f;
    start.acceleration = 0.0f;

    start_trajectory_.push_back(start);

    goal.position = 0.0f * DEG2RAD;
    goal.velocity = 0.0f;
    goal.acceleration = 0.0f;

    goal_trajectory_.push_back(goal);
  }

  Serial.println("fuck");
  chain.setMoveTime(3.0f);
  Serial.println(chain.getMoveTime());
  Serial.println(chain.getControlTime(),4);
  Serial.println(start_trajectory_.size());
  Serial.println(goal_trajectory_.size());
  Serial.println(goal_trajectory_.at(0).position);
  Serial.println(goal_trajectory_.at(1).position);
  Serial.println(goal_trajectory_.at(2).position);
  Serial.println(goal_trajectory_.at(3).position);
  chain.makeTrajectory(start_trajectory_, goal_trajectory_);
  Serial.println("fuck");
  
  // chain.move();

  // Eigen::MatrixXf coeffi = chain.getTrajectoryCoefficient();
  // PRINT::MATRIX(coeffi);

  // initThread();
  // startThread();
}

void loop()
{
  Serial.println("loop");
  osDelay(LOOP_TIME * 1000);
}

/// DON'T TOUCH BELOW CODE///

namespace THREAD
{
osThreadId loop;
osThreadId robot_state;
osThreadId motor_control;
} // namespace THREAD

void initThread()
{
  MUTEX::create();

  // define thread
  osThreadDef(THREAD_NAME_LOOP, Loop, osPriorityNormal, 0, 1024 * 10);
  osThreadDef(THREAD_NAME_ROBOT_STATE, THREAD::Robot_State, osPriorityNormal, 0, 1024 * 10);
  osThreadDef(THREAD_NAME_MOTOR_CONTROL, THREAD::Motor_Control, osPriorityNormal, 0, 1024 * 10);

  // create thread
  THREAD::loop = osThreadCreate(osThread(THREAD_NAME_LOOP), NULL);
  THREAD::robot_state = osThreadCreate(osThread(THREAD_NAME_ROBOT_STATE), NULL);
  THREAD::motor_control = osThreadCreate(osThread(THREAD_NAME_MOTOR_CONTROL), NULL);
}

void startThread()
{
  // start kernel
  Serial.println("Thread Start");
  osKernelStart();
}

static void Loop(void const *argument)
{
  (void)argument;

  for (;;)
  {
    loop();
  }
}