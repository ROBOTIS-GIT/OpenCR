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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include <OMDynamixel.h>
#include <OMPath.h>
#include <vector>

#define DXL_SIZE 5
#define BAUD_RATE 1000000

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CONTROL_TIME 0.010f
#define MOVE_TIME 3.0f

OM_PATH::JointTrajectory joint_trajectory(DXL_SIZE);
std::vector<Trajectory> start_trajectory;
std::vector<Trajectory> goal_trajectory;

OM_DYNAMIXEL::Dynamixel dxl;
std::vector<float> goal_position;
std::vector<float> goal_velocity;
std::vector<float> goal_acceleration;
std::vector<float> present_position;

static float tTime[2] = {0.0, 0.0};

void setup()
{
  Serial.begin(57600);
  // while (!Serial); // Wait for openning Serial port

  dxl.init(BAUD_RATE);

  dxl.enableAllDynamixel();

  goal_position.reserve(DXL_SIZE);
  goal_velocity.reserve(DXL_SIZE);
  goal_acceleration.reserve(DXL_SIZE);

  present_position = dxl.getAngle();

  for (uint16_t index = 0; index < DXL_SIZE; index++)
  {
    Trajectory start, goal;

    start.position = present_position.at(index);
    start.velocity = 0.0;
    start.acceleration = 0.0;

    start_trajectory.push_back(start);

    goal.position = 0.0f * DEG2RAD;
    goal.velocity = 0.0;
    goal.acceleration = 0.0;

    goal_trajectory.push_back(goal);
  }

  joint_trajectory.init(start_trajectory, goal_trajectory, MOVE_TIME, CONTROL_TIME);
}

void loop() 
{
  float t = (float)(millis() / 1000.0f);

  if ((t-tTime[0]) >= CONTROL_TIME)
  {
    tTime[0] = t;
  }

  if ((t-tTime[1]) >= CONTROL_TIME)
  {
    jointControl();
    LOG::INFO("TIME : ", t-tTime[1]);
    
    tTime[1] = t;
  }
}

void jointControl()
{
  uint16_t step_time = uint16_t(floor(MOVE_TIME/CONTROL_TIME) + 1.0);
  float tick_time = 0;
  static uint16_t step_cnt = 0;
  static float moving = true;

  goal_position.clear();
  goal_velocity.clear();
  goal_acceleration.clear();

  if (moving)
  {
    if (step_cnt < step_time)
    {
      tick_time = CONTROL_TIME * step_cnt;
      
      goal_position = joint_trajectory.getPosition(tick_time);
      goal_velocity = joint_trajectory.getVelocity(tick_time);
      goal_acceleration = joint_trajectory.getAcceleration(tick_time);

      dxl.setAngle(goal_position);
      log();

      step_cnt++;
    }
    else
    {
      step_cnt = 0;
      moving   = false; 
    }
  }  
}

void log()
{
  Serial.print("[");
  for (int8_t index = 0; index < DXL_SIZE; index++)
  {
    Serial.print("ID : ");
    Serial.print(dxl.getDynamixelIDs().at(index));
    Serial.print(" GoalPos: ");
    Serial.print(dxl.convertRadian2Value(1, goal_position.at(index)));
    Serial.print(" | ");
  }
  Serial.println("]");
}