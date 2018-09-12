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
#include <vector>

#define DXL_SIZE 5
#define BAUD_RATE 1000000

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define POSITION1 -20.0f
#define POSITION2 0.0f

std::vector<float> goal_position;
std::vector<float> present_position;

OM_DYNAMIXEL::Dynamixel dxl;

void setup()
{
  Serial.begin(57600);
  // while (!Serial);

  dxl.init(BAUD_RATE);
  dxl.enableAllDynamixel();

  goal_position.reserve(DXL_SIZE);
  for (uint8_t num = 0; num < DXL_SIZE; num++)
  {
    goal_position.push_back(POSITION1 * DEG2RAD);
  }

  present_position.reserve(DXL_SIZE);
}

void loop()
{
  dxl.setAngle(goal_position);

  do
  {
    present_position = dxl.getAngle();

    log();
  } while (abs(dxl.convertRadian2Value(dxl.getDynamixelIDs().at(0), goal_position.at(0)) -
               dxl.convertRadian2Value(dxl.getDynamixelIDs().at(0), present_position.at(0))) > 20);

  swap();
}

void log()
{
  Serial.print("[");
  for (int8_t index = 0; index < DXL_SIZE; index++)
  {
    Serial.print("ID : ");
    Serial.print(dxl.getDynamixelIDs().at(index));
    Serial.print(" GoalPos: ");
    Serial.print(dxl.convertRadian2Value(dxl.getDynamixelIDs().at(index), goal_position.at(index)));
    Serial.print(" PresPos: ");
    Serial.print(dxl.convertRadian2Value(dxl.getDynamixelIDs().at(index), present_position.at(index)));
    Serial.print(" | ");
  }
  Serial.println("]");
}

void swap()
{
  static bool flag = true;
  goal_position.clear();

  if (flag == true)
  {
    for (uint8_t num = 0; num < DXL_SIZE; num++)
    {
      goal_position.push_back(POSITION2 * DEG2RAD);
    }
  }
  else
  {
    for (uint8_t num = 0; num < DXL_SIZE; num++)
    {
      goal_position.push_back(POSITION1 * DEG2RAD);
    }
  }

  flag = !flag;
}