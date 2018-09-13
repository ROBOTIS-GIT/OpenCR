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

#include <OMManager.h>
#include <OMMath.h>
#include <OMKinematics.h>
#include <OMDebug.h>

#include <Eigen.h>

#define X_AXIS OM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS OM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS OM_MATH::makeVector3(0.0, 0.0, 1.0)

#define WORLD 0
#define COMP1 1
#define COMP2 2
#define COMP3 3
#define COMP4 4
#define TOOL 5

#define NONE -1

void setup()
{
  Serial.begin(57600);
  while (!Serial); // Wait for openning Serial port

  OM_MANAGER::Manipulator manipulator;
  OM_KINEMATICS::Chain kinematics;
  manipulator.addWorld(WORLD, COMP1);
  manipulator.addComponent(COMP1, WORLD, COMP2, OM_MATH::makeVector3(-0.100, 0.0, 0.0), IDENTITY_MATRIX, Z_AXIS, 1, 0.5, 1.0);
  manipulator.addComponent(COMP2, COMP1, COMP4, OM_MATH::makeVector3(0.0, 0.0, 0.050), IDENTITY_MATRIX, Y_AXIS, 2, -1.5, 1.5);
  manipulator.addComponentChild(COMP2, COMP3);
  manipulator.addComponent(COMP3, COMP2, TOOL, OM_MATH::makeVector3(0.0, 0.050, 0.0), IDENTITY_MATRIX);
  manipulator.addComponent(COMP4, COMP2, TOOL, OM_MATH::makeVector3(0.050, 0.0, 0.0), IDENTITY_MATRIX, Y_AXIS, 3, 2.0);  
  manipulator.addTool(TOOL, COMP4, OM_MATH::makeVector3(0.025, 0.0, 0.0), IDENTITY_MATRIX, 4, 1.0, 0.5);

  Eigen::MatrixXf jacobian = kinematics.jacobian(&manipulator, TOOL);

  kinematics.forward(&manipulator);
  manipulator.checkManipulatorSetting();

  Pose target;
  target.position = OM_MATH::makeVector3(-0.035, 0.0, 0.0875);

  target.orientation = IDENTITY_MATRIX;
  std::vector<float> joint_angle = kinematics.inverse(&manipulator, TOOL, target);
  // std::vector<float> joint_angle = kinematics.sr_inverse(&manipulator, TOOL, target);
  // std::vector<float> joint_angle = kinematics.position_only_inverse(&manipulator, TOOL, target);

  int8_t index = 0;
  std::map<Name, Component>::iterator it;
  for(it = manipulator.getIteratorBegin(); it != manipulator.getIteratorEnd(); it++)
  {
    if(manipulator.getComponentJointId(it->first) != -1)
    {
      manipulator.setComponentJointAngle(it->first, joint_angle.at(index));
      index++;
    }
  }
  
  kinematics.forward(&manipulator, COMP1);
  manipulator.checkManipulatorSetting();

  LOG::INFO("Result of inverse : ");
  PRINT::VECTOR(joint_angle);
}

void loop()
{
}