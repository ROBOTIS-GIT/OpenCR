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

/* Authors: Hye-Jong KIM */

#ifndef OPENMANIPULATOR_LINK_H_
#define OPENMANIPULATOR_LINK_H_

#include "OMManager.h"
#include "OMAPI.h"

#define WORLD     0
#define BASE      1
#define JOINT0    2
#define JOINT1    3
#define JOINT2    4
#define JOINT3    5
#define JOINT4    6
#define JOINT5    7
#define JOINT6    8
#define JOINT7    9
#define JOINT8    10
#define JOINT9    11
#define JOINT10   12
#define SUCTION   13

Manipulator omlink(3); //(dof)

void initOMLink()
{
  MANAGER::addWorld(omlink, WORLD, BASE);
  MANAGER::addComponent(omlink, BASE, WORLD, JOINT0, MATH::makeEigenVector3(-150, 0, 0), Matrix3f::Identity(3,3));
  MANAGER::addComponent(omlink, JOINT0, BASE, JOINT1, Vector3f::Zero(), Matrix3f::Identity(3,3), 1, MATH::makeEigenVector3(0,0,1));
  MANAGER::addComponentChild(omlink, JOINT0, JOINT2);
  MANAGER::addComponentChild(omlink, JOINT0, JOINT7);
  MANAGER::addComponent(omlink, JOINT1, JOINT0, JOINT5, MATH::makeEigenVector3(0, 22, 52), Matrix3f::Identity(3,3), 1, MATH::makeEigenVector3(0,1,0));
  MANAGER::addComponent(omlink, JOINT2, JOINT0, JOINT3, MATH::makeEigenVector3(0, -22, 52), Matrix3f::Identity(3,3), 2, MATH::makeEigenVector3(0,1,0));
  MANAGER::addComponent(omlink, JOINT3, JOINT2, JOINT4, MATH::makeEigenVector3(50, 7, 0), Matrix3f::Identity(3,3), 3, MATH::makeEigenVector3(0,1,0));
  MANAGER::addComponent(omlink, JOINT4, JOINT3, JOINT5, MATH::makeEigenVector3(200, 6, 0), Matrix3f::Identity(3,3), -1, MATH::makeEigenVector3(0,1,0));
  MANAGER::addComponent(omlink, JOINT5, JOINT1, JOINT6, MATH::makeEigenVector3(200, -16, 0), Matrix3f::Identity(3,3), -1, MATH::makeEigenVector3(0,1,0));
  MANAGER::addComponent(omlink, JOINT6, JOINT5, SUCTION, MATH::makeEigenVector3(200, -9, 0), Matrix3f::Identity(3,3), -1, MATH::makeEigenVector3(0,1,0));
  MANAGER::addComponent(omlink, JOINT7, JOINT0, JOINT8, MATH::makeEigenVector3(-45.31539, 6, 73.13091), Matrix3f::Identity(3,3), -1, MATH::makeEigenVector3(0,1,0));
  MANAGER::addComponent(omlink, JOINT8, JOINT7, JOINT9, MATH::makeEigenVector3(200, 9, 0), Matrix3f::Identity(3,3), -1, MATH::makeEigenVector3(0,1,0));
  MANAGER::addComponent(omlink, JOINT9, JOINT8, JOINT10, MATH::makeEigenVector3(76.60444, -6, 0), Matrix3f::Identity(3,3), -1, MATH::makeEigenVector3(0,1,0));
  MANAGER::addComponent(omlink, JOINT10, JOINT9, SUCTION, MATH::makeEigenVector3(200, -6, 0), Matrix3f::Identity(3,3), -1, MATH::makeEigenVector3(0,1,0));
  MANAGER::addTool(omlink, SUCTION, JOINT6, MATH::makeEigenVector3(38.67882, 3, -13.37315), Matrix3f::Identity(3,3), 4);

  MANAGER::checkManipulatorSetting(omlink);

  connectForward(OMLinkKinematics::forward());            //KINEMATICS::foward
  connectInverse(OMLinkKinematics::geometricInverse());   //KINEMATICS::inverse
  connectGetPassiveJointAngle(myGetPassiveJointAngle());  //KINEMATICS::getPassiveJointAngle
}

void myGetPassiveJointAngle(Manipulator* omlink, bool* error = false)
{
  float joint_angle[3];
  joint_angle[0] = getComponentJointAngle(omlink, JOINT0);
  joint_angle[1] = getComponentJointAngle(omlink, JOINT1);
  joint_angle[2] = getComponentJointAngle(omlink, JOINT2);

  setComponentJointAngle(omlink, JOINT3, joint_angle[1]-joint_angle[2]);
  setComponentJointAngle(omlink, JOINT4, -M_PI-(joint_angle[1]-joint_angle[2]));
  setComponentJointAngle(omlink, JOINT5, -M_PI-(joint_angle[1]-joint_angle[2]));
  setComponentJointAngle(omlink, JOINT6, (155 * DEG2RAD)-joint_angle[2]);
  setComponentJointAngle(omlink, JOINT7, joint_angle[1]);
  setComponentJointAngle(omlink, JOINT8, (15 * DEG2RAD)-joint_angle[1]);
  setComponentJointAngle(omlink, JOINT9, joint_angle[2]-(195 * DEG2RAD));
  setComponentJointAngle(omlink, JOINT10, (90 * DEG2RAD)-joint_angle[2]);
}

#endif //OPENMANIPULATOR_LINK_H_
