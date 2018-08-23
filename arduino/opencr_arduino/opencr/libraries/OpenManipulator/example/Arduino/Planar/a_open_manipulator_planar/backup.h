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

/* Authors: Darby Lim */

#ifndef OPENMANIPULATOR_PLANAR_H_
#define OPENMANIPULATOR_PLANAR_H_

#include "../../include/open_manipulator/OMManager.hpp"
#include "../../include/open_manipulator/OMMath.hpp"

// Manipulator<3, 11, 8, 1> omlink(3); // <type "link", the number of joint, the number of link, the number of tool> (dof)
Manipulator<4, 9, 7, 0> omlink(3); // <type "link", the number of joint, the number of link, the number of tool> (dof)

void initManipulator()
{
  OMMath math_;
  omlink.joint[0].init(1, math_.makeEigenVector3(0,0,1));
  omlink.joint[1].init(2, math_.makeEigenVector3(0,0,1));
  omlink.joint[2].init(3, math_.makeEigenVector3(0,0,1));
  omlink.joint[3].init(-1, math_.makeEigenVector3(0,0,1));
  omlink.joint[4].init(-1, math_.makeEigenVector3(0,0,1));
  omlink.joint[5].init(-1, math_.makeEigenVector3(0,0,1));
  omlink.joint[6].init(-1, math_.makeEigenVector3(0,0,1));
  omlink.joint[7].init(-1, math_.makeEigenVector3(0,0,1));
  omlink.joint[8].init(-1, math_.makeEigenVector3(0,0,1));

  omlink.base.init(1);
  omlink.base.setPostion(Eigen::Vector3f::Zero());
  omlink.base.setOrientation(Eigen::Matrix3f::Identity(3,3));
  omlink.base.setRelativeBasePostion(Eigen::Vector3f::Zero());
  omlink.base.setRelativeBaseOrientation(Eigen::Matrix3f::Identity(3,3));
  omlink.base.setInnerJoint(0, math_.makeEigenVector3(-150, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));    //from base_pose

// can use sqrt?
  omlink.link[0].init(3);
  omlink.link[0].setInnerJoint(0, math_.makeEigenVector3(170*cos(270), 170*sin(270), 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  omlink.link[0].setInnerJoint(1, math_.makeEigenVector3(170*cos(30),  170*sin(30),  0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  omlink.link[0].setInnerJoint(2, math_.makeEigenVector3(170*cos(150), 170*sin(150), 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

  omlink.link[1].init(2);
  omlink.link[1].setInnerJoint(0, math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  omlink.link[1].setInnerJoint(3, math_.makeEigenVector3(120*cos(45),  120*cos(45),  0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

  omlink.link[2].init(2);
  omlink.link[2].setInnerJoint(1, math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  omlink.link[2].setInnerJoint(4, math_.makeEigenVector3(120*cos(165), 120*sin(165), 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

  omlink.link[3].init(2);
  omlink.link[3].setInnerJoint(2, math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  omlink.link[3].setInnerJoint(5, math_.makeEigenVector3(120*cos(285), 120*sin(285), 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

  omlink.link[4].init(2);
  omlink.link[4].setInnerJoint(3, math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  omlink.link[4].setInnerJoint(6, math_.makeEigenVector3(120*cos(-45), 120*sin(-45), 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

  omlink.link[5].init(2);
  omlink.link[5].setInnerJoint(4, math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  omlink.link[5].setInnerJoint(7, math_.makeEigenVector3(120*cos(75),  120*sin(75),  0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

  omlink.link[6].init(2);
  omlink.link[6].setInnerJoint(5, math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  omlink.link[6].setInnerJoint(8, math_.makeEigenVector3(120*cos(195), 120*sin(195), 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

  omlink.link[7].init(3);
  omlink.link[7].setInnerJoint(6, math_.makeEigenVector3(120*cos(-90), 120*sin(-90), 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  omlink.link[7].setInnerJoint(7, math_.makeEigenVector3(120*cos(30),  120*sin(30),  0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  omlink.link[7].setInnerJoint(8, math_.makeEigenVector3(120*cos(150), 120*sin(150), 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

//   omlink.tool[0].init(2);    // for what?
//   omlink.tool[0].setRelativeToolPosition(Eigen::Vecto3f::Zero());
//   omlink.tool[0].setRelativeToolOrientation Eigen::Matrix3f::Identity(3,3));
//   omlink.tool[0].setInnerJoint(6, math_.makeEigenVector3(-38.67882, -3, 13.37315), math_.makeRotationMatrix(0*DEG2RAD, -90*DEG2RAD, 0*DEG2RAD));
//   omlink.tool[0].setInnerJoint(10, math_.makeEigenVector3(-10, 3, 54.33076), math_.makeRotationMatrix(0*DEG2RAD, -25*DEG2RAD, 0*DEG2RAD));
}

#endif //OPENMANIPULATOR_LINK_H_
