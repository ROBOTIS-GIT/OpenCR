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

#ifndef OPENMANIPULATOR_LINK_H_
#define OPENMANIPULATOR_LINK_H_

#include "../../include/open_manipulator/OMManager.h"
#include "../../include/open_manipulator/OMMath.h"

Manipulator<3, 11, 8, 1> omlink(3); // <type "link", the number of joint, the number of link, the number of tool> (dof)

void initManipulator()
{

  omlink.joint[0].init(1, makeEigenVector3(0,0,1));
  omlink.joint[1].init(2, makeEigenVector3(0,1,0));
  omlink.joint[2].init(3, makeEigenVector3(0,1,0));
  omlink.joint[3].init(-1, makeEigenVector3(0,1,0));
  omlink.joint[4].init(-1, makeEigenVector3(0,1,0));
  omlink.joint[5].init(-1, makeEigenVector3(0,1,0));
  omlink.joint[6].init(-1, makeEigenVector3(0,1,0));
  omlink.joint[7].init(-1, makeEigenVector3(0,1,0));
  omlink.joint[8].init(-1, makeEigenVector3(0,1,0));
  omlink.joint[9].init(-1, makeEigenVector3(0,1,0));
  omlink.joint[10].init(-1, makeEigenVector3(0,1,0));

  omlink.base.init(1);
  omlink.base.setPostion(Eigen::Vector3f::Zero());
  omlink.base.setOrientation(Eigen::Matrix3f::Identity(3,3));
  omlink.base.setInnerJoint(0, makeEigenVector3(150, 0, 0), makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

  omlink.link[0].init(4);
  omlink.link[0].setInnerJoint(0, makeEigenVector3(), makeRotationMatrix());
  omlink.link[0].setInnerJoint(1, makeEigenVector3(), makeRotationMatrix());
  omlink.link[0].setInnerJoint(2, makeEigenVector3(), makeRotationMatrix());
  omlink.link[0].setInnerJoint(7, makeEigenVector3(), makeRotationMatrix());

  omlink.link[1].init(2);
  omlink.link[1].setInnerJoint(1, makeEigenVector3(), makeRotationMatrix());
  omlink.link[1].setInnerJoint(5, makeEigenVector3(), makeRotationMatrix());

  omlink.link[2].init(2);
  omlink.link[2].setInnerJoint(2, makeEigenVector3(), makeRotationMatrix());
  omlink.link[2].setInnerJoint(3, makeEigenVector3(), makeRotationMatrix());

  omlink.link[3].init(2);
  omlink.link[3].setInnerJoint(3, makeEigenVector3(), makeRotationMatrix());
  omlink.link[3].setInnerJoint(4, makeEigenVector3(), makeRotationMatrix());

  omlink.link[4].init(3);
  omlink.link[4].setInnerJoint(4, makeEigenVector3(), makeRotationMatrix());
  omlink.link[4].setInnerJoint(5, makeEigenVector3(), makeRotationMatrix());
  omlink.link[4].setInnerJoint(6, makeEigenVector3(), makeRotationMatrix());

  omlink.link[5].init(2);
  omlink.link[5].setInnerJoint(7, makeEigenVector3(), makeRotationMatrix());
  omlink.link[5].setInnerJoint(8, makeEigenVector3(), makeRotationMatrix());

  omlink.link[6].init(3);
  omlink.link[6].setInnerJoint(5, makeEigenVector3(), makeRotationMatrix());
  omlink.link[6].setInnerJoint(8, makeEigenVector3(), makeRotationMatrix());
  omlink.link[6].setInnerJoint(9, makeEigenVector3(), makeRotationMatrix());

  omlink.link[7].init(2);
  omlink.link[7].setInnerJoint(9, makeEigenVector3(), makeRotationMatrix());
  omlink.link[7].setInnerJoint(10, makeEigenVector3(), makeRotationMatrix());

  omlink.tool[0].init(2);
  omlink.tool[0].setInnerJoint(6, makeEigenVector3(), makeRotationMatrix());
  omlink.tool[0].setInnerJoint(10, makeEigenVector3(), makeRotationMatrix());

}

#endif //OPENMANIPULATOR_LINK_H_
