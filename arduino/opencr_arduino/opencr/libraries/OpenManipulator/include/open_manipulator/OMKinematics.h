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

#ifndef OMKINEMATICS_H_
#define OMKINEMATICS_H_

#include "../../include/open_manipulator/OMManager.h"
#include "../../include/open_manipulator/OMMath.h"
#include "../../include/open_manipulator/OMDebug.h"

#include <Eigen.h>        // Calls main Eigen matrix class library
#include <Eigen/LU>       // Calls inverse, determinant, LU decomp., etc.
#include <Eigen/Dense>
#include <math.h>



class OMKinematicsMethod
{
  private:
    OMMath math_;

  public: 
    OMKinematicsMethod();
    ~OMKinematicsMethod();
    void getBasePose(Base &base, Eigen::Vector3f base_position, Eigen::Matrix3f base_orientation);
    void getBaseJointPose(Manipulator *manipulator, int8_t base_joint_number);
    void getSinglejointPose(Manipulator *manipulator, int8_t joint_number, int8_t mather_joint_number, int8_t link_number);
    void getToolPose(Manipulator *manipulator, int8_t tool_number, int8_t mather_joint_number);
    void jacobian();
}

class OMChainKinematics
{
  private:
    OMMath math_;

  public:
    OMChainKinematics();
    ~OMChainKinematics();
};

class OMScaraKinematics
{
  private:
    OMMath math_;

  public:
    OMScaraKinematics();
    ~OMScaraKinematics();

};

class OMLinkKinematics
{
  private:
    OMMath math_;

    void getPassiveJointAngle(Joint *joint);

  public:
    OMLinkKinematics();
    ~OMLinkKinematics();

    void forward(Manipulator *omlink, Eigen::Vector3f base_position, Eigen::Matrix3f base_orientation);
    void forward(Manipulator *omlink);
    void inverse(Manipulator *omlink);
};

class OMDeltaKinematics
{
  private:
    OMMath math_;

  public:
    OMDeltaKinematics();
    ~OMDeltaKinematics();

};

class MYKinematics
{
 private:
   OMMath math_;

 public:
   MYKinematics();
   ~MYKinematics();
};

#endif // OMKINEMATICS_H_
