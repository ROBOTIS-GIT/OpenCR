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

#ifndef LINKKINEMATICS_H_
#define LINKKINEMATICS_H_

#if defined(__OPENCR__)
  #include <RobotisManipulator.h>
#else
  #include <robotis_manipulator/robotis_manipulator.h>
#endif


using namespace robotis_manipulator;

namespace kinematics
{

/*****************************************************************************
** Kinematics Solver Using Chain Rule and Geometric Inverse For OpenManipulator Link
*****************************************************************************/
class Link : public robotis_manipulator::Kinematics
{
public:
  Link(){}
  virtual ~Link(){}

  virtual void setOption(const void *arg){}

  virtual Eigen::MatrixXd jacobian(Manipulator *manipulator, Name tool_name)
  {
    return {};
  }

  virtual void solveForwardKinematics(Manipulator *manipulator)
  {
    updatePassiveJointValue(manipulator);
    forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
  }

  virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value)
  {
    *goal_joint_value = geometricInverse(manipulator, tool_name, target_pose);
    return true;
  }

  //private
  void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
  {
    Name my_name = component_name;
    Name parent_name = manipulator->getComponentParentName(my_name);
    int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

    Pose parent_pose_value;
    Pose my_pose_value;

    //Get Parent Pose
    if (parent_name == manipulator->getWorldName())
    {
      parent_pose_value = manipulator->getWorldPose();
    }
    else
    {
      parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
    }

    //position
    my_pose_value.kinematic.position = parent_pose_value.kinematic.position
                                    + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
    //orientation
    my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name));
    //linear velocity
    my_pose_value.dynamic.linear.velocity = math::vector3(0.0, 0.0, 0.0);
    //angular velocity
    my_pose_value.dynamic.angular.velocity = math::vector3(0.0, 0.0, 0.0);
    //linear acceleration
    my_pose_value.dynamic.linear.acceleration = math::vector3(0.0, 0.0, 0.0);
    //angular acceleration
    my_pose_value.dynamic.angular.acceleration = math::vector3(0.0, 0.0, 0.0);

    manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

    for (int8_t index = 0; index < number_of_child; index++)
    {
      Name child_name = manipulator->getComponentChildName(my_name).at(index);
      forwardSolverUsingChainRule(manipulator, child_name);
    }
  }

  std::vector<JointValue> geometricInverse(Manipulator *manipulator, Name tool_name, Pose target_pose) //for basic model);
  {
    std::vector<JointValue> target_angle_vector;
    Eigen::Vector3d control_position; //joint6-joint1
    Eigen::Vector3d tool_relative_position = manipulator->getComponentRelativePositionFromParent(tool_name);
    Eigen::Vector3d base_position = manipulator->getComponentPositionFromWorld(manipulator->getWorldChildName());
    Eigen::Vector3d temp_vector;

    JointValue target_angle[3];
    double link[3];
    double temp_x;
    double temp_y;

    temp_x = target_pose.kinematic.position(0) - base_position(0);
    temp_y = target_pose.kinematic.position(1) - base_position(1);
    target_angle[0].position = atan2(temp_y, temp_x);

    control_position(0) = target_pose.kinematic.position(0) - tool_relative_position(0) * cos(target_angle[0].position);
    control_position(1) = target_pose.kinematic.position(1) - tool_relative_position(0) * sin(target_angle[0].position);
    control_position(2) = target_pose.kinematic.position(2) - tool_relative_position(2);

    temp_vector = manipulator->getComponentRelativePositionFromParent(manipulator->getComponentParentName(manipulator->getComponentParentName(manipulator->getComponentParentName(tool_name))));
    link[0] = temp_vector(2);
    temp_vector = manipulator->getComponentRelativePositionFromParent(manipulator->getComponentParentName(manipulator->getComponentParentName(tool_name)));
    link[1] = temp_vector(0);
    temp_vector = manipulator->getComponentRelativePositionFromParent(manipulator->getComponentParentName(tool_name));
    link[2] = temp_vector(0);

    temp_y = control_position(2) - base_position(2) - link[0];
    temp_x = (control_position(0) - base_position(0)) / cos(target_angle[0].position);

    target_angle[1].position = -(acos(((temp_x * temp_x + temp_y * temp_y + link[1] * link[1] - link[2] * link[2])) / (2 * link[1] * sqrt(temp_x * temp_x + temp_y * temp_y))) + atan2(temp_y, temp_x));
    target_angle[2].position = -acos((link[1] * link[1] + link[2] * link[2] - (temp_x * temp_x + temp_y * temp_y)) / (2 * link[1] * link[2])) + target_angle[1].position;

    target_angle[0].velocity = 0.0;
    target_angle[1].velocity = 0.0;
    target_angle[2].velocity = 0.0;
    target_angle[0].acceleration = 0.0;
    target_angle[1].acceleration = 0.0;
    target_angle[2].acceleration = 0.0;

    target_angle_vector.push_back(target_angle[0]);
    target_angle_vector.push_back(target_angle[1]);
    target_angle_vector.push_back(target_angle[2]);
    
    return target_angle_vector;
  }

  void updatePassiveJointValue(Manipulator *manipulator)
  {
    std::vector<double> joint_angle;
    joint_angle = manipulator->getAllActiveJointPosition();

    joint_angle.push_back(joint_angle[1] - joint_angle[2]);
    joint_angle.push_back(-M_PI - (joint_angle[1] - joint_angle[2]));
    joint_angle.push_back(-M_PI - (joint_angle[1] - joint_angle[2]));
    joint_angle.push_back(-M_PI - joint_angle[2]);
    joint_angle.push_back(joint_angle[1]);
    joint_angle.push_back(-(15 * DEG2RAD) - joint_angle[1]);
    joint_angle.push_back(joint_angle[2] - (195 * DEG2RAD));
    joint_angle.push_back((90 * DEG2RAD) - joint_angle[2]);

    manipulator->setAllJointPosition(joint_angle);
  }
};
}

#endif // LINK_KINEMATICS_H_