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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "../../include/robotis_manipulator/robotis_manipulator.h"

using namespace robotis_manipulator;


/*****************************************************************************
** Constructor and Destructor
*****************************************************************************/
RobotisManipulator::RobotisManipulator()
{
  moving_state_ = false;
  actuator_added_stete_ = false;
  step_moving_state_ = false;
  trajectory_initialized_state_ = false;
}

RobotisManipulator::~RobotisManipulator() {}


/*****************************************************************************
** Initialize Function
*****************************************************************************/
void RobotisManipulator::addWorld(Name world_name,
                           Name child_name,
                           Eigen::Vector3d world_position,
                           Eigen::Matrix3d world_orientation)
{
  manipulator_.addWorld(world_name, child_name, world_position, world_orientation);
}

void RobotisManipulator::addJoint(Name my_name,
                                  Name parent_name,
                                  Name child_name,
                                  Eigen::Vector3d relative_position,
                                  Eigen::Matrix3d relative_orientation,
                                  Eigen::Vector3d axis_of_rotation,
                                  int8_t joint_actuator_id, 
                                  double max_position_limit, 
                                  double min_position_limit,
                                  double coefficient,
                                  double mass,
                                  Eigen::Matrix3d inertia_tensor,
                                  Eigen::Vector3d center_of_mass)
{
  manipulator_.addJoint(my_name, parent_name, child_name, 
                        relative_position, relative_orientation, axis_of_rotation, joint_actuator_id, 
                        max_position_limit, min_position_limit, coefficient, mass, 
                        inertia_tensor, center_of_mass);
}

void RobotisManipulator::addComponentChild(Name my_name, Name child_name)
{
  manipulator_.addComponentChild(my_name, child_name);
}

void RobotisManipulator::addTool(Name my_name,
                                 Name parent_name,
                                 Eigen::Vector3d relative_position,
                                 Eigen::Matrix3d relative_orientation,
                                 int8_t tool_id, 
                                 double max_position_limit, 
                                 double min_position_limit,
                                 double coefficient,
                                 double mass,
                                 Eigen::Matrix3d inertia_tensor,
                                 Eigen::Vector3d center_of_mass)
{
  manipulator_.addTool(my_name, parent_name, 
                       relative_position, relative_orientation, tool_id, 
                       max_position_limit, min_position_limit, coefficient, mass,
                       inertia_tensor, center_of_mass);
}

void RobotisManipulator::printManipulatorSetting()
{
  manipulator_.printManipulatorSetting();
}

void RobotisManipulator::addKinematics(Kinematics *kinematics)
{
  kinematics_= kinematics;
}

void RobotisManipulator::addJointActuator(Name actuator_name, JointActuator *joint_actuator, std::vector<uint8_t> id_array, const void *arg)
{
  joint_actuator_.insert(std::make_pair(actuator_name, joint_actuator));
  if(joint_actuator_.find(actuator_name) != joint_actuator_.end())
  {
    joint_actuator_.at(actuator_name)->init(id_array, arg);
  }
  else
  {
    //error
  }
  for(uint32_t index = 0; index < id_array.size(); index++)
  {
    manipulator_.setComponentActuatorName(manipulator_.findComponentNameUsingId(id_array.at(index)),actuator_name);
  }
  actuator_added_stete_ = true;
}

void RobotisManipulator::addToolActuator(Name actuator_name, ToolActuator *tool_actuator, uint8_t id, const void *arg)
{
  tool_actuator_.insert(std::make_pair(actuator_name, tool_actuator));
  if(tool_actuator_.find(actuator_name) != tool_actuator_.end())
  {
    tool_actuator_.at(actuator_name)->init(id, arg);
  }
  else
  {
    //error
  }
  manipulator_.setComponentActuatorName(manipulator_.findComponentNameUsingId(id),actuator_name);
  actuator_added_stete_ = true;
}

void RobotisManipulator::addCustomTrajectory(Name trajectory_name, CustomJointTrajectory *custom_trajectory)
{
  trajectory_.addCustomTrajectory(trajectory_name, custom_trajectory);
}

void RobotisManipulator::addCustomTrajectory(Name trajectory_name, CustomTaskTrajectory *custom_trajectory)
{
  trajectory_.addCustomTrajectory(trajectory_name, custom_trajectory);
}


/*****************************************************************************
** Manipulator Function
*****************************************************************************/
Manipulator *RobotisManipulator::getManipulator()
{
  return &manipulator_;
}

JointValue RobotisManipulator::getJointValue(Name joint_name)
{
  return manipulator_.getJointValue(joint_name);
}

JointValue RobotisManipulator::getToolValue(Name tool_name)
{
  return manipulator_.getJointValue(tool_name);
}

std::vector<JointValue> RobotisManipulator::getAllActiveJointValue()
{
  return manipulator_.getAllActiveJointValue();
}

std::vector<JointValue> RobotisManipulator::getAllJointValue()
{
  return manipulator_.getAllJointValue();
}

std::vector<double> RobotisManipulator::getAllToolPosition()
{
  return manipulator_.getAllToolPosition();
}

std::vector<JointValue> RobotisManipulator::getAllToolValue()
{
  return manipulator_.getAllToolValue();
}

KinematicPose RobotisManipulator::getKinematicPose(Name component_name)
{
  return manipulator_.getComponentKinematicPoseFromWorld(component_name);
}

DynamicPose RobotisManipulator::getDynamicPose(Name component_name)
{
  return manipulator_.getComponentDynamicPoseFromWorld(component_name);
}

Pose RobotisManipulator::getPose(Name component_name)
{
  return manipulator_.getComponentPoseFromWorld(component_name);
}


/*****************************************************************************
** Kinematics Function (Including Virtual Function)
*****************************************************************************/
Eigen::MatrixXd RobotisManipulator::jacobian(Name tool_name)
{
  return kinematics_->jacobian(&manipulator_, tool_name);
}

void RobotisManipulator::solveForwardKinematics()
{
  return kinematics_->solveForwardKinematics(&manipulator_);
}

bool RobotisManipulator::solveInverseKinematics(Name tool_name, Pose goal_pose, std::vector<JointValue>* goal_joint_value)
{
  return kinematics_->solveInverseKinematics(&manipulator_, tool_name, goal_pose, goal_joint_value);
}

void RobotisManipulator::setKinematicsOption(const void* arg)
{
  kinematics_->setOption(arg);
}


/*****************************************************************************
** Actuator Function (Including Virtual Function)
*****************************************************************************/
void RobotisManipulator::setJointActuatorMode(Name actuator_name, std::vector<uint8_t> id_array, const void *arg)
{
  if(actuator_added_stete_)
  {
    if(joint_actuator_.find(actuator_name) != joint_actuator_.end())
    {
      joint_actuator_.at(actuator_name)->setMode(id_array, arg);
    }
    else
    {
      robotis_manipulator::log::error("[jointActuatorSetMode] Worng Actuator Name.");
    }
  }
}

void RobotisManipulator::setToolActuatorMode(Name actuator_name, const void *arg)
{
  if(actuator_added_stete_)
  {
    if(tool_actuator_.find(actuator_name) != tool_actuator_.end())
    {
      tool_actuator_.at(actuator_name)->setMode(arg);
    }
    else
    {
      //error
    }
  }
}

std::vector<uint8_t> RobotisManipulator::getJointActuatorId(Name actuator_name)
{
  if(actuator_added_stete_)
  {
    if(joint_actuator_.find(actuator_name) != joint_actuator_.end())
    {
      return joint_actuator_.at(actuator_name)->getId();
    }
    else
    {
      //error
    }
  }
  return {};
}

uint8_t RobotisManipulator::getToolActuatorId(Name actuator_name)
{
  if(actuator_added_stete_)
  {
    if(tool_actuator_.find(actuator_name) != tool_actuator_.end())
    {
      return tool_actuator_.at(actuator_name)->getId();
    }
    else
    {
      //error
    }
  }
  return {};
}

void RobotisManipulator::enableActuator(Name actuator_name)
{
  if(actuator_added_stete_)
  {
    if(joint_actuator_.find(actuator_name) != joint_actuator_.end())
    {
      joint_actuator_.at(actuator_name)->enable();
    }
    else if(tool_actuator_.find(actuator_name) != tool_actuator_.end())
    {
      tool_actuator_.at(actuator_name)->enable();
    }
    else
    {
      //error
    }
  }
  trajectory_initialized_state_ = false;
}

void RobotisManipulator::disableActuator(Name actuator_name)
{
  if(actuator_added_stete_)
  {
    if(joint_actuator_.find(actuator_name) != joint_actuator_.end())
    {
      joint_actuator_.at(actuator_name)->disable();
    }
    else if(tool_actuator_.find(actuator_name) != tool_actuator_.end())
    {
      tool_actuator_.at(actuator_name)->disable();
    }
    else
    {
      //error
    }
  }
}

void RobotisManipulator::enableAllJointActuator()
{
  if(actuator_added_stete_)
  {
    std::map<Name, JointActuator *>::iterator it_joint_actuator;
    for(it_joint_actuator = joint_actuator_.begin(); it_joint_actuator != joint_actuator_.end(); it_joint_actuator++)
    {
      joint_actuator_.at(it_joint_actuator->first)->enable();
    }
  }
  trajectory_initialized_state_ = false;
}

void RobotisManipulator::disableAllJointActuator()
{
  if(actuator_added_stete_)
  {
    std::map<Name, JointActuator *>::iterator it_joint_actuator;
    for(it_joint_actuator = joint_actuator_.begin(); it_joint_actuator != joint_actuator_.end(); it_joint_actuator++)
    {
      joint_actuator_.at(it_joint_actuator->first)->disable();
    }
  }
}

void RobotisManipulator::enableAllToolActuator()
{
  if(actuator_added_stete_)
  {
    std::map<Name, ToolActuator *>::iterator it_tool_actuator;
    for(it_tool_actuator = tool_actuator_.begin(); it_tool_actuator != tool_actuator_.end(); it_tool_actuator++)
    {
      tool_actuator_.at(it_tool_actuator->first)->enable();
    }
  }
  trajectory_initialized_state_ = false;
}

void RobotisManipulator::disableAllToolActuator()
{
  if(actuator_added_stete_)
  {
    std::map<Name, ToolActuator *>::iterator it_tool_actuator;
    for(it_tool_actuator = tool_actuator_.begin(); it_tool_actuator != tool_actuator_.end(); it_tool_actuator++)
    {
      tool_actuator_.at(it_tool_actuator->first)->disable();
    }
  }
}

void RobotisManipulator::enableAllActuator()
{
  if(actuator_added_stete_)
  {
    std::map<Name, JointActuator *>::iterator it_joint_actuator;
    for(it_joint_actuator = joint_actuator_.begin(); it_joint_actuator != joint_actuator_.end(); it_joint_actuator++)
    {
      joint_actuator_.at(it_joint_actuator->first)->enable();
    }
    std::map<Name, ToolActuator *>::iterator it_tool_actuator;
    for(it_tool_actuator = tool_actuator_.begin(); it_tool_actuator != tool_actuator_.end(); it_tool_actuator++)
    {
      tool_actuator_.at(it_tool_actuator->first)->enable();
    }
  }
  trajectory_initialized_state_ = false;
}

void RobotisManipulator::disableAllActuator()
{
  if(actuator_added_stete_)
  {
    std::map<Name, JointActuator *>::iterator it_joint_actuator;
    for(it_joint_actuator = joint_actuator_.begin(); it_joint_actuator != joint_actuator_.end(); it_joint_actuator++)
    {
      joint_actuator_.at(it_joint_actuator->first)->disable();
    }
    std::map<Name, ToolActuator *>::iterator it_tool_actuator;
    for(it_tool_actuator = tool_actuator_.begin(); it_tool_actuator != tool_actuator_.end(); it_tool_actuator++)
    {
      tool_actuator_.at(it_tool_actuator->first)->disable();
    }
  }
}

bool RobotisManipulator::getActuatorEnabledState(Name actuator_name)
{
  if(actuator_added_stete_)
  {
    if(joint_actuator_.find(actuator_name) != joint_actuator_.end())
    {
      return joint_actuator_.at(actuator_name)->getEnabledState();
    }
    else if(tool_actuator_.find(actuator_name) != tool_actuator_.end())
    {
      return tool_actuator_.at(actuator_name)->getEnabledState();
    }
    else
    {
      return {};
    }
  }
  return {};
}

bool RobotisManipulator::sendJointActuatorValue(Name joint_component_name, JointValue value)
{
  // trajectory manipulator set
  // trajectory_.getManipulator()->setJointValue(joint_component_name,value);
  // trajectory_.updatePresentWayPoint(kinematics_dynamics_);

  if(actuator_added_stete_)
  {
    double coefficient = manipulator_.getCoefficient(joint_component_name);
    value.position = value.position / coefficient;
    value.velocity = value.velocity / coefficient;
    value.acceleration = value.acceleration / coefficient;
    value.effort = value.effort;

    std::vector<uint8_t> id;
    std::vector<JointValue> value_vector;
    id.push_back(manipulator_.getId(joint_component_name));
    value_vector.push_back(value);

    //send to actuator
    return joint_actuator_.at(manipulator_.getComponentActuatorName(joint_component_name))->sendJointActuatorValue(id, value_vector);
  }
  else
  {
    manipulator_.setJointValue(joint_component_name, value);
    return true;
  }
  return false;
}

bool RobotisManipulator::sendMultipleJointActuatorValue(std::vector<Name> joint_component_name, std::vector<JointValue> value_vector)
{
  if(joint_component_name.size() != value_vector.size())
    return false; //error;

  // trajectory manipulator set
  // for(uint8_t index = 0; index < joint_component_name.size(); index++)
  //   trajectory_.getManipulator()->setJointValue(joint_component_name.at(index), value_vector.at(index));
  // trajectory_.updatePresentWayPoint(kinematics_dynamics_);

  if(actuator_added_stete_)
  {
    std::vector<int8_t> joint_id;
    for(uint32_t index = 0; index < value_vector.size(); index++)
    {
      double coefficient = manipulator_.getCoefficient(joint_component_name.at(index));
      value_vector.at(index).position = value_vector.at(index).position / coefficient;
      value_vector.at(index).velocity = value_vector.at(index).velocity / coefficient;
      value_vector.at(index).acceleration = value_vector.at(index).acceleration / coefficient;
      value_vector.at(index).effort = value_vector.at(index).effort;
      joint_id.push_back(manipulator_.getId(joint_component_name.at(index)));
    }

    std::vector<uint8_t> single_actuator_id;
    std::vector<JointValue> single_value_vector;
    std::map<Name, JointActuator *>::iterator it_joint_actuator;
    for(it_joint_actuator = joint_actuator_.begin(); it_joint_actuator != joint_actuator_.end(); it_joint_actuator++)
    {
      single_actuator_id = joint_actuator_.at(it_joint_actuator->first)->getId();
      for(uint32_t index = 0; index < single_actuator_id.size(); index++)
      {
        for(uint32_t index2=0; index2 < joint_id.size(); index2++)
        {
           if(single_actuator_id.at(index) == joint_id.at(index2))
           {
             single_value_vector.push_back(value_vector.at(index2));
           }
        }
      }
      joint_actuator_.at(it_joint_actuator->first)->sendJointActuatorValue(single_actuator_id, single_value_vector);
    }
    return true;
  }
  else
  {
    //set to manipulator
    for(uint8_t index = 0; index < joint_component_name.size(); index++)
      manipulator_.setJointValue(joint_component_name.at(index), value_vector.at(index));
    return true;
  }
  return false;
}

bool RobotisManipulator::sendAllJointActuatorValue(std::vector<JointValue> value_vector)
{
  // trajectory manipulator set
  // trajectory_.setPresentJointWayPoint(value_vector);
  // trajectory_.updatePresentWayPoint(kinematics_dynamics_);

  if(actuator_added_stete_)
  {
    std::map<Name, Component>::iterator it;
    std::vector<int8_t> joint_id;
    int index = 0;
    for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
    {
      if(manipulator_.checkComponentType(it->first, ACTIVE_JOINT_COMPONENT))
      {
        double coefficient = manipulator_.getCoefficient(it->first);
        value_vector.at(index).position = value_vector.at(index).position / coefficient;
        value_vector.at(index).velocity = value_vector.at(index).velocity / coefficient;
        value_vector.at(index).acceleration = value_vector.at(index).acceleration / coefficient;
        value_vector.at(index).effort = value_vector.at(index).effort;
        joint_id.push_back(manipulator_.getId(it->first));
        index++;
      }
    }

    std::vector<uint8_t> single_actuator_id;
    std::vector<JointValue> single_value_vector;
    std::map<Name, JointActuator *>::iterator it_joint_actuator;
    for(it_joint_actuator = joint_actuator_.begin(); it_joint_actuator != joint_actuator_.end(); it_joint_actuator++)
    {
      single_actuator_id = joint_actuator_.at(it_joint_actuator->first)->getId();
      for(uint32_t index = 0; index < single_actuator_id.size(); index++)
      {
        for(uint32_t index2=0; index2 < joint_id.size(); index2++)
        {
           if(single_actuator_id.at(index) == joint_id.at(index2))
           {
             single_value_vector.push_back(value_vector.at(index2));
           }
        }
      }
      joint_actuator_.at(it_joint_actuator->first)->sendJointActuatorValue(single_actuator_id, single_value_vector);
    }
    return true;
  }
  else
  {
    //set to manipulator
    manipulator_.setAllActiveJointValue(value_vector);
  }
  return false;
}

JointValue RobotisManipulator::receiveJointActuatorValue(Name joint_component_name)
{
  if(actuator_added_stete_)
  {
    std::vector<uint8_t> actuator_id;
    std::vector<JointValue> result;

    actuator_id.push_back(manipulator_.getId(joint_component_name));

    result = joint_actuator_.at(manipulator_.getComponentActuatorName(joint_component_name))->receiveJointActuatorValue(actuator_id);

    double coefficient = manipulator_.getCoefficient(joint_component_name);
    result.at(0).position = result.at(0).position * coefficient;
    result.at(0).velocity = result.at(0).velocity * coefficient;
    result.at(0).acceleration = result.at(0).acceleration * coefficient;
    result.at(0).effort = result.at(0).effort;

    manipulator_.setJointValue(joint_component_name, result.at(0));
    return result.at(0);
  }
  return {};
}

std::vector<JointValue> RobotisManipulator::receiveMultipleJointActuatorValue(std::vector<Name> joint_component_name)
{
  if(actuator_added_stete_)
  {
    std::vector<JointValue> get_value_vector;
    std::vector<uint8_t> get_actuator_id;

    std::vector<JointValue> single_value_vector;
    std::vector<uint8_t> single_actuator_id;
    std::map<Name, JointActuator *>::iterator it_joint_actuator;
    for(it_joint_actuator = joint_actuator_.begin(); it_joint_actuator != joint_actuator_.end(); it_joint_actuator++)
    {
      single_actuator_id = joint_actuator_.at(it_joint_actuator->first)->getId();
      single_value_vector = joint_actuator_.at(it_joint_actuator->first)->receiveJointActuatorValue(single_actuator_id);
      for(uint32_t index=0; index < single_actuator_id.size(); index++)
      {
        get_actuator_id.push_back(single_actuator_id.at(index));
        get_value_vector.push_back(single_value_vector.at(index));
      }
    }

    std::vector<JointValue> result_vector;
    JointValue result;

    for(uint32_t index = 0; index < joint_component_name.size(); index++)
    {
      for(uint32_t index2 = 0; index2 < get_actuator_id.size(); index2++)
      {
        if(manipulator_.getId(joint_component_name.at(index)) == get_actuator_id.at(index2))
        {
          double coefficient = manipulator_.getCoefficient(joint_component_name.at(index));
          result.position = get_value_vector.at(index2).position * coefficient;
          result.velocity = get_value_vector.at(index2).velocity * coefficient;
          result.acceleration = get_value_vector.at(index2).acceleration * coefficient;
          result.effort = get_value_vector.at(index2).effort;
          manipulator_.setJointValue(joint_component_name.at(index), result);
          result_vector.push_back(result);
        }
      }
    }

    return result_vector;
  }
  return {};
}

std::vector<JointValue> RobotisManipulator::receiveAllJointActuatorValue()
{
  if(actuator_added_stete_)
  {
    std::vector<JointValue> get_value_vector;
    std::vector<uint8_t> get_actuator_id;

    std::vector<JointValue> single_value_vector;
    std::vector<uint8_t> single_actuator_id;
    std::map<Name, JointActuator *>::iterator it_joint_actuator;
    for(it_joint_actuator = joint_actuator_.begin(); it_joint_actuator != joint_actuator_.end(); it_joint_actuator++)
    {
      single_actuator_id = joint_actuator_.at(it_joint_actuator->first)->getId();
      single_value_vector = joint_actuator_.at(it_joint_actuator->first)->receiveJointActuatorValue(single_actuator_id);
      for(uint32_t index=0; index < single_actuator_id.size(); index++)
      {
        get_actuator_id.push_back(single_actuator_id.at(index));
        get_value_vector.push_back(single_value_vector.at(index));
      }
    }

    std::map<Name, Component>::iterator it;
    std::vector<JointValue> result_vector;
    JointValue result;

    for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
    {
      for(uint32_t index2 = 0; index2 < get_actuator_id.size(); index2++)
      {
        if(manipulator_.checkComponentType(it->first,ACTIVE_JOINT_COMPONENT) && manipulator_.getId(it->first) == get_actuator_id.at(index2))
        {
          double coefficient = manipulator_.getCoefficient(it->first);
          result.position = get_value_vector.at(index2).position * coefficient;
          result.velocity = get_value_vector.at(index2).velocity * coefficient;
          result.acceleration = get_value_vector.at(index2).acceleration * coefficient;
          result.effort = get_value_vector.at(index2).effort;
          manipulator_.setJointValue(it->first, result);
          result_vector.push_back(result);
        }
      }
    }

    return result_vector;
  }
  return {};
}
/////////////////////////////////////////

bool RobotisManipulator::sendToolActuatorValue(Name tool_component_name, JointValue value)
{
  // trajectory manipulator set
  // trajectory_.getManipulator()->setJointValue(tool_component_name,value);

  if(actuator_added_stete_)
  {
    double coefficient;
    coefficient = manipulator_.getCoefficient(tool_component_name);
    value.position = value.position / coefficient;
    value.velocity = value.velocity / coefficient;
    value.acceleration = value.acceleration / coefficient;
    value.effort = value.effort;

    return tool_actuator_.at(manipulator_.getComponentActuatorName(tool_component_name))->sendToolActuatorValue(value);
  }
  else
  {
    //set to manipulator
    manipulator_.setJointValue(tool_component_name, value);
    return true;
  }
  return false;
}

bool RobotisManipulator::sendMultipleToolActuatorValue(std::vector<Name> tool_component_name, std::vector<JointValue> value_vector)
{
  // trajectory manipulator set
  // for(uint8_t index = 0; index < tool_component_name.size(); index++)
  //   trajectory_.getManipulator()->setJointValue(tool_component_name.at(index), value_vector.at(index));

  if(actuator_added_stete_)
  {
    for (uint32_t index = 0; index < tool_component_name.size(); index++)
    {
      double coefficient = manipulator_.getCoefficient(tool_component_name.at(index));
      value_vector.at(index).position = value_vector.at(index).position / coefficient;
      value_vector.at(index).velocity = value_vector.at(index).velocity / coefficient;
      value_vector.at(index).acceleration = value_vector.at(index).acceleration / coefficient;

      if(!tool_actuator_.at(manipulator_.getComponentActuatorName(tool_component_name.at(index)))->sendToolActuatorValue(value_vector.at(index)))
        return false;
    }
    return true;
  }
  else
  {
    //set to manipulator
    for(uint8_t index = 0; index < tool_component_name.size(); index++)
      manipulator_.setJointValue(tool_component_name.at(index), value_vector.at(index));
    return true;
  }
  return false;
}

bool RobotisManipulator::sendAllToolActuatorValue(std::vector<JointValue> value_vector)
{
  // trajectory manipulator set
  // trajectory_.getManipulator()->setAllToolValue(value_vector);

  if(actuator_added_stete_)
  {
    std::vector<Name> tool_component_name;
    tool_component_name = manipulator_.getAllToolComponentName();
    for (uint32_t index = 0; index < tool_component_name.size(); index++)
    {
      double coefficient = manipulator_.getCoefficient(tool_component_name.at(index));
      value_vector.at(index).position = value_vector.at(index).position / coefficient;
      value_vector.at(index).velocity = value_vector.at(index).velocity / coefficient;
      value_vector.at(index).acceleration = value_vector.at(index).acceleration / coefficient;

      if(!tool_actuator_.at(manipulator_.getComponentActuatorName(tool_component_name.at(index)))->sendToolActuatorValue(value_vector.at(index)))
        return false;
    }
    return true;
  }
  else
  {
    //set to manipualtor
    manipulator_.setAllToolValue(value_vector);
  }
  return false;
}

JointValue RobotisManipulator::receiveToolActuatorValue(Name tool_component_name)
{
  if(actuator_added_stete_)
  {
    JointValue result;
    result = tool_actuator_.at(manipulator_.getComponentActuatorName(tool_component_name))->receiveToolActuatorValue();

    double coefficient = manipulator_.getCoefficient(tool_component_name);
    result.position = result.position * coefficient;
    result.velocity = result.velocity * coefficient;
    result.acceleration = result.acceleration * coefficient;

    manipulator_.setJointValue(tool_component_name, result);
    return result;
  }
  return {};
}

std::vector<JointValue> RobotisManipulator::receiveMultipleToolActuatorValue(std::vector<Name> tool_component_name)
{
  if(actuator_added_stete_)
  {
    std::vector<JointValue> result_vector;
    ActuatorValue result;
    for (uint32_t index = 0; index < tool_component_name.size(); index++)
    {
      result = tool_actuator_.at(manipulator_.getComponentActuatorName(tool_component_name.at(index)))->receiveToolActuatorValue();

      double coefficient = manipulator_.getCoefficient(tool_component_name.at(index));
      result.position = result.position * coefficient;
      result.velocity = result.velocity * coefficient;
      result.acceleration = result.acceleration * coefficient;

      manipulator_.setJointValue(tool_component_name.at(index), result);
      result_vector.push_back(result);
    }
    return result_vector;
  }
  return {};
}

std::vector<JointValue> RobotisManipulator::receiveAllToolActuatorValue()
{
  if(actuator_added_stete_)
  {
    std::vector<Name> tool_component_name;
    tool_component_name = manipulator_.getAllToolComponentName();
    std::vector<JointValue> result_vector;
    ActuatorValue result;
    for (uint32_t index = 0; index < tool_component_name.size(); index++)
    {
      result = tool_actuator_.at(manipulator_.getComponentActuatorName(tool_component_name.at(index)))->receiveToolActuatorValue();

      double coefficient = manipulator_.getCoefficient(tool_component_name.at(index));
      result.position = result.position * coefficient;
      result.velocity = result.velocity * coefficient;
      result.acceleration = result.acceleration * coefficient;

      manipulator_.setJointValue(tool_component_name.at(index), result);
      result_vector.push_back(result);
    }
    return result_vector;
  }
  return {};
}



/*****************************************************************************
** Time Function
*****************************************************************************/
void RobotisManipulator::startMoving()      //Private
{
  moving_state_ = true;
  trajectory_.setStartTimeToPresentTime();
}

double RobotisManipulator::getTrajectoryMoveTime()
{
  return trajectory_.getMoveTime();
}

bool RobotisManipulator::getMovingState()
{
  return moving_state_;
}


/*****************************************************************************
** Check Joint Limit Function
*****************************************************************************/
bool RobotisManipulator::checkJointLimit(Name component_name, double joint_position)
{
  if(trajectory_.getManipulator()->checkJointLimit(component_name, joint_position))
    return true;
  else
  {
    log::error("[checkJointLimit] Goal value exceeded limit at " + STRING(component_name) + ".");
    return false;
  }
}

bool RobotisManipulator::checkJointLimit(Name component_name, JointValue value)
{
  if(trajectory_.getManipulator()->checkJointLimit(component_name, value.position))
    return true;
  else
  {
    log::error("[checkJointLimit] Goal value exceeded limit at " + STRING(component_name) + ".");
    return false;
  }
}

bool RobotisManipulator::checkJointLimit(std::vector<Name> component_name, std::vector<double> position_vector)
{
  for(uint32_t index = 0; index < component_name.size(); index++)
  {
    if(!trajectory_.getManipulator()->checkJointLimit(component_name.at(index), position_vector.at(index)))
    {
      log::error("[checkJointLimit] Goal value exceeded limit at " + STRING(component_name.at(index)) + ".");
      return false;
    }
  }
  return true;
}

bool RobotisManipulator::checkJointLimit(std::vector<Name> component_name, std::vector<JointValue> value_vector)
{
  for(uint32_t index = 0; index < component_name.size(); index++)
  {
    if(!trajectory_.getManipulator()->checkJointLimit(component_name.at(index), value_vector.at(index).position))
    {
      log::error("[checkJointLimit] Goal value exceeded limit at " + STRING(component_name.at(index)) + ".");
      return false;
    }
  }
  return true;
}


/*****************************************************************************
** Trajectory Control Fuction
*****************************************************************************/
Trajectory *RobotisManipulator::getTrajectory()
{
  return &trajectory_;
}

void RobotisManipulator::makeJointTrajectoryFromPresentPosition(std::vector<double> delta_goal_joint_position, double move_time, std::vector<JointValue> present_joint_value)
{
  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  JointWaypoint present_way_point = trajectory_.getPresentJointWaypoint();
  std::vector<double> goal_joint_position;
  for(int i = 0; i < trajectory_.getManipulator()->getDOF(); i ++)
    goal_joint_position.push_back(present_way_point.at(i).position + delta_goal_joint_position.at(i));

  makeJointTrajectory(goal_joint_position, move_time);
}

void RobotisManipulator::makeJointTrajectory(std::vector<double> goal_joint_position, double move_time, std::vector<JointValue> present_joint_value)
{
  trajectory_.setTrajectoryType(JOINT_TRAJECTORY);
  trajectory_.setMoveTime(move_time);

  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  JointWaypoint present_way_point = trajectory_.getPresentJointWaypoint();

  JointValue goal_way_point_temp;
  JointWaypoint goal_way_point;
  for (uint8_t index = 0; index < trajectory_.getManipulator()->getDOF(); index++)
  {
    goal_way_point_temp.position = goal_joint_position.at(index);
    goal_way_point_temp.velocity = 0.0;
    goal_way_point_temp.acceleration = 0.0;
    goal_way_point_temp.effort = 0.0;

    goal_way_point.push_back(goal_way_point_temp);
  }

  if(getMovingState())
  {
    moving_state_=false;
    while(!step_moving_state_);
  }
  trajectory_.makeJointTrajectory(present_way_point, goal_way_point);
  startMoving();
}

void RobotisManipulator::makeJointTrajectory(std::vector<JointValue> goal_joint_value, double move_time, std::vector<JointValue> present_joint_value)
{
  trajectory_.setTrajectoryType(JOINT_TRAJECTORY);
  trajectory_.setMoveTime(move_time);

  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  JointWaypoint present_way_point = trajectory_.getPresentJointWaypoint();

  if(getMovingState())
  {
    moving_state_=false;
    while(!step_moving_state_);
  }
  trajectory_.makeJointTrajectory(present_way_point, goal_joint_value);
  startMoving();
}

void RobotisManipulator::makeJointTrajectory(Name tool_name, Eigen::Vector3d goal_position, double move_time, std::vector<JointValue> present_joint_value)
{
  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  KinematicPose goal_pose;

  goal_pose.position = goal_position;
  goal_pose.orientation = trajectory_.getManipulator()->getComponentOrientationFromWorld(tool_name);
  makeJointTrajectory(tool_name, goal_pose, move_time);
}

void RobotisManipulator::makeJointTrajectory(Name tool_name, Eigen::Matrix3d goal_orientation, double move_time, std::vector<JointValue> present_joint_value)
{
  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  KinematicPose goal_pose;

  goal_pose.position = trajectory_.getManipulator()->getComponentPositionFromWorld(tool_name);
  goal_pose.orientation = goal_orientation;
  makeJointTrajectory(tool_name, goal_pose, move_time);
}

void RobotisManipulator::makeJointTrajectory(Name tool_name, KinematicPose goal_pose, double move_time, std::vector<JointValue> present_joint_value)
{
  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  trajectory_.setTrajectoryType(JOINT_TRAJECTORY);
  trajectory_.setMoveTime(move_time);

  JointWaypoint present_way_point = trajectory_.getPresentJointWaypoint();

  Pose temp_goal_pose;
  temp_goal_pose.kinematic = goal_pose;
  temp_goal_pose = trajectory_.removeWaypointDynamicData(temp_goal_pose);
  std::vector<JointValue> goal_joint_angle;
  if(kinematics_->solveInverseKinematics(trajectory_.getManipulator(), tool_name, temp_goal_pose, &goal_joint_angle))
  {
    if(getMovingState())
    {
      moving_state_=false;
      while(!step_moving_state_) ;
    }
    trajectory_.makeJointTrajectory(present_way_point, goal_joint_angle);
    startMoving();
  }
  else
    log::error("[JOINT_TRAJECTORY] Fail to solve IK");
}

void RobotisManipulator::makeTaskTrajectoryFromPresentPose(Name tool_name, Eigen::Vector3d position_meter, double move_time, std::vector<JointValue> present_joint_value)
{
  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  KinematicPose goal_pose;

  goal_pose.position = trajectory_.getManipulator()->getComponentPositionFromWorld(tool_name) + position_meter;
  goal_pose.orientation = trajectory_.getManipulator()->getComponentOrientationFromWorld(tool_name);
  makeTaskTrajectory(tool_name, goal_pose, move_time);
}

void RobotisManipulator::makeTaskTrajectoryFromPresentPose(Name tool_name, Eigen::Matrix3d orientation_meter, double move_time, std::vector<JointValue> present_joint_value)
{
  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  KinematicPose goal_pose;

  goal_pose.position = trajectory_.getManipulator()->getComponentPositionFromWorld(tool_name);
  goal_pose.orientation = orientation_meter * trajectory_.getManipulator()->getComponentOrientationFromWorld(tool_name);
  makeTaskTrajectory(tool_name, goal_pose, move_time);
}

void RobotisManipulator::makeTaskTrajectoryFromPresentPose(Name tool_name, KinematicPose goal_pose_delta, double move_time, std::vector<JointValue> present_joint_value)
{
  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  KinematicPose goal_pose;

  goal_pose.position = trajectory_.getManipulator()->getComponentPositionFromWorld(tool_name) + goal_pose_delta.position;
  goal_pose.orientation = goal_pose_delta.orientation * trajectory_.getManipulator()->getComponentOrientationFromWorld(tool_name);
  makeTaskTrajectory(tool_name, goal_pose, move_time);
}

void RobotisManipulator::makeTaskTrajectory(Name tool_name, Eigen::Vector3d goal_position, double move_time, std::vector<JointValue> present_joint_value)
{
  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  KinematicPose goal_pose;

  goal_pose.position = goal_position;
  goal_pose.orientation = trajectory_.getManipulator()->getComponentOrientationFromWorld(tool_name);
  makeTaskTrajectory(tool_name, goal_pose, move_time);
}

void RobotisManipulator::makeTaskTrajectory(Name tool_name, Eigen::Matrix3d goal_orientation, double move_time, std::vector<JointValue> present_joint_value)
{
  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  KinematicPose goal_pose;

  goal_pose.position = trajectory_.getManipulator()->getComponentPositionFromWorld(tool_name);
  goal_pose.orientation = goal_orientation;
  makeTaskTrajectory(tool_name, goal_pose, move_time);
}

void RobotisManipulator::makeTaskTrajectory(Name tool_name, KinematicPose goal_pose, double move_time, std::vector<JointValue> present_joint_value)
{
  trajectory_.setTrajectoryType(TASK_TRAJECTORY);
  trajectory_.setPresentControlToolName(tool_name);
  trajectory_.setMoveTime(move_time);

  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  TaskWaypoint present_task_way_point = trajectory_.getPresentTaskWaypoint(tool_name);

  TaskWaypoint goal_task_way_point;                             //data conversion
  goal_task_way_point.kinematic = goal_pose;
  goal_task_way_point = trajectory_.removeWaypointDynamicData(goal_task_way_point);

  if(getMovingState())
  {
    moving_state_=false;
    while(!step_moving_state_) ;
  }
  trajectory_.makeTaskTrajectory(present_task_way_point, goal_task_way_point);
  startMoving();
}

void RobotisManipulator::setCustomTrajectoryOption(Name trajectory_name, const void* arg)
{
  trajectory_.setCustomTrajectoryOption(trajectory_name, arg);
}

void RobotisManipulator::makeCustomTrajectory(Name trajectory_name, Name tool_name, const void *arg, double move_time, std::vector<JointValue> present_joint_value)
{
  trajectory_.setTrajectoryType(CUSTOM_TASK_TRAJECTORY);
  trajectory_.setPresentControlToolName(tool_name);
  trajectory_.setMoveTime(move_time);

  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  TaskWaypoint present_task_way_point = trajectory_.getPresentTaskWaypoint(tool_name);

  if(getMovingState())
  {
    moving_state_=false;
    while(!step_moving_state_) ;
  }
  trajectory_.makeCustomTrajectory(trajectory_name, present_task_way_point, arg);
  startMoving();
}

void RobotisManipulator::makeCustomTrajectory(Name trajectory_name, const void *arg, double move_time, std::vector<JointValue> present_joint_value)
{
  trajectory_.setTrajectoryType(CUSTOM_JOINT_TRAJECTORY);
  trajectory_.setMoveTime(move_time);

  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  JointWaypoint present_joint_way_point = trajectory_.getPresentJointWaypoint();

  if(getMovingState())
  {
    moving_state_=false;
    while(!step_moving_state_) ;
  }
  trajectory_.makeCustomTrajectory(trajectory_name, present_joint_way_point, arg);
  startMoving();
}

void RobotisManipulator::sleepTrajectory(double wait_time, std::vector<JointValue> present_joint_value)
{
  trajectory_.setTrajectoryType(JOINT_TRAJECTORY);
  trajectory_.setMoveTime(wait_time);

  if(present_joint_value.size() != 0)
  {
    trajectory_.setPresentJointWaypoint(present_joint_value);
    trajectory_.updatePresentWaypoint(kinematics_);
  }

  JointWaypoint present_joint_way_point = trajectory_.getPresentJointWaypoint();
  JointWaypoint goal_way_point_vector = trajectory_.getPresentJointWaypoint();
  goal_way_point_vector = trajectory_.removeWaypointDynamicData(goal_way_point_vector);

  if(getMovingState())
  {
    moving_state_= false;
    while(!step_moving_state_) ;
  }
  trajectory_.makeJointTrajectory(present_joint_way_point, goal_way_point_vector);
  startMoving();
}

void RobotisManipulator::makeToolTrajectory(Name tool_name, double tool_goal_position)
{
  JointValue tool_value;
  tool_value.position = tool_goal_position;
  tool_value.velocity = 0.0;
  tool_value.acceleration = 0.0;
  tool_value.effort =0.0;

  if(checkJointLimit(tool_name, tool_value))
  {
    trajectory_.setToolGoalValue(tool_name, tool_value);
  }
}



JointWaypoint RobotisManipulator::getTrajectoryJointValue(double tick_time)       //Private
{
  JointWaypoint joint_way_point_value;

  ////////////////////////Joint Trajectory/////////////////////////
  if(trajectory_.checkTrajectoryType(JOINT_TRAJECTORY))
  {
    joint_way_point_value = trajectory_.getJointTrajectory().getJointWaypoint(tick_time);

    if(!checkJointLimit(trajectory_.getManipulator()->getAllActiveJointComponentName(), joint_way_point_value))
    {
      joint_way_point_value = trajectory_.removeWaypointDynamicData(trajectory_.getPresentJointWaypoint());
      moving_state_ = false;
    }
  }
  /////////////////////////////////////////////////////////////////
  ///
  /////////////////////////Task Trajectory/////////////////////////
  else if(trajectory_.checkTrajectoryType(TASK_TRAJECTORY))
  {
    TaskWaypoint task_way_point;
    task_way_point = trajectory_.getTaskTrajectory().getTaskWaypoint(tick_time);

    if(kinematics_->solveInverseKinematics(trajectory_.getManipulator(), trajectory_.getPresentControlToolName(), task_way_point, &joint_way_point_value))
    {
      if(!checkJointLimit(trajectory_.getManipulator()->getAllActiveJointComponentName(), joint_way_point_value))
      {
        joint_way_point_value = trajectory_.removeWaypointDynamicData(trajectory_.getPresentJointWaypoint());
        moving_state_ = false;
      }
    }
    else
    {
      joint_way_point_value = trajectory_.removeWaypointDynamicData(trajectory_.getPresentJointWaypoint());
      log::error("[TASK_TRAJECTORY] fail to solve IK");
      moving_state_ = false;
    }
  }
  /////////////////////////////////////////////////////////////////
  ///
  //////////////////////Custom Trajectory/////////////////////////
  else if(trajectory_.checkTrajectoryType(CUSTOM_JOINT_TRAJECTORY))
  {
    joint_way_point_value = trajectory_.getCustomJointTrajectory(trajectory_.getPresentCustomTrajectoryName())->getJointWaypoint(tick_time);

    if(!checkJointLimit(trajectory_.getManipulator()->getAllActiveJointComponentName(), joint_way_point_value))
    {
      joint_way_point_value = trajectory_.removeWaypointDynamicData(trajectory_.getPresentJointWaypoint());
      moving_state_ = false;
    }
  }
  else if(trajectory_.checkTrajectoryType(CUSTOM_TASK_TRAJECTORY))
  {
    TaskWaypoint task_way_point;
    task_way_point = trajectory_.getCustomTaskTrajectory(trajectory_.getPresentCustomTrajectoryName())->getTaskWaypoint(tick_time);

    if(kinematics_->solveInverseKinematics(trajectory_.getManipulator(), trajectory_.getPresentControlToolName(), task_way_point, &joint_way_point_value))
    {
      if(!checkJointLimit(trajectory_.getManipulator()->getAllActiveJointComponentName(), joint_way_point_value))
      {
        joint_way_point_value = trajectory_.removeWaypointDynamicData(trajectory_.getPresentJointWaypoint());
        moving_state_ = false;
      }
    }
    else
    {
      joint_way_point_value = trajectory_.removeWaypointDynamicData(trajectory_.getPresentJointWaypoint());
      log::error("[CUSTOM_TASK_TRAJECTORY] fail to solve IK");
      moving_state_ = false;
    }
  }
  /////////////////////////////////////////////////////////////////
  //set present joint task value to trajectory manipulator
  trajectory_.setPresentJointWaypoint(joint_way_point_value);
  trajectory_.updatePresentWaypoint(kinematics_);

//  Eigen::Vector3d print_temp = trajectory_.getManipulator()->getComponentDynamicPoseFromWorld("gripper").angular.velocity;
//  log::PRINT("ang vel");
//  log::PRINT_VECTOR(print_temp);

  return joint_way_point_value;
}

std::vector<JointValue> RobotisManipulator::getJointGoalValueFromTrajectory(double present_time)
{
  trajectory_.setPresentTime(present_time);

  if(!trajectory_initialized_state_)
  {
    trajectory_.initTrajectoryWaypoint(manipulator_, kinematics_);
    trajectory_initialized_state_ = true;
  }

  if(moving_state_)
  {
    step_moving_state_ = false;
    JointWaypoint joint_goal_way_point;
    double tick_time = trajectory_.getTickTime();
    
    if(tick_time < trajectory_.getMoveTime())
    {
      moving_state_ = true;
      joint_goal_way_point = getTrajectoryJointValue(tick_time);
    }
    else
    {
      moving_state_ = false;
      joint_goal_way_point =  getTrajectoryJointValue(trajectory_.getMoveTime());
    }
    step_moving_state_ = true;
    return joint_goal_way_point;
  }
  return {};
}

std::vector<JointValue> RobotisManipulator::getJointGoalValueFromTrajectoryTickTime(double tick_time)
{
  if(!trajectory_initialized_state_)
  {
    trajectory_.initTrajectoryWaypoint(manipulator_, kinematics_);
    trajectory_initialized_state_ = true;
  }

  if(moving_state_)
  {
    step_moving_state_ = false;
    JointWaypoint joint_goal_way_point ;
    if(tick_time < trajectory_.getMoveTime())
    {
      moving_state_ = true;
      joint_goal_way_point = getTrajectoryJointValue(tick_time);
    }
    else
    {
      moving_state_ = false;
      joint_goal_way_point = getTrajectoryJointValue(trajectory_.getMoveTime());
    }
    step_moving_state_ = true;
    return joint_goal_way_point;
  }
  return {};
}

std::vector<JointValue> RobotisManipulator::getToolGoalValue()
{
  std::vector<JointValue> result_vector;
  std::vector<Name> tool_component_name = trajectory_.getManipulator()->getAllToolComponentName();
  for(uint32_t index =0; index<tool_component_name.size(); index++)
  {
    result_vector.push_back(trajectory_.getToolGoalValue(tool_component_name.at(index)));
  }
  return result_vector;
}
