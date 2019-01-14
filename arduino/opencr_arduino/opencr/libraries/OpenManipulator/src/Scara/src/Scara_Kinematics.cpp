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

#include "../include/open_manipulator_libs/Scara_Kinematics.h"

using namespace ROBOTIS_MANIPULATOR;
using namespace SCARA_KINEMATICS;

void Scara::setOption(const void *arg)  // <--- why const void pointer is used??
{ 
  STRING *get_arg_ = (STRING *)arg;

  if(get_arg_[0] =="inverse_solver")
  {
    STRING inverse_kinematics_solver_option = get_arg_[1];
    inverse_kinematics_solver_option_ = inverse_kinematics_solver_option;
  }
}

void Scara::updatePassiveJointValue(Manipulator *manipulator){}

Eigen::MatrixXd Scara::jacobian(Manipulator *manipulator, Name tool_name)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

  Eigen::Vector3d joint_axis = ZERO_VECTOR;

  Eigen::Vector3d position_changed = ZERO_VECTOR;
  Eigen::Vector3d orientation_changed = ZERO_VECTOR;
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

  int8_t index = 0;
  Name my_name = manipulator->getIteratorBegin()->first;

  for (int8_t size = 0; size < manipulator->getDOF(); size++)
  {
    Name parent_name = manipulator->getComponentParentName(my_name);
    if (parent_name == manipulator->getWorldName())
    {
      joint_axis = manipulator->getWorldOrientation() * manipulator->getAxis(my_name);
    }
    else
    {
      joint_axis = manipulator->getComponentOrientationFromWorld(parent_name) * manipulator->getAxis(my_name);
    }

    position_changed = RM_MATH::skewSymmetricMatrix(joint_axis) *
                       (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
    orientation_changed = joint_axis;

    pose_changed << position_changed(0),
        position_changed(1),
        position_changed(2),
        orientation_changed(0),
        orientation_changed(1),
        orientation_changed(2);

    jacobian.col(index) = pose_changed;
    index++;
    my_name = manipulator->getComponentChildName(my_name).at(0); 
  }
  return jacobian;
}

void Scara::forwardKinematics(Manipulator *manipulator)  
{
  forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

bool Scara::inverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<double> *goal_joint_value)
{
  if(inverse_kinematics_solver_option_ == "position_only_inverse")
    return inverseSolverUsingPositionOnlySRJacobian(manipulator, tool_name, target_pose, goal_joint_value);
  else if (inverse_kinematics_solver_option_ == "sr_inverse")
    return inverseSolverUsingSRJacobian(manipulator, tool_name, target_pose, goal_joint_value);
  else if(inverse_kinematics_solver_option_ == "normal_inverse")
    return inverseSolverUsingJacobian(manipulator, tool_name, target_pose, goal_joint_value);
  else if(inverse_kinematics_solver_option_ == "geometric_inverse")
    return inverseKinematicsSolverUsingGeometry(manipulator, tool_name, target_pose, goal_joint_value);
  else
  {
    RM_LOG::ERROR("Wrong inverse solver name (please change the solver)");
  }
  return false;
}


//---------------------------------------------------------------------------------------------------- 

void Scara::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = manipulator->getComponentParentName(my_name);
  int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  Eigen::Vector3d parent_position_to_world, my_position_to_world;
  Eigen::Matrix3d parent_orientation_to_world, my_orientation_to_world;

  if (parent_name == manipulator->getWorldName())
  {
    parent_position_to_world = manipulator->getWorldPosition();
    parent_orientation_to_world = manipulator->getWorldOrientation();
  }
  else
  {
    parent_position_to_world = manipulator->getComponentPositionFromWorld(parent_name);
    parent_orientation_to_world = manipulator->getComponentOrientationFromWorld(parent_name);
  }

  my_position_to_world = parent_orientation_to_world * manipulator->getComponentRelativePositionFromParent(my_name) + parent_position_to_world;
  my_orientation_to_world = parent_orientation_to_world * RM_MATH::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getValue(my_name));

  manipulator->setComponentPositionFromWorld(my_name, my_position_to_world);
  manipulator->setComponentOrientationFromWorld(my_name, my_orientation_to_world);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = manipulator->getComponentChildName(my_name).at(index);
    forwardSolverUsingChainRule(manipulator, child_name);
  }
}

bool Scara::inverseSolverUsingJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<double>* goal_joint_value)
{
  const double lambda = 0.7;
  const int8_t iteration = 10;

  Manipulator _manipulator = *manipulator;

  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF());

  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd angle_changed = Eigen::VectorXd::Zero(_manipulator.getDOF());

  std::vector<double> result;

  for (int8_t count = 0; count < iteration; count++)
  {
    forwardKinematics(&_manipulator);

    jacobian = this->jacobian(&_manipulator,tool_name);

    pose_changed = RM_MATH::poseDifference(target_pose.position, _manipulator.getComponentPositionFromWorld(tool_name),
                                           target_pose.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
    if (pose_changed.norm() < 1E-6)
    {
      *goal_joint_value = _manipulator.getAllActiveJointValue();
      return true;
    }

    ColPivHouseholderQR<MatrixXd> dec(jacobian);
    angle_changed = lambda * dec.solve(pose_changed);

    std::vector<double> set_angle_changed;
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      set_angle_changed.push_back(_manipulator.getAllActiveJointValue().at(index) + angle_changed(index));

    _manipulator.setAllActiveJointValue(set_angle_changed);
  }
  *goal_joint_value = _manipulator.getAllActiveJointValue();
  return false;
}

bool Scara::inverseSolverUsingPositionOnlySRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<double>* goal_joint_value)
{
  //manipulator
  Manipulator _manipulator = *manipulator;

  //solver parameter
  double lambda = 0.0;
  const double param = 0.002;
  const int8_t iteration = 10;

  const double gamma = 0.5;             //rollback delta

  //jacobian
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF());
  Eigen::MatrixXd position_jacobian = Eigen::MatrixXd::Identity(3, _manipulator.getDOF());
  Eigen::MatrixXd sr_jacobian = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //delta parameter
  Eigen::Vector3d position_changed = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd angle_changed = Eigen::VectorXd::Zero(_manipulator.getDOF());    //delta angle (dq)
  Eigen::VectorXd gerr(_manipulator.getDOF());

  //sr sovler parameter
  double wn_pos = 1 / 0.3;
  double pre_Ek = 0.0;
  double new_Ek = 0.0;

  Eigen::MatrixXd We(3, 3);
  We << wn_pos, 0, 0,
      0, wn_pos, 0,
      0, 0, wn_pos;

  Eigen::MatrixXd Wn = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //angle parameter
  std::vector<double> present_angle;                                               //angle (q)
  std::vector<double> set_angle;                                                   //set angle (q + dq)

  ////////////////////////////solving//////////////////////////////////

  forwardKinematics(&_manipulator);
  //////////////checking dx///////////////
  position_changed = RM_MATH::positionDifference(target_pose.position, _manipulator.getComponentPositionFromWorld(tool_name));
  pre_Ek = position_changed.transpose() * We * position_changed;
  ///////////////////////////////////////

  /////////////////////////////debug/////////////////////////////////
  #if defined(KINEMATICS_DEBUG)
  Eigen::Vector3d target_orientation_rpy = RM_MATH::convertRotationToRPY(target_pose.orientation);
  Eigen::VectorXd debug_target_pose(6);
  for(int t=0; t<3; t++)
    debug_target_pose(t) = target_pose.position(t);
  for(int t=0; t<3; t++)
    debug_target_pose(t+3) = target_orientation_rpy(t);

  Eigen::Vector3d present_position = _manipulator.getComponentPositionFromWorld(tool_name);
  Eigen::MatrixXd present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
  Eigen::Vector3d present_orientation_rpy = RM_MATH::convertRotationToRPY(present_orientation);
  Eigen::VectorXd debug_present_pose(6);
  for(int t=0; t<3; t++)
    debug_present_pose(t) = present_position(t);
  for(int t=0; t<3; t++)
    debug_present_pose(t+3) = present_orientation_rpy(t);

  RM_LOG::PRINTLN("------------------------------------");
  RM_LOG::WARN("iter : first");
  RM_LOG::WARN("Ek : ", pre_Ek*1000000000000);
  RM_LOG::PRINTLN("tar_pose");
  RM_LOG::PRINTLN_VECTOR(debug_target_pose,16);
  RM_LOG::PRINTLN("pre_pose");
  RM_LOG::PRINTLN_VECTOR(debug_present_pose,16);
  RM_LOG::PRINTLN("delta_pose");
  RM_LOG::PRINTLN_VECTOR(debug_target_pose-debug_present_pose,16);
  #endif
  ////////////////////////////debug//////////////////////////////////

  //////////////////////////solving loop///////////////////////////////
  for (int8_t count = 0; count < iteration; count++)
  {
    //////////solve using jacobian//////////
    jacobian = this->jacobian(&_manipulator, tool_name);
    position_jacobian.row(0) = jacobian.row(0);
    position_jacobian.row(1) = jacobian.row(1);
    position_jacobian.row(2) = jacobian.row(2);
    lambda = pre_Ek + param;

    sr_jacobian = (position_jacobian.transpose() * We * position_jacobian) + (lambda * Wn);     //calculate sr_jacobian (J^T*we*J + lamda*Wn)
    gerr = position_jacobian.transpose() * We * position_changed;                                //calculate gerr (J^T*we) dx

    ColPivHouseholderQR<Eigen::MatrixXd> dec(sr_jacobian);                    //solving (get dq)
    angle_changed = dec.solve(gerr);                                          //(J^T*we) * dx = (J^T*we*J + lamda*Wn) * dq

    present_angle = _manipulator.getAllActiveJointValue();
    set_angle.clear();
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      set_angle.push_back(_manipulator.getAllActiveJointValue().at(index) + angle_changed(index));
    _manipulator.setAllActiveJointValue(set_angle);
    forwardKinematics(&_manipulator);
    ////////////////////////////////////////

    //////////////checking dx///////////////
    position_changed = RM_MATH::positionDifference(target_pose.position, _manipulator.getComponentPositionFromWorld(tool_name));
    new_Ek = position_changed.transpose() * We * position_changed;
    ////////////////////////////////////////

    /////////////////////////////debug/////////////////////////////////
    #if defined(KINEMATICS_DEBUG)
    present_position = _manipulator.getComponentPositionFromWorld(tool_name);
    present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
    present_orientation_rpy = RM_MATH::convertRotationToRPY(present_orientation);
    for(int t=0; t<3; t++)
      debug_present_pose(t) = present_position(t);
    for(int t=0; t<3; t++)
      debug_present_pose(t+3) = present_orientation_rpy(t);
    RM_LOG::WARN("iter : ", count,0);
    RM_LOG::WARN("Ek : ", new_Ek*1000000000000);
    RM_LOG::PRINTLN("tar_pose");
    RM_LOG::PRINTLN_VECTOR(debug_target_pose,16);
    RM_LOG::PRINTLN("pre_pose");
    RM_LOG::PRINTLN_VECTOR(debug_present_pose,16);
    RM_LOG::PRINTLN("delta_pose");
    RM_LOG::PRINTLN_VECTOR(debug_target_pose-debug_present_pose,16);
    #endif
    ////////////////////////////debug//////////////////////////////////

    if (new_Ek < 1E-12)
    {
      /////////////////////////////debug/////////////////////////////////
      #if defined(KINEMATICS_DEBUG)
      RM_LOG::WARN("iter : ", count,0);
      RM_LOG::WARN("Ek : ", new_Ek*1000000000000);
      RM_LOG::ERROR("IK Success");
      RM_LOG::PRINTLN("------------------------------------");
      #endif
      //////////////////////////debug//////////////////////////////////
      *goal_joint_value = _manipulator.getAllActiveJointValue();
      return true;
    }
    else if (new_Ek < pre_Ek)
    {
      pre_Ek = new_Ek;
    }
    else
    {
      present_angle = _manipulator.getAllActiveJointValue();
      for (int8_t index = 0; index < _manipulator.getDOF(); index++)
        set_angle.push_back(_manipulator.getAllActiveJointValue().at(index) - (gamma * angle_changed(index)));
      _manipulator.setAllActiveJointValue(set_angle);

      forwardKinematics(&_manipulator);
    }
  }
  RM_LOG::ERROR("[position_only]fail to solve inverse kinematics (please change the solver)");
  *goal_joint_value = _manipulator.getAllActiveJointValue();
  return false;
}

bool Scara::inverseSolverUsingSRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<double>* goal_joint_value)
{
  //manipulator
  Manipulator _manipulator = *manipulator;

  //solver parameter
  double lambda = 0.0;
  const double param = 0.002;
  const int8_t iteration = 10;

  const double gamma = 0.5;             //rollback delta

  //sr sovler parameter
  double wn_pos = 1 / 0.3;
  double wn_ang = 1 / (2 * M_PI);
  double pre_Ek = 0.0;
  double new_Ek = 0.0;

  Eigen::MatrixXd We(6, 6);
  We << wn_pos, 0, 0, 0, 0, 0,
      0, wn_pos, 0, 0, 0, 0,
      0, 0, wn_pos, 0, 0, 0,
      0, 0, 0, wn_ang, 0, 0,
      0, 0, 0, 0, wn_ang, 0,
      0, 0, 0, 0, 0, wn_ang;

  Eigen::MatrixXd Wn = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //jacobian
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF());
  Eigen::MatrixXd sr_jacobian = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //delta parameter
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd angle_changed = Eigen::VectorXd::Zero(_manipulator.getDOF());    //delta angle (dq)
  Eigen::VectorXd gerr(_manipulator.getDOF());

  //angle parameter
  std::vector<double> present_angle;                                               //angle (q)
  std::vector<double> set_angle;                                                   //set angle (q + dq)

  ////////////////////////////solving//////////////////////////////////

  forwardKinematics(&_manipulator);
  //////////////checking dx///////////////
  pose_changed = RM_MATH::poseDifference(target_pose.position, _manipulator.getComponentPositionFromWorld(tool_name), target_pose.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
  pre_Ek = pose_changed.transpose() * We * pose_changed;
  ///////////////////////////////////////

  /////////////////////////////debug/////////////////////////////////
  #if defined(KINEMATICS_DEBUG)
  Eigen::Vector3d target_orientation_rpy = RM_MATH::convertRotationToRPY(target_pose.orientation);
  Eigen::VectorXd debug_target_pose(6);
  for(int t=0; t<3; t++)
    debug_target_pose(t) = target_pose.position(t);
  for(int t=0; t<3; t++)
    debug_target_pose(t+3) = target_orientation_rpy(t);

  Eigen::Vector3d present_position = _manipulator.getComponentPositionFromWorld(tool_name);
  Eigen::MatrixXd present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
  Eigen::Vector3d present_orientation_rpy = RM_MATH::convertRotationToRPY(present_orientation);
  Eigen::VectorXd debug_present_pose(6);
  for(int t=0; t<3; t++)
    debug_present_pose(t) = present_position(t);
  for(int t=0; t<3; t++)
    debug_present_pose(t+3) = present_orientation_rpy(t);

  RM_LOG::PRINTLN("------------------------------------");
  RM_LOG::WARN("iter : first");
  RM_LOG::WARN("Ek : ", pre_Ek*1000000000000);
  RM_LOG::PRINTLN("tar_pose");
  RM_LOG::PRINTLN_VECTOR(debug_target_pose,16);
  RM_LOG::PRINTLN("pre_pose");
  RM_LOG::PRINTLN_VECTOR(debug_present_pose,16);
  RM_LOG::PRINTLN("delta_pose");
  RM_LOG::PRINTLN_VECTOR(debug_target_pose-debug_present_pose,16);
  #endif
  ////////////////////////////debug//////////////////////////////////

  //////////////////////////solving loop///////////////////////////////
  for (int8_t count = 0; count < iteration; count++)
  {
    //////////solve using jacobian//////////
    jacobian = this->jacobian(&_manipulator, tool_name);
    lambda = pre_Ek + param;

    sr_jacobian = (jacobian.transpose() * We * jacobian) + (lambda * Wn);     //calculate sr_jacobian (J^T*we*J + lamda*Wn)
    gerr = jacobian.transpose() * We * pose_changed;                          //calculate gerr (J^T*we) dx

    ColPivHouseholderQR<Eigen::MatrixXd> dec(sr_jacobian);                    //solving (get dq)
    angle_changed = dec.solve(gerr);                                          //(J^T*we) * dx = (J^T*we*J + lamda*Wn) * dq

    present_angle = _manipulator.getAllActiveJointValue();
    set_angle.clear();
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      set_angle.push_back(present_angle.at(index) + angle_changed(index));
    _manipulator.setAllActiveJointValue(set_angle);
    forwardKinematics(&_manipulator);
    ////////////////////////////////////////

    //////////////checking dx///////////////
    pose_changed = RM_MATH::poseDifference(target_pose.position, _manipulator.getComponentPositionFromWorld(tool_name), target_pose.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
    new_Ek = pose_changed.transpose() * We * pose_changed;
    ////////////////////////////////////////

    /////////////////////////////debug/////////////////////////////////
    #if defined(KINEMATICS_DEBUG)
    present_position = _manipulator.getComponentPositionFromWorld(tool_name);
    present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
    present_orientation_rpy = RM_MATH::convertRotationToRPY(present_orientation);
    for(int t=0; t<3; t++)
      debug_present_pose(t) = present_position(t);
    for(int t=0; t<3; t++)
      debug_present_pose(t+3) = present_orientation_rpy(t);
    RM_LOG::WARN("iter : ", count,0);
    RM_LOG::WARN("Ek : ", new_Ek*1000000000000);
    RM_LOG::PRINTLN("tar_pose");
    RM_LOG::PRINTLN_VECTOR(debug_target_pose,16);
    RM_LOG::PRINTLN("pre_pose");
    RM_LOG::PRINTLN_VECTOR(debug_present_pose,16);
    RM_LOG::PRINTLN("delta_pose");
    RM_LOG::PRINTLN_VECTOR(debug_target_pose-debug_present_pose,16);
    #endif
    ////////////////////////////debug//////////////////////////////////

    if (new_Ek < 1E-12)
    {
      /////////////////////////////debug/////////////////////////////////
      #if defined(KINEMATICS_DEBUG)
      RM_LOG::WARN("iter : ", count,0);
      RM_LOG::WARN("Ek : ", new_Ek*1000000000000);
      RM_LOG::ERROR("Success");
      RM_LOG::PRINTLN("------------------------------------");
      #endif
      //////////////////////////debug//////////////////////////////////
      *goal_joint_value = _manipulator.getAllActiveJointValue();
      return true;
    }
    else if (new_Ek < pre_Ek)
    {
      pre_Ek = new_Ek;
    }
    else
    {
      present_angle = _manipulator.getAllActiveJointValue();
      for (int8_t index = 0; index < _manipulator.getDOF(); index++)
        set_angle.push_back(present_angle.at(index) - (gamma * angle_changed(index)));
      _manipulator.setAllActiveJointValue(set_angle);

      forwardKinematics(&_manipulator);
    }
  }
  RM_LOG::ERROR("[sr]fail to solve inverse kinematics (please change the solver)");
  *goal_joint_value = _manipulator.getAllActiveJointValue();   // <--- why get values like this..??
  return false;
}

bool Scara::inverseKinematicsSolverUsingGeometry(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<double>* goal_joint_value)
{
  double link[3] = {0.067, 0.067, 0.107};
  double target_angle[3];
  std::vector<double> target_angle_vector;

  // Compute the length from Joint1 to the end effector
  double temp_target_pose[2];
  double target_pose_length;
  temp_target_pose[0] = target_pose.position(0) + 0.241;
  temp_target_pose[1] = target_pose.position(1);
  target_pose_length = sqrt((temp_target_pose[0])*temp_target_pose[0] + temp_target_pose[1]*temp_target_pose[1]);

  // Compute the length of Position Difference and Target Angle
  double error=1000.0; // random large initial value

  for (uint16_t count=0; count<=900; count++){  
    double theta=(double)count/10*DEG2RAD;

    // Assume theta = target_angle[1] = target_angle[2]
    double alpha = acos((link[1]*link[1]+link[2]*link[2]-link[0]*link[0]-target_pose_length*target_pose_length+2*link[1]*link[2]*cos(theta)) 
                  / (-2*target_pose_length*link[0]));
    double beta = acos((link[0]*link[0]+link[1]*link[1]-link[2]*link[2]-target_pose_length*target_pose_length+2*link[0]*link[1]*cos(theta)) 
                  / (-2*target_pose_length*link[2]));
    double temp_error = abs(alpha + beta - 2*theta); 

    if (temp_error < error){
      target_angle[0] = PI/2 -acos(temp_target_pose[1]/target_pose_length) - alpha;
      target_angle[1] = theta;
      target_angle[2] = theta;
      error = temp_error;
    }
  }

  // Set joint angle 
  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(target_angle[1]);
  target_angle_vector.push_back(target_angle[2]);

  *goal_joint_value = target_angle_vector;

  return true;
}
