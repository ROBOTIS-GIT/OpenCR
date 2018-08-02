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

/* Authors: Hye-Jong KIM, Darby Lim*/

#include "../../../include/open_manipulator/OMManager.h"


///////////////////////////*initialize function*/////////////////////////////
void Manipulator::addWorld(Name world_name, NAME child_name, Vector3f world_position = Vector3f::Zero(), Matrix3f world_orientation = Matrix3f::Identity(3,3), bool *error = false)
{
  world_.name_ = world_name;
  world_.child_ = child_name;
  world_.pose_.position = world_position;
  world_.pose_.orientation = world_orientation;
  world_.origin_.velocity = 0.0;
  world_.origin_.acceleration = 0.0;
}

void Manipulator::addComponent(Name me_name, Name parent_name, NAME child_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t actuator_id = -1, 
                  Vector3f axis_of_rotation = Vector3f::Zero(), float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero(), bool *error = false)
{

}

void Manipulator::addComponentChild(Name me_name, NAME child_name, bool *error = false)
{

}

void Manipulator::addTool(Name me_name, Name parent_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t tool_id = -1,
              float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero())
{
  
}

void Manipulator::checkManipulatorSetting(bool *error = false)
{

}
/////////////////////////////////////////////////////////////////////////////

///////////////////////////////Set function//////////////////////////////////
void Manipulator::setWorldPose(Pose world_pose, bool *error = false)
{

}

void Manipulator::setWorldPosition(Vector3f world_position, bool *error = false)
{

}

void Manipulator::setWorldOrientation(Matrix3f world_orientation, bool *error = false)
{

}

void Manipulator::setWorldState(State world_state, bool *error = false)
{

}

void Manipulator::setWorldVelocity(Vector6f world_velocity, bool *error = false)
{

}

void Manipulator::setWorldAcceleration(Vector6f world_acceleration, bool *error = false)
{

}

void Manipulator::setComponent(Name name, Component component, bool *error = false)
{

}

void Manipulator::setComponentPoseToWorld(Name name Pose pose_to_world, bool *error = false)
{

}

void Manipulator::setComponentPositionToWorld(Name name, Vector3f position_to_world, bool *error = false)
{

}

void Manipulator::setComponentOrientationToWorld(Name name, Matrix3f orientation_to_wolrd, bool *error = false)
{

}

void Manipulator::setComponentStateToWorld(Name name State state_to_world, bool *error = false)
{

}

void Manipulator::setComponentVelocityToWorld(Name name, Vector6f velocity, bool *error = false)
{

}

void Manipulator::setComponentAccelerationToWorld(Name name, Vector6f accelaration, bool *error = false)
{

}

void Manipulator::setComponentJointAngle(Name name, float angle, bool *error = false)
{

}

void Manipulator::setComponentJointVelocity(Name name, float angular_velocity, bool *error = false)
{

}

void Manipulator::setComponentJointAcceleration(Name name, float angular_acceleration, bool *error = false)
{

}

void Manipulator::setComponentToolOnOff(Name name, bool on_off, bool *error = false)
{

}

void Manipulator::setComponentToolValue(Name name, float actuator_value, bool *error = false)
{

}

/////////////////////////////////////////////////////////////////////////////

///////////////////////////////Get function//////////////////////////////////
int Manipulator::getDOF(bool *error = false)
{

}

int8_t Manipulator::getComponentSize(bool *error = false)
{

}

Name Manipulator::getWorldName(bool *error = false)
{

}

Name Manipulator::getWorldChildName(bool *error = false)
{

}

Pose Manipulator::getWorldPose(bool *error = false)
{

}

Vector3f Manipulator::getWorldPosition(bool *error = false)
{

}

Matrix3f Manipulator::getWorldOrientation(bool *error = false)
{

}

State Manipulator::getWorldState(bool *error = false)
{

}

Vector6f Manipulator::getWorldVelocity(bool *error = false)
{

}

Vector6f Manipulator::getWorldAcceleration(bool *error = false)
{

}

map<Name, Component> Manipulator::getAllComponent(bool *error = false)
{

}

Component Manipulator::getComponent(Name name, bool *error = false)
{

}

Name Manipulator::getComponentParentName(Name name, bool *error = false)
{

}

vector<NAME> Manipulator::getComponentChildName(Name name, bool *error = false)
{

}

Pose Manipulator::getComponentPoseToWorld(Name name, bool *error = false)
{

}

Vector3f Manipulator::getComponentPositionToWorld(Name name, bool *error = false)
{

}

Matrix3f Manipulator::getComponentOrientationToWorld(Name name, bool *error = false)
{

}

State Manipulator::getComponentStateToWorld(Name name, bool *error = false)
{

}

Vector6f Manipulator::getComponentVelocityToWorld(Name name, bool *error = false)
{

}

Vector6f Manipulator::getComponentAccelerationToWorld(Name name, bool *error = false)
{

}

Pose Manipulator::getComponentRelativePoseToParent(Name name, bool *error = false)
{

}

Vector3f Manipulator::getComponentRelativePositionToParent(Name name, bool *error = false)
{

}

Matrix3f Manipulator::getComponentRelativeOrientationToParent(Name name, bool *error = false)
{

}

Joint Manipulator::getComponentJoint(Name name, bool *error = false)
{

}

int8_t Manipulator::getComponentJointId(Name name, bool *error = false)
{

}

Vector3f Manipulator::getComponentJointAxis(Name name, bool *error = false)
{

} 
 
float Manipulator::getComponentJointAngle(Name name, bool *error = false)
{

}

float Manipulator::getComponentJointVelocity(Name name, bool *error = false)
{

}

float Manipulator::getComponentJointAcceleration(Name name, bool *error = false)
{

}

Tool Manipulator::getComponentTool(Name name, bool *error = false)
{

}

int8_t Manipulator::getComponentToolId(Name name, bool *error = false)
{

}

bool Manipulator::getComponentToolOnOff(Name name, bool *error = false)
{

}

float Manipulator::getComponentToolValue(Name name, bool *error = false)
{

}

float Manipulator::getComponentMass(Name name, bool *error = false)
{

}

Matrix3f Manipulator::getComponentInertiaTensor(Name name, bool *error = false)
{

}

Pose Manipulator::getComponentCenterOfMassPose(Name name, bool *error = false)
{

}

Vector3f Manipulator::getComponentCenterOfMassPosition(Name name, bool *error = false)
{

}

Matrix3f Manipulator::getComponentCenterOfMassOrientation(Name name, bool *error = false)
{

}