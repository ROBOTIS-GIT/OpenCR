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

#include "../include/open_manipulator_libs/Scara.h"

Scara::Scara()
{}
Scara::~Scara()
{}

void Scara::initDebug()
{
  DEBUG.begin(57600); // Using Serial4(=SerialBT2)

  RM_LOG::PRINT("OpenManipulator Debugging Port"); 
}

void Scara::initManipulator(bool hardware_flag, STRING usb_port, STRING baud_rate)
{
  // ??
  Serial.print("Initial actuator angles: ");
  for (int i = 0; i < DXL_SIZE; i++)
  {
    Serial.print(0.0);
    Serial.print(",");
  }
  Serial.println("");
  delay(300);             

  // ??
  hardware_flag_ = hardware_flag;  


  //--------------------------------------------------------------------
  // Manipulator Parameters Initialization
  //--------------------------------------------------------------------
  addWorld("world",   // world name
           "joint1"); // child name

  addJoint("joint1",  // my name
           "world",   // parent name
           "joint2",  // child name
           RM_MATH::makeVector3(-0.241, 0.0, 0.057),     // relative position
           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
           Z_AXIS,    // axis of rotation
           1);        // actuator id

  addJoint("joint2", // my name
           "joint1", // parent name
           "joint3", // child name
           RM_MATH::makeVector3(0.067, 0.0, 0.0),        // relative position
           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
           Z_AXIS, // axis of rotation
           2);     // actuator id

  addJoint("joint3", // my name
           "joint2", // parent name
           "tool",   // child name
           RM_MATH::makeVector3(0.067, 0.0, 0.0),        // relative position
           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
           Z_AXIS,  // axis of rotation
           3);      // actuator id

  addTool("tool",   // my name
          "joint3", // parent name
          RM_MATH::makeVector3(0.107, 0.0, 0.0),        // relative position
          RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
          4,        // actuator id
          1.0);     // Change unit from `meter` to `radian`  


  //--------------------------------------------------------------------
  // Kinematics Initialization 
  //--------------------------------------------------------------------

  kinematics_ = new SCARA_KINEMATICS::Scara();
  addKinematics(kinematics_);
  STRING inverse_option[2] = {"inverse_solver", "geometric_inverse"};
                                                                     // STRING to uint8_t
                                                                     // 0: ~~
                                                                     // 1: ~~           
                                                                     // 2: ~~           
  void *inverse_option_arg = &inverse_option;
  kinematicsSetOption(inverse_option_arg); 

  if(hardware_flag_)
  {
    //--------------------------------------------------------------------
    // Joint Initialization
    //--------------------------------------------------------------------

    // Initialize(?)joint actuator 
    joint_ = new SCARA_DYNAMIXEL::JointDynamixel();

    // Set communication arguments 
    STRING dxl_comm_arg[2] = {usb_port, baud_rate};
    void *p_dxl_comm_arg = &dxl_comm_arg; 

    // Set joint actuator id 
    jointDxlId.push_back(1);
    jointDxlId.push_back(2);
    jointDxlId.push_back(3);
    addJointActuator(JOINT_DYNAMIXEL, joint_, jointDxlId, p_dxl_comm_arg);

    // set joint actuator parameter
    STRING joint_dxl_opt_arg[2] = {"Return_Delay_Time", "0"};
    void *p_joint_dxl_opt_arg = &joint_dxl_opt_arg;
    jointActuatorSetMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_opt_arg);

    // set joint actuator control mode
    STRING joint_dxl_mode_arg = "position_mode";
    void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
    jointActuatorSetMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);


    //--------------------------------------------------------------------
    // Tool Initialization
    //--------------------------------------------------------------------

    // Initialize tool actuator 
    tool_ = new SCARA_DYNAMIXEL::ToolDynamixel();

    // Set tool actuator id 
    uint8_t gripperDxlId = 4;
    addToolActuator(TOOL_DYNAMIXEL, tool_, gripperDxlId, p_dxl_comm_arg);

    // Set gripper actuator parameter 
    STRING gripper_dxl_opt_arg[2] = {"Return_Delay_Time", "0"};
    void *p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
    toolActuatorSetMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

    // Set gripper actuator control mode 
    STRING gripper_dxl_mode_arg = "position_mode";
    void *p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
    toolActuatorSetMode(TOOL_DYNAMIXEL, p_gripper_dxl_mode_arg);


    // Eanble all actuators 
    allActuatorEnable();
    
    // Receive(->read??) all joint actuators values 
    receiveAllJointActuatorValue();

    // Receive(->read??) all tool actuators values 
    receiveAllToolActuatorValue();
  }

  //--------------------------------------------------------------------
  // Drawing Initialization 
  //--------------------------------------------------------------------

  // Add drawing trajectories 
  addDrawingTrajectory(DRAWING_LINE, &line_);
  addDrawingTrajectory(DRAWING_CIRCLE, &circle_);
  addDrawingTrajectory(DRAWING_RHOMBUS, &rhombus_);
  addDrawingTrajectory(DRAWING_HEART, &heart_);

  // manipulator trajectory & control time initialization 
  setTrajectoryControlTime(CONTROL_RATE);
}

/* Execute output data received from external controllers */
void Scara::executeData()  
{
  double curr_time = millis()/1000.0;

  if (curr_time - prev_control_time_ >= CONTROL_RATE)  
  {
    std::vector<WayPoint> goal_value = getJointGoalValueFromTrajectory(curr_time);
    std::vector<double> tool_value = getToolGoalValue();

    if (hardware_flag_)
    {   
      receiveAllJointActuatorValue();   // <-- for what?
      receiveAllToolActuatorValue();    // <-- for what?  
      if(goal_value.size() != 0) sendAllJointActuatorValue(goal_value); // <-- for what?  
      if(tool_value.size() != 0) sendAllToolActuatorValue(tool_value);  // <-- for what?  
    }
    else
    {
      if(goal_value.size() != 0) setAllActiveJointWayPoint(goal_value); // visualization   // <-- for what?  
      if(tool_value.size() != 0) setAllToolValue(tool_value);
    }

    forwardKinematics(); 
    prev_control_time_ = millis()/1000.0; // instead of curr_time...?
  }
}

/* Check if hardwares are used or not) */
bool Scara::getHardwareFlag() 
{
  return hardware_flag_;
}

/* Check if the program read data within control rate) */
bool Scara::getReceiveDataFlag()  
{
  return receive_data_flag_;
}

/* Get the previous time when data were received */
bool Scara::getConsecutiveMotionFlag() 
{
  return consecutive_motion_flag_;
}

/* Get the previous time when data were received */
double Scara::getPrevReceiveTime() 
{
  return prev_receive_time_;
}

/* Get the previous time when data were received */
int Scara::getOffsetPositionNum() 
{
  return offset_position_num_;
}

/* Set whether data were received or not */
void Scara::setReceiveDataFlag(bool receive_data_flag) 
{
  receive_data_flag_ = receive_data_flag;
}

/* Set whether data were received or not */
void Scara::setConsecutiveMotionFlag(bool consecutive_motion_flag) 
{
  consecutive_motion_flag_ = consecutive_motion_flag;
}

/* Set the previous time when data were received */
void Scara::setPrevReceiveTime(double prev_receive_time)  
{
  prev_receive_time_ = prev_receive_time;
}

/* Set the blahblah */
void Scara::setOffsetPositionNum(int offset_position_num)  
{
  offset_position_num_ = offset_position_num;
}
