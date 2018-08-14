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

#include <OMAPI.h>
#include <OMManager.h>
#include <OMMath.h>
#include <OMKinematics.h>
#include <OMDebug.h>
#include <OMBridge.h>
#include <OMDynamixel.h>
#include <OMPath.h>

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

///////////move/////////////////////

#define CHECK_FLAG   0
#define WAIT_FOR_SEC 1

#define PROCESSINGON true
#define DYNAMIXEL  true
#define TORQUE     true

#define MAX_MOTION_NUM 20

#define DXL_SIZE 3
#define BAUD_RATE 1000000

#define CONTROL_PERIOD 0.010f
#define MOVE_TIME 1.0f

bool motion    = false;
bool repeat    = false;

bool moving    = false;
bool send_processing_flug = true ;

float move_time;

String cmd[20];

float motion_storage[MAX_MOTION_NUM][5] = {0.0, };
uint8_t motion_cnt = 0;
uint8_t filled_motion_num = 0;


const float initial_motion_set[MAX_MOTION_NUM][5] = { // time, grip, joint1, joint2, joint3,
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},  
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0},
                                                { 0.0,  0.0,   0.0,  0.0,  0.0}
                                                };

OM_PATH::JointTrajectory joint_trajectory(DXL_SIZE);
std::vector<Trajectory> start_trajectory_vector;
std::vector<Trajectory> goal_trajectory_vector;

//////////////////////////////////////////

OM_MANAGER::Manipulator omlink;
OM_DYNAMIXEL::Dynamixel dxl(BAUD_RATE);


namespace MyFunction
{
  void setPassiveJointAngle()
  {
    float joint_angle[3];
    joint_angle[0] = omlink.getComponentJointAngle(JOINT0);
    joint_angle[1] = omlink.getComponentJointAngle(JOINT1);
    joint_angle[2] = omlink.getComponentJointAngle(JOINT2);

    omlink.setComponentJointAngle(JOINT3, (joint_angle[1] - joint_angle[2]));
    omlink.setComponentJointAngle(JOINT4, -M_PI - (joint_angle[1] - joint_angle[2]));
    omlink.setComponentJointAngle(JOINT5, -M_PI - (joint_angle[1] - joint_angle[2]));
    omlink.setComponentJointAngle(JOINT6, -M_PI - joint_angle[2]);
    omlink.setComponentJointAngle(JOINT7, joint_angle[1]);
    omlink.setComponentJointAngle(JOINT8, -(15 * DEG2RAD) - joint_angle[1]);
    omlink.setComponentJointAngle(JOINT9, joint_angle[2] - (195 * DEG2RAD));
    omlink.setComponentJointAngle(JOINT10, (90 * DEG2RAD) - joint_angle[2]);
  }

  Trajectory makeSingleJointTrajectory(float angle, float velocity = 0.0, float acceleration = 0.0)
  {
    Trajectory trajectory;
    trajectory.position = angle;
    trajectory.velocity = velocity;
    trajectory.acceleration = acceleration;
    return trajectory;
  }

  std::vector <Trajectory> makeTrajectoryVector(std::vector <float> angle_vector, std::vector <float> velocity_vector, std::vector <float> acceleration_vector)
  {
    std::vector <Trajectory> trajectory_vector;
    for(uint8_t i = 0; i < angle_vector.size(); i++)
      trajectory_vector.push_back(makeSingleJointTrajectory(angle_vector.at(i), velocity_vector.at(i), acceleration_vector.at(i)));

    return trajectory_vector;
  }

  std::vector <Trajectory> makeTrajectoryVector(std::vector <float> angle_vector)
  {
    std::vector <Trajectory> trajectory_vector;
    for(uint8_t i = 0; i < angle_vector.size(); i++)
      trajectory_vector.push_back(makeSingleJointTrajectory(angle_vector.at(i), 0.0, 0.0));

    return trajectory_vector;
  }

  void updateJointTrajectory(std::vector <float> target_angle, float inner_move_time)
  {
    std::vector <float> start_angle;
    start_angle = OPEN_MANIPULATOR::MANAGER::getAllActiveJointAngle(&omlink);
    start_trajectory_vector = makeTrajectoryVector(start_angle);
    goal_trajectory_vector = makeTrajectoryVector(target_angle);

    joint_trajectory.init(start_trajectory_vector, goal_trajectory_vector, inner_move_time, CONTROL_PERIOD);
    move_time = inner_move_time;
    moving   = true;
  }

  Pose setPose(String dir)
  {
    Pose target_pose;
    float step = 0.02;

    target_pose = omlink.getComponentPoseToWorld(SUCTION);

    if (dir == "forward")
    {
      target_pose.position(0) += step;
    }  
    else if (dir == "back")
    {
      target_pose.position(0) -= step;
    }
    else if (dir == "left")
    {
      target_pose.position(1) += step;
    }
    else if (dir == "right")
    {
      target_pose.position(1) -= step;
    }
    else if (dir == "up")
    {
      target_pose.position(2) += step;
    }
    else if (dir == "down")
    {
      target_pose.position(2) -= step;
    }

    DEBUG.print(target_pose.position(0));
    DEBUG.print(" , ");
    DEBUG.print(target_pose.position(1));
    DEBUG.print(" , ");
    DEBUG.print(target_pose.position(2));
    DEBUG.println(" ");

    return target_pose;
  }

  void dataFromProcessing(String get)
  {
    get.trim();
    OM_PROCESSING::split(get, ',', cmd);
    DEBUG.print(cmd[0]);
    DEBUG.print(",");
    DEBUG.print(cmd[1]);
    DEBUG.print(" / ");
    DEBUG.print("do : ");

    if (cmd[0] == "om")
    {
      DEBUG.print("om-");
      if (cmd[1] == "ready")
      {
        DEBUG.print("ready");
        DEBUG.println(" ");
        if (DYNAMIXEL)
        {
          dxl.enableAllDynamixel();
        }

        if (PROCESSINGON)
          send_processing_flug = true ;
          //PROCESSING::sendAngle2Processing(OPEN_MANIPULATOR::MANAGER::getAllJointAngle(&omlink)); 
      }
      else if (cmd[1] == "end")
      {
        DEBUG.print("end");
        DEBUG.println(" ");
        send_processing_flug = false ;
        if (DYNAMIXEL)
          dxl.disableAllDynamixel();
      }
    }
    else if (cmd[0] == "joint")
    {
      DEBUG.print("joint-");
      std::vector<float> target_angle;

      for (uint8_t i = 1; i < 4; i++)
        target_angle.push_back(cmd[i].toFloat());
      updateJointTrajectory(target_angle, MOVE_TIME);
      DEBUG.print(target_angle.at(0));
      DEBUG.print(" , ");
      DEBUG.print(target_angle.at(1));
      DEBUG.print(" , ");
      DEBUG.print(target_angle.at(2));
      DEBUG.println(" ");
    }
    else if (cmd[0] == "suction")
    {
      DEBUG.print("suction-");
      if (cmd[1] == "on")
      {
        DEBUG.print("on");
        DEBUG.println(" ");
      }//suction on
        
      else if (cmd[1] == "off")
      {
        DEBUG.print("off");
        DEBUG.println(" ");
      }//suction off
        
      else{}
    }
    else if (cmd[0] == "task")
    {
      DEBUG.print("task-");
      Pose target_pose;
      std::vector<float> target_angle;

      target_pose = setPose(cmd[1]);
      target_angle = OM_KINEMATICS::LINK::geometricInverse(&omlink, SUCTION, target_pose);

      updateJointTrajectory(target_angle, MOVE_TIME);
      DEBUG.println(" ");
    }
    else if (cmd[0] == "motor")
    {
      DEBUG.print("motor-");
      if (DYNAMIXEL)
      {
        if (cmd[1] == "enable")
        {
          DEBUG.print("enable");
          DEBUG.println(" ");
          dxl.enableAllDynamixel();
        }
        else if (cmd[1] == "disable")
        {
          DEBUG.print("disable");
          DEBUG.println(" ");
          dxl.disableAllDynamixel();
        }
      }
    }
    else if (cmd[0] == "get")
    {
      DEBUG.print("get-");
      if (cmd[1] == "on")
      {
        DEBUG.print("suction on");
        DEBUG.println(" ");
        motion_storage[filled_motion_num][0] = MOVE_TIME;  //mov_time
        motion_storage[filled_motion_num][4] = -1.0;
        for (uint8_t j = 1; j < 4; j++)
        {
          motion_storage[filled_motion_num][j] = 0.0;
        }
        filled_motion_num++;
      }
      else if (cmd[1] == "off")
      {
        DEBUG.print("suction off");
        DEBUG.println(" ");
        motion_storage[filled_motion_num][0] = MOVE_TIME;  //mov_time
        motion_storage[filled_motion_num][4] = 1.0;
        for (uint8_t j = 1; j < 4; j++)
        {
          motion_storage[filled_motion_num][j] = 0.0;
        }
        filled_motion_num++;
      }
      else if (cmd[1] == "clear")
      {
        DEBUG.print("clear");
        DEBUG.println(" ");
        for (uint8_t i = 0; i < MAX_MOTION_NUM; i++)
        {
          for (uint8_t j = 0; j < 5; j++)
          {
            motion_storage[i][j] = 0.0;
          }
        }
        
        filled_motion_num = 0;
        motion_cnt = 0;
        motion     = false;
        repeat     = false;
      }
      else if (cmd[1] == "pose")
      {
        DEBUG.print("pose");
        DEBUG.println(" ");
        if (cmd[2].toInt() < MAX_MOTION_NUM)
        {
          std::vector<float> target_angle = OPEN_MANIPULATOR::MANAGER::getAllActiveJointAngle(&omlink);

          motion_storage[filled_motion_num][0]   = MOVE_TIME;  //mov_time
          for (uint8_t i = 0; i < 3; i++)
          {
            motion_storage[filled_motion_num][i+1] = target_angle.at(i);
          }
          motion_storage[filled_motion_num][4] = 0.0;
          filled_motion_num++;
        }
      }
    }
    else if (cmd[0] == "hand")
    {
      DEBUG.print("hand-");
      if (cmd[1] == "once")
      {
        DEBUG.print("once");
        DEBUG.println(" ");
        if (DYNAMIXEL)
        {
          dxl.enableAllDynamixel();
          //PROCESSING::sendAngle2Processing(OPEN_MANIPULATOR::MANAGER::getAllJointAngle(&omlink)); 
        }

        motion_cnt = 0;
        motion = true;
        repeat = false;
      }
      else if (cmd[1] == "repeat")
      {
        DEBUG.print("repeat");
        DEBUG.println(" ");
        if (DYNAMIXEL)
        {
          dxl.enableAllDynamixel();
          //PROCESSING::sendAngle2Processing(OPEN_MANIPULATOR::MANAGER::getAllJointAngle(&omlink)); 
        }

        motion_cnt = 0;
        motion = true;
        repeat = true;
      }
      else if (cmd[1] == "stop")
      {
        DEBUG.print("stop");
        DEBUG.println(" ");
        for (uint8_t i = 0; i < MAX_MOTION_NUM; i++)
        {
          for (uint8_t j = 0; j < 5; j++)
          {
            motion_storage[i][j] = 0.0;
          }
        }
        motion_cnt = 0;
        motion     = false;
        repeat     = false;
      }
    }
    else if (cmd[0] == "motion")
    {
      DEBUG.print("motion-");
      if (cmd[1] == "start")
      {
        DEBUG.print("start");
        DEBUG.println(" ");
        for (uint8_t i = 0; i < MAX_MOTION_NUM; i++)
        {
          for (uint8_t j = 0; j < 5; j++)
          {
            motion_storage[i][j] = initial_motion_set[i][j];
          }
        }

        filled_motion_num = MAX_MOTION_NUM;  
        motion_cnt = 0;          
        motion = true;
        repeat = true;
      }
      else if (cmd[1] == "stop")
      {
        DEBUG.print("stop");
        DEBUG.println(" ");
        for (uint8_t i = 0; i < MAX_MOTION_NUM; i++)
        {
          for (uint8_t j = 0; j < 5; j++)
          {
            motion_storage[i][j] = 0.0;
          }
        }
        motion_cnt = 0;
        motion     = false;
        repeat     = false;
      }
    }
  }

  void getData(uint32_t wait_time)
  {
    static uint8_t state = 0;
    static uint32_t tick = 0;

    static bool processing_flag = false;
    String get_processing_data = "";

    if (Serial.available())
    {
      get_processing_data = Serial.readStringUntil('\n');
      processing_flag = true;
    }

    if(processing_flag)
    {
      switch (state)
      {
        case CHECK_FLAG:
            DEBUG.print("data : ");
            dataFromProcessing(get_processing_data);   
            tick = millis();
            state = WAIT_FOR_SEC;
        break;
        
        case WAIT_FOR_SEC:
          if ((millis() - tick) >= wait_time)
          {
            processing_flag = false;
            state = CHECK_FLAG;
          }
        break;
        
        default :
        state = CHECK_FLAG;
        break;
      }
    }
  }

  void setMotion()
  {
    if (motion)
    {
      if (moving)
      {
        return;
      }
      else
      {
        //repeat check
        if (motion_cnt >= filled_motion_num)
        {
          if (repeat)
          {
            motion_cnt = 0;
          }
          else
          {
            motion_cnt = 0;
            motion     = false;     
          }
        }

        //motion make
        if (motion_storage[motion_cnt][4] == -1.0)
        {
          //suction off
          motion_cnt++;
        }
        else if (motion_storage[motion_cnt][4] == 1.0)
        {
          //suction on
          motion_cnt++;
        }
        else
        {
          std::vector <float> target_angle;
          for (int8_t i = 0; i < 3; i++)
          {
            target_angle.push_back(motion_storage[motion_cnt][i+1]);
          }
          updateJointTrajectory(target_angle, motion_storage[motion_cnt][0]);
          motion_cnt++;
        }
      }
    }
    else
    {
      motion_cnt = 0;
    }
  }

  void jointMove(float present_time)
  {
    static float start_time = 0.0;
    float tick_time;

    std::vector<float> goal_position;
    std::vector<float> goal_velocity;
    std::vector<float> goal_acceleration;

    std::vector<float> dxl_goal_angle;

    if(moving)
    {
      tick_time = present_time - start_time;
      if(tick_time < move_time)
      {
        goal_position = joint_trajectory.getPosition(tick_time);
        goal_velocity = joint_trajectory.getVelocity(tick_time);
        goal_acceleration = joint_trajectory.getAcceleration(tick_time);  

        dxl_goal_angle.push_back(goal_position.at(0));
        dxl_goal_angle.push_back(-goal_position.at(1));
        dxl_goal_angle.push_back(goal_position.at(2));
        dxl.setAngle(dxl_goal_angle);
        dxl_goal_angle.clear();

        // dxl.setAngle(goal_position);
        moving   = true; 
      }
      else
      {
        goal_position = joint_trajectory.getPosition(move_time);
        goal_velocity = joint_trajectory.getVelocity(move_time);
        goal_acceleration = joint_trajectory.getAcceleration(move_time);
        
        dxl_goal_angle.push_back(goal_position.at(0));
        dxl_goal_angle.push_back(-goal_position.at(1));
        dxl_goal_angle.push_back(goal_position.at(2));
        dxl.setAngle(dxl_goal_angle);
        dxl_goal_angle.clear();
        
        // dxl.setAngle(goal_position);
        moving   = false; 
        start_time = present_time;
      }
    }
    else
    {
      start_time = present_time;
    }


  //   static uint16_t step_cnt = 0;
  //   uint16_t step = uint16_t(floor(MOVE_TIME/CONTROL_PERIOD) + 1.0);
  //   float tick_time = 0.0;

  //   std::vector<float> goal_position;
  //   std::vector<float> goal_velocity;
  //   std::vector<float> goal_acceleration;
  //   if(moving)
  //   {
  //     if (step_cnt < step)
  //     {
  //       tick_time = CONTROL_PERIOD * step_cnt;

  //       goal_position = joint_trajectory.getPosition(tick_time);
  //       goal_velocity = joint_trajectory.getVelocity(tick_time);
  //       goal_acceleration = joint_trajectory.getAcceleration(tick_time);
            
  //       USB.print(goal_position.at(0));
  //       USB.print(" , ");
  //       USB.print(goal_position.at(1));
  //       USB.print(" , ");
  //       USB.print(goal_position.at(2));
  //       USB.print(" /      tick time : ");
  //       USB.print(tick_time);
        
  //       dxl.setAngle(goal_position);
  //       step_cnt++;
  //       moving   = true; 
  //     }
  //     else if(step_cnt < step+1)
  //     {
  //       tick_time = CONTROL_PERIOD * step_cnt;
  //       //updateJointTrajectory(OPEN_MANIPULATOR::MANAGER::getAllActiveJointAngle(&omlink), MOVE_TIME);
  //       step_cnt++;
  //       moving   = true; 
  //     }
  //     else
  //     {
  //       step_cnt = 0;
  //       moving   = false;
  //     } 
  //   }
  }
}// MYfunction

void initOMLink()
{
  //init manipulator
  omlink.addWorld(WORLD, BASE);
  omlink.addComponent(BASE, WORLD, JOINT0, OM_MATH::makeVector3(-0.23867882, 0, 0), Matrix3f::Identity(3,3));
  omlink.addComponent(JOINT0, BASE, JOINT1, Vector3f::Zero(), Matrix3f::Identity(3,3), OM_MATH::makeVector3(0,0,1), 1, -1);
  omlink.addComponentChild(JOINT0, JOINT2);
  omlink.addComponentChild(JOINT0, JOINT7);
  omlink.addComponent(JOINT1, JOINT0, JOINT5, OM_MATH::makeVector3(0, 0.022, 0.052), Matrix3f::Identity(3,3), OM_MATH::makeVector3(0,1,0), 2, 1);
  omlink.addComponent(JOINT2, JOINT0, JOINT3, OM_MATH::makeVector3(0, -0.022, 0.052), Matrix3f::Identity(3,3), OM_MATH::makeVector3(0,1,0), 3, -1);
  omlink.addComponent(JOINT3, JOINT2, JOINT4, OM_MATH::makeVector3(0.050, 0.007, 0), Matrix3f::Identity(3,3), OM_MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT4, JOINT3, JOINT5, OM_MATH::makeVector3(0.200, 0.006, 0), Matrix3f::Identity(3,3), OM_MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT5, JOINT1, JOINT6, OM_MATH::makeVector3(0.200, -0.016, 0), Matrix3f::Identity(3,3), OM_MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT6, JOINT5, SUCTION, OM_MATH::makeVector3(0.200, -0.009, 0), Matrix3f::Identity(3,3), OM_MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT7, JOINT0, JOINT8, OM_MATH::makeVector3(-0.04531539, 0.006, 0.07313091), Matrix3f::Identity(3,3), OM_MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT8, JOINT7, JOINT9, OM_MATH::makeVector3(0.200, 0.009, 0), Matrix3f::Identity(3,3), OM_MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT9, JOINT8, JOINT10, OM_MATH::makeVector3(0.07660444, -0.006, 0), Matrix3f::Identity(3,3), OM_MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT10, JOINT9, SUCTION, OM_MATH::makeVector3(0.200, -0.006, 0), Matrix3f::Identity(3,3), OM_MATH::makeVector3(0,1,0));
  omlink.addTool(SUCTION, JOINT6, OM_MATH::makeVector3(0.03867882, 0.003, -0.01337315-0.01), Matrix3f::Identity(3,3), 4, 1);

  //initial joint angle set
  omlink.setComponentJointAngle(JOINT0, 0.0);
  omlink.setComponentJointAngle(JOINT1, -M_PI/2 + 0.0*DEG2RAD);
  omlink.setComponentJointAngle(JOINT2, -M_PI + 0.0*DEG2RAD);
  MyFunction::setPassiveJointAngle();

  //solve kinematics
  OM_KINEMATICS::LINK::forward(&omlink);

  //check setting
  //omlink.checkManipulatorSetting();
}


#endif //OPENMANIPULATOR_LINK_H_
