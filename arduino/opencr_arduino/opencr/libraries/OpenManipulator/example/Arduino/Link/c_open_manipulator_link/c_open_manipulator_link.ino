//#include <OpenManipulator.h>
#include "OpenManipulator_Link.h"

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

#define PROCESSING true
#define DYNAMIXEL  false
#define TORQUE     true

#define MOTION_NUM 14

RC100 rc100;
String cmd[20];

float motion_storage[MOTION_NUM][2+omlink.getDof()] = {0.0, };
uint8_t motion_cnt = 0;
uint8_t motion_num = 0;
bool motion    = false;
bool repeat    = false;

const float initial_motion_set[MOTION_NUM][5] = { // time, grip, joint1, joint2, joint3,
                                            { 2.0,  0.0,   0.0,  0.92, -0.33},
                                            { 2.0,  1.0,   0.0,  0.92, -0.33},  
                                            { 2.0,  0.0,   0.0,   0.0, -1.37},
                                            { 2.0,  0.0,   0.0, -0.32, -0.81},
                                            { 2.0, -1.0,   0.0, -0.32, -0.81},
                                            { 2.0,  0.0,   0.0,  0.92, -0.33},
                                            { 2.0,  0.0,   1.5,  0.75, -0.80},
                                            { 2.0,  0.0,   0.0,  0.92, -0.33},
                                            { 2.0,  0.0,  -1.5,  0.75, -0.80},
                                            { 2.0,  0.0,   0.0,  0.92, -0.33},
                                            { 2.0,  0.0,   0.0,   0.0, -1.37},
                                            { 2.0,  0.0,   0.0, -0.32, -0.81},
                                            { 2.0,  1.0,   0.0, -0.32, -0.81},
                                            { 2.0,  0.0,   0.0,   0.0, -1.37}
                                        };

void setup() 
{
  Serial.begin(57600);
  while (!Serial)
    ;
  initOMLink();

  USB.println("///////////////////////////////////////////////////////////////////////////////////////");
  USB.println("///////////////////////////////////Test Inverse////////////////////////////////////////");
  USB.println("///////////////////////////////////////////////////////////////////////////////////////");

  VectorXf target_angle(omlink.getDOF());
  Pose target_pose;
  target_pose.position = MATH::makeVector3(0.05, -0.05, 0.0);
  target_pose.orientation = Matrix3f::Identity(3,3);

  target_angle = KINEMATICS::LINK::geometricInverse(&omlink, SUCTION, target_pose);
  
  target_angle = target_angle*RAD2DEG;
  USB.println("\nTest Inverse");
  USB.println(" target_pose : ");
  PRINT::VECTOR(target_pose.position);
  USB.println(" target_angle : ");
  PRINT::VECTOR(target_angle);
  target_angle = target_angle*DEG2RAD;

  omlink.setComponentJointAngle(JOINT0, target_angle(0));
  omlink.setComponentJointAngle(JOINT1, target_angle(1));
  omlink.setComponentJointAngle(JOINT2, target_angle(2));
  MyFunction::setPassiveJointAngle(&omlink);
  KINEMATICS::LINK::forward(&omlink);
  omlink.checkManipulatorSetting();

///////////////////////////////////////////////////////////////////////////////////////
  USB.println("///////////////////////////////////////////////////////////////////////////////////////");
  USB.println("///////////////////////////////////Test Inverse////////////////////////////////////////");
  USB.println("///////////////////////////////////////////////////////////////////////////////////////");
  target_pose.position = MATH::makeVector3(0.0, 0.0, 0.229);
  target_pose.orientation = Matrix3f::Identity(3,3);

  target_angle = KINEMATICS::LINK::geometricInverse(&omlink, SUCTION, target_pose);
  
  target_angle = target_angle*RAD2DEG;
  USB.println("\nTest Inverse");
  USB.println(" target_pose : ");
  PRINT::VECTOR(target_pose.position);
  USB.println(" target_angle : ");
  PRINT::VECTOR(target_angle);
  target_angle = target_angle*DEG2RAD;

  omlink.setComponentJointAngle(JOINT0, target_angle(0));
  omlink.setComponentJointAngle(JOINT1, target_angle(1));
  omlink.setComponentJointAngle(JOINT2, target_angle(2));
  MyFunction::setPassiveJointAngle(&omlink);
  KINEMATICS::LINK::forward(&omlink);
  omlink.checkManipulatorSetting();

}

void loop() 
{
  getData(100);
  setMotion();
  KINEMATICS::LINK::forward(&omlink);
}

void getData(uint32_t wait_time)
{
  static uint8_t state = 0;
  static uint32_t tick = 0;

  bool processing_flag = false;
  String get_processing_data = "";

  if (Serial.available())
  {
    get_processing_data = Serial.readStringUntil('\n');
    processing_flag = true;
  }

  switch (state)
  {
    case CHECK_FLAG:
        dataFromProcessing(get_processing_data);   
        tick = millis();
        state  = WAIT_FOR_SEC;
     break;
    
    case WAIT_FOR_SEC:
      if ((millis() - tick) >= wait_time)
      {
        state = CHECK_FLAG;
      }
     break;
    
    default :
     state = CHECK_FLAG;
     break;
  }
}

void setMotion()
{
  if (motion)
  {
    if (/*moving*/)
      return;

    if (motion_cnt >= motion_num)
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

    if (motion_storage[motion_cnt][1] == -1.0)
    {
      // setGripAngle(-0.4);
      // setGripAngle(-0.2);
      // move(1.5);
      //suction off

      motion_cnt++;
    }
    else if (motion_storage[motion_cnt][1] == 1.0)
    {
      //setGripAngle(grip_off);
      //move(1.5);
      //suction on

      motion_cnt++;
    }
    else
    {
      std::vector <float> target_angle;
      
      for (int i = 0; i < omlink.getDOF(); i++)
        target_angle.push_back(motion_storage[motion_cnt][i+1]);
  
      MANAGER::setAllActiveJointAngle(omlink, target_angle);
      //move(motion_storage[motion_cnt][0]);

      motion_cnt++;
    }
  }
  else
  {
    motion_cnt = 0;
  }
}

void dataFromProcessing(String get)
{
  get.trim();

  PROCESSING::split(get, ',', cmd);

  if (cmd[0] == "om")
  {
    if (cmd[1] == "ready")
    {
      if (DYNAMIXEL)
      {
        //setTorque(true);
        PROCESSING::sendAngle2Processing(getAngle()); 
      }

      if (PROCESSING)
        PROCESSING::sendAngle2Processing(getState()); 
    }
    else if (cmd[1] == "end")
    {
      if (DYNAMIXEL)
        //setTorque(false);
    }
  }
  else if (cmd[0] == "joint")
  {
    std::Vector<float> target_angle;

    for (int i = 0; i < omlink.getDOF(); i++)
      target_angle.push_back(cmd[i].toFloat());

    //omlink.setJointAngle(target_angle);
    //move();
  }
  else if (cmd[0] == "suction")
  {
    if (cmd[1] == "on")
      omlink.setComponentToolOnOff(SUCTION, true);
    else if (cmd[1] == "off")
      omlink.setComponentToolOnOff(SUCTION, false);
      
    //move(1.5);
  }
  else if (cmd[0] == "task")
  {
    Pose target_pose;
    std::vector<float> target_angle;

    target_pose = setPose(cmd[1]);
    target_angle = KINEMATICS::LINK::geometricInverse(&omlink, SUCTION, target_pose);

    //omlink.setJointAngle(target_angle);
    //move();
  }
  else if (cmd[0] == "motor")
  {
    if (DYNAMIXEL)
    {
      if (cmd[1] == "enable")
        //setTorque(true);
      else if (cmd[1] == "disable")
        //setTorque(false);
    }
  }
  else if (cmd[0] == "get")
  {
    if (cmd[1] == "on")
    {
      motion_storage[motion_num][0] = 3.0;  //mov_time
      motion_storage[motion_num][1] = -1.0;
      motion_num++;
    }
    else if (cmd[1] == "off")
    {
      motion_storage[motion_num][0] = 3.0;  //mov_time
      motion_storage[motion_num][1] = 1.0;
      motion_num++;
    }
    else if (cmd[1] == "clear")
    {
      for (int i = 0; i < MOTION_NUM; i++)
      {
        for (int j = 0; j < 2+omlink.getDOF(); j++)
        {
          motion_storage[i][j] = 0.0;
        }
      }
      
      motion_num = 0;
      motion_cnt = 0;
      motion     = false;
      repeat     = false;
    }
    else if (cmd[1] == "pose")
    {
      if (cmd[2].toInt() < MOTION_NUM)
      {
        std::Vector<float> target_angle = MANAGER::getAllJointAngle(omlink);

        if (DYNAMIXEL)
          PROCESSING::sendAngle2Processing(target_angle); 

        for (int i = 0; i < 2+omlink.getDOF(); i++)
        {
          motion_storage[motion_num][0]   = 3.0;  //mov_time
          motion_storage[motion_num][i+1] = target_angle.at(i);
        }
        motion_num++;
      }
    }
  }
  else if (cmd[0] == "hand")
  {
    if (cmd[1] == "once")
    {
      if (DYNAMIXEL)
      {
        //setTorque(true);
        PROCESSING::sendAngle2Processing(MANAGER::getAllJointAngle(omlink)); 
      }

      motion_cnt = 0;
      motion = true;
    }
    else if (cmd[1] == "repeat")
    {
      if (DYNAMIXEL)
      {
        //setTorque(true);
        PROCESSING::sendAngle2Processing(MANAGER::getAllJointAngle(omlink)); 
      }

      motion_cnt = 0;
      motion = true;
      repeat = true;
    }
    else if (cmd[1] == "stop")
    {
      for (int i = 0; i < MOTION_NUM; i++)
      {
        for (int j = 0; j < 2+omlink.getDOF(); j++)
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
    if (cmd[1] == "start")
    {
      if (DYNAMIXEL)
        PROCESSING::sendAngle2Processing(MANAGER::getAllJointAngle(omlink));

      if (PROCESSING)
        PROCESSING::sendAngle2Processing(MANAGER::getAllJointAngle(omlink));

      for (int i = 0; i < MOTION_NUM; i++)
      {
        for (int j = 0; j < 2+omlink.getDOF(); j++)
        {
          motion_storage[i][j] = motion_set[i][j];
        }
      }

      motion_num = MOTION_NUM;  
      motion_cnt = 0;          
      motion = true;
      repeat = true;
    }
    else if (cmd[1] == "stop")
    {
      motion_cnt = 0;
      motion     = false;
      repeat     = false;
    }
  }
}

Pose setPose(String dir)
{
  Pose target_pose;
  float step = 0.010;

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
    
  return target_pose;
}

