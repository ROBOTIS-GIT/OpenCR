//#include <OpenManipulator.h>
#include "OpenManipulator_Link.h"

float present_time = 0.0;
float previous_time[3] = {0.0, 0.0, 0.0};

std::vector<float> init_joint_angle;
std::vector<float> target_angle;

std::vector<float> dxl_angle;
std::vector<float> OMlink_angle;

std::vector<float> ac_angle_temp;
std::vector<float> ac_angle_min;
std::vector<float> ac_angle_max;
bool error_flug[3];
int8_t i_check = 0;

void setup() 
{
  Serial.begin(57600);
  DEBUG.begin(57600);
  // while (!Serial)
  //   ;

  dxl.init();

  dxl.enableAllDynamixel();
  OM_PROCESSING::initProcessing(12);

  init_joint_angle.push_back(0.0*DEG2RAD);
  init_joint_angle.push_back(-90.0*DEG2RAD);
  init_joint_angle.push_back(-160.0*DEG2RAD);

  dxl_angle.push_back(init_joint_angle.at(0));
  dxl_angle.push_back(-init_joint_angle.at(1));
  dxl_angle.push_back(init_joint_angle.at(2));
  dxl.setAngle(dxl_angle);
  dxl_angle.clear();

  //dxl.setAngle(init_joint_angle);

  initOMLink();
  previous_time[0] = (float)(millis()/1000.0f);
  previous_time[1] = (float)(millis()/1000.0f);
  previous_time[2] = (float)(millis()/1000.0f);
}

void loop() 
{
  present_time = (float)(millis()/1000.0f);
  
  MyFunction::getData(8);
  MyFunction::setMotion();

  if(error_flug[0])
  {
    if(present_time-previous_time[1] >= CONTROL_PERIOD)
    {
      MyFunction::jointMove(present_time);  
      previous_time[1] = (float)(millis()/1000.0f);
    } 
  }  

  if(present_time-previous_time[0] >= CONTROL_PERIOD)
  {
    dxl_angle = dxl.getAngle();
    OMlink_angle.push_back(dxl_angle.at(0));
    OMlink_angle.push_back(-dxl_angle.at(1));
    OMlink_angle.push_back(dxl_angle.at(2));
    OPEN_MANIPULATOR::MANAGER::setAllActiveJointAngle(&omlink, OMlink_angle);
    dxl_angle.clear();
    OMlink_angle.clear();

    MyFunction::setPassiveJointAngle();
    OM_KINEMATICS::LINK::forward(&omlink);

    if(send_processing_flug)
      {
        OM_PROCESSING::sendAngle2Processing(OPEN_MANIPULATOR::MANAGER::getAllJointAngle(&omlink));
        DEBUG.print(" angle : ");
        for(i_check=0; i_check < 3; i_check++)
        {
          if(OPEN_MANIPULATOR::MANAGER::getAllJointAngle(&omlink).at(i_check)<-2*M_PI)
          {
            error_flug[0] = false;
          }
          else if(OPEN_MANIPULATOR::MANAGER::getAllJointAngle(&omlink).at(i_check)>2*M_PI)
          {
            error_flug[0] = false;
          }
          else
          {
            error_flug[0] = true;
          }
          DEBUG.print(OPEN_MANIPULATOR::MANAGER::getAllJointAngle(&omlink).at(i_check+1));
          DEBUG.print(" , ");
        }
        DEBUG.println(" ");
      }
    previous_time[0] = (float)(millis()/1000.0f);
  } 
}


