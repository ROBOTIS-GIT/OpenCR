//#include <OpenManipulator.h>
#include "OpenManipulator_Link.h"

uint32_t present_time = 0;
uint32_t previous_time[2] = {0, 0};

// std::vector<float> ac_angle_temp;
// std::vector<float> ac_angle_min;
// std::vector<float> ac_angle_max;
// bool test_flug[3];
// int8_t ias = 0;

void setup() 
{
  Serial.begin(57600);
  DEBUG.begin(57600);
  // while (!Serial)
  //   ;

  dxl.init();
  PROCESSING::initProcessing(12);

  initOMLink();
  MyFunction::updateJointTrajectory(OPEN_MANIPULATOR::MANAGER::getAllActiveJointAngle(&omlink) ,MOVE_TIME);
  // while (true)
  //   ;
  previous_time[0] = millis();
  previous_time[1] = millis();

  // ac_angle_temp.push_back(0.0);
  // ac_angle_temp.push_back(-90.0*DEG2RAD);
  // ac_angle_temp.push_back(-180.0*DEG2RAD);
  // ac_angle_min.push_back(-90.0*DEG2RAD);
  // ac_angle_min.push_back(-90.0*DEG2RAD);
  // ac_angle_min.push_back(-180.0*DEG2RAD);
  // ac_angle_max.push_back(90.0*DEG2RAD);
  // ac_angle_max.push_back(0.0*DEG2RAD);
  // ac_angle_max.push_back(-130.0*DEG2RAD);
  // test_flug[0] = true;
  // test_flug[1] = true;
  // test_flug[2] = true;
}

void loop() 
{
   present_time = millis();
  MyFunction::getData(8);
  //MyFunction::setMotion();
  if(present_time-previous_time[0] >= CONTROL_PERIOD*1000)
  {
    //MyFunction::jointMove();
    // for(ias = 0; ias < 3; ias++)
    // {
    //   if(test_flug[ias])
    //   {
    //     ac_angle_temp.at(ias) += 0.1*DEG2RAD;
    //     if(ac_angle_temp.at(ias) > ac_angle_max[ias])
    //     {
    //       test_flug[ias] = false;
    //     }
    //   }
    //   else
    //   {
    //     ac_angle_temp.at(ias) -= 0.1*DEG2RAD;
    //     if(ac_angle_temp.at(ias) < ac_angle_min[ias])
    //     {
    //       test_flug[ias] = true;
    //     }
    //   }
    // }
    previous_time[0] = millis();
  }
  //forward
  // OPEN_MANIPULATOR::MANAGER::setAllActiveJointAngle(&omlink, ac_angle_temp);
  OPEN_MANIPULATOR::MANAGER::setAllActiveJointAngle(&omlink, dxl.getAngle());
  MyFunction::setPassiveJointAngle();
  KINEMATICS::LINK::forward(&omlink);
  
  
  if(present_time-previous_time[1] >= 8)
  {
    if(send_processing_flug)
      {
        PROCESSING::sendAngle2Processing(OPEN_MANIPULATOR::MANAGER::getAllJointAngle(&omlink));
      }
    previous_time[1] = millis();
  }

  
}


