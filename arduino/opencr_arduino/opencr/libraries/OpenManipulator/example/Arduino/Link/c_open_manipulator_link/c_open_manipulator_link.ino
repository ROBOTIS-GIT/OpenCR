//#include <OpenManipulator.h>
#include "OpenManipulator_Link.h"

uint32_t present_time = 0;
uint32_t previous_time = 0;

void setup() 
{
  Serial.begin(57600);
  while (!Serial)
    ;
  
  dxl.init();
  dxl.setMaxPositionLimit(1, 180*DEG2RAD);
  dxl.setMinPositionLimit(1, -180*DEG2RAD);
  dxl.setMaxPositionLimit(2, 180*DEG2RAD);
  dxl.setMinPositionLimit(2, -180*DEG2RAD);
  dxl.setMaxPositionLimit(3, 180*DEG2RAD);
  dxl.setMinPositionLimit(3, -180*DEG2RAD);

  initOMLink();
  dxl.setAngle(getAllActiveJointAngle(&oimlink));
//   USB.println("///////////////////////////////////////////////////////////////////////////////////////");
//   USB.println("///////////////////////////////////Test Inverse////////////////////////////////////////");
//   USB.println("///////////////////////////////////////////////////////////////////////////////////////");

//   VectorXf target_angle(omlink.getDOF());
//   Pose target_pose;
//   target_pose.position = MATH::makeVector3(0.05, -0.05, 0.0);
//   target_pose.orientation = Matrix3f::Identity(3,3);

//   target_angle = KINEMATICS::LINK::geometricInverse(&omlink, SUCTION, target_pose);
  
//   target_angle = target_angle*RAD2DEG;
//   USB.println("\nTest Inverse");
//   USB.println(" target_pose : ");
//   PRINT::VECTOR(target_pose.position);
//   USB.println(" target_angle : ");
//   PRINT::VECTOR(target_angle);
//   target_angle = target_angle*DEG2RAD;

//   omlink.setComponentJointAngle(JOINT0, target_angle(0));
//   omlink.setComponentJointAngle(JOINT1, target_angle(1));
//   omlink.setComponentJointAngle(JOINT2, target_angle(2));
//   MyFunction::setPassiveJointAngle(&omlink);
//   KINEMATICS::LINK::forward(&omlink);
//   omlink.checkManipulatorSetting();

// ///////////////////////////////////////////////////////////////////////////////////////
//   USB.println("///////////////////////////////////////////////////////////////////////////////////////");
//   USB.println("///////////////////////////////////Test Inverse////////////////////////////////////////");
//   USB.println("///////////////////////////////////////////////////////////////////////////////////////");
//   target_pose.position = MATH::makeVector3(0.0, 0.0, 0.229);
//   target_pose.orientation = Matrix3f::Identity(3,3);

//   target_angle = KINEMATICS::LINK::geometricInverse(&omlink, SUCTION, target_pose);
  
//   target_angle = target_angle*RAD2DEG;
//   USB.println("\nTest Inverse");
//   USB.println(" target_pose : ");
//   PRINT::VECTOR(target_pose.position);
//   USB.println(" target_angle : ");
//   PRINT::VECTOR(target_angle);
//   target_angle = target_angle*DEG2RAD;

//   omlink.setComponentJointAngle(JOINT0, target_angle(0));
//   omlink.setComponentJointAngle(JOINT1, target_angle(1));
//   omlink.setComponentJointAngle(JOINT2, target_angle(2));
//   MyFunction::setPassiveJointAngle(&omlink);
//   KINEMATICS::LINK::forward(&omlink);
//   omlink.checkManipulatorSetting();

}

void loop() 
{
  present_time = millis();
  
  MyFunction::getData(100);
  MyFunction::setMotion();

  if(present_time-previous_time >= CONTROL_PERIOD)
  {
    MyFunction::jointMove();
    previous_time = millis();
  }

  MANAGER::setAllActiveJointAngle(&omlink, dxl.getAngle());
  MyFunction::setPassiveJointAngle();
  KINEMATICS::LINK::forward(&omlink);
  if(send_processing_flug)
  {
    PROCESSING::sendAngle2Processing(getAllJointAngle(&omlink));
  }

  
}


