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

//void mySetPassiveJointAngle(Manipulator* omlink);

//Manipulator omlink;

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


}

