//#include <OpenManipulator.h>
#include <OMManager.h>
#include <OMMath.h>
#include <OMKinematics.h>
#include <OMDebug.h>
//#include "OpenManipulator_Link.h"

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

Manipulator omlink;

void setup() 
{
  Serial.begin(57600);
  while (!Serial)
    ;
  
  omlink.addWorld(WORLD, BASE);
  omlink.addComponent(BASE, WORLD, JOINT0, MATH::makeVector3(-150, 0, 0), Matrix3f::Identity(3,3));
  omlink.addComponent(JOINT0, BASE, JOINT1, Vector3f::Zero(), Matrix3f::Identity(3,3), 1, MATH::makeVector3(0,0,1));
  omlink.addComponentChild(JOINT0, JOINT2);
  omlink.addComponentChild(JOINT0, JOINT7);
  omlink.addComponent(JOINT1, JOINT0, JOINT5, MATH::makeVector3(0, 22, 52), Matrix3f::Identity(3,3), 1, MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT2, JOINT0, JOINT3, MATH::makeVector3(0, -22, 52), Matrix3f::Identity(3,3), 2, MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT3, JOINT2, JOINT4, MATH::makeVector3(50, 7, 0), Matrix3f::Identity(3,3), 3, MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT4, JOINT3, JOINT5, MATH::makeVector3(200, 6, 0), Matrix3f::Identity(3,3), -1, MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT5, JOINT1, JOINT6, MATH::makeVector3(200, -16, 0), Matrix3f::Identity(3,3), -1, MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT6, JOINT5, SUCTION, MATH::makeVector3(200, -9, 0), Matrix3f::Identity(3,3), -1, MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT7, JOINT0, JOINT8, MATH::makeVector3(-45.31539, 6, 73.13091), Matrix3f::Identity(3,3), -1, MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT8, JOINT7, JOINT9, MATH::makeVector3(200, 9, 0), Matrix3f::Identity(3,3), -1, MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT9, JOINT8, JOINT10, MATH::makeVector3(76.60444, -6, 0), Matrix3f::Identity(3,3), -1, MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT10, JOINT9, SUCTION, MATH::makeVector3(200, -6, 0), Matrix3f::Identity(3,3), -1, MATH::makeVector3(0,1,0));
  omlink.addTool(SUCTION, JOINT6, MATH::makeVector3(38.67882, 3, -13.37315), Matrix3f::Identity(3,3), 4);

  omlink.checkManipulatorSetting();
}

void loop() 
{
  KINEMATICS::LINK::forward(&omlink);
  
  //myGetPassiveJointAngle(&omlink);

}

void myGetPassiveJointAngle(Manipulator* omlink, bool* error = false)
{
  float joint_angle[3];
  joint_angle[0] = omlink->getComponentJointAngle(JOINT0);
  joint_angle[1] = omlink->getComponentJointAngle(JOINT1);
  joint_angle[2] = omlink->getComponentJointAngle(JOINT2);

  omlink->setComponentJointAngle(JOINT3, joint_angle[1]-joint_angle[2]);
  omlink->setComponentJointAngle(JOINT4, -M_PI-(joint_angle[1]-joint_angle[2]));
  omlink->setComponentJointAngle(JOINT5, -M_PI-(joint_angle[1]-joint_angle[2]));
  omlink->setComponentJointAngle(JOINT6, (155 * DEG2RAD)-joint_angle[2]);
  omlink->setComponentJointAngle(JOINT7, joint_angle[1]);
  omlink->setComponentJointAngle(JOINT8, (15 * DEG2RAD)-joint_angle[1]);
  omlink->setComponentJointAngle(JOINT9, joint_angle[2]-(195 * DEG2RAD));
  omlink->setComponentJointAngle(JOINT10, (90 * DEG2RAD)-joint_angle[2]);
}
