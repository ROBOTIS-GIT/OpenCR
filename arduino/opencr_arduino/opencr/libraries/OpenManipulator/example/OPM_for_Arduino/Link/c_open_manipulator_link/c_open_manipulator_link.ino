#include <OpenManipulator.h>
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

void setup() 
{
  Serial.begin(57600);

  //pinMode(RELAY_PIN, OUTPUT);
  initOMLink();
}

void loop() 
{
  

}

void myGetPassiveJointAngle(Manipulator* omlink, bool* error = false)
{
  float joint_angle[3];
  joint_angle[0] = getComponentJointAngle(omlink, JOINT0);
  joint_angle[1] = getComponentJointAngle(omlink, JOINT1);
  joint_angle[2] = getComponentJointAngle(omlink, JOINT2);

  setComponentJointAngle(omlink, JOINT3, joint_angle[1]-joint_angle[2]);
  setComponentJointAngle(omlink, JOINT4, -M_PI-(joint_angle[1]-joint_angle[2]));
  setComponentJointAngle(omlink, JOINT5, -M_PI-(joint_angle[1]-joint_angle[2]));
  setComponentJointAngle(omlink, JOINT6, (155 * DEG2RAD)-joint_angle[2]);
  setComponentJointAngle(omlink, JOINT7, joint_angle[1]);
  setComponentJointAngle(omlink, JOINT8, (15 * DEG2RAD)-joint_angle[1]);
  setComponentJointAngle(omlink, JOINT9, joint_angle[2]-(195 * DEG2RAD));
  setComponentJointAngle(omlink, JOINT10, (90 * DEG2RAD)-joint_angle[2]);
}
