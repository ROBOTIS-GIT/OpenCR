#include <OMManager.h>
#include <OMMath.h>
#include <OMKinematics.h>

#include <Eigen.h>

#define X_AXIS MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS MATH::makeVector3(0.0, 0.0, 1.0)


#define WORLD 0
#define COMP1 1
#define COMP2 2
#define COMP3 3
#define COMP4 4
#define TOOL 5

#define NONE -1

void setup()
{
  Serial.begin(57600);
  while (!Serial);

  Manipulator manipulator;
  manipulator.addWorld(WORLD, COMP1);
  manipulator.addComponent(COMP1, WORLD, COMP2, MATH::makeVector3(-0.100, 0.0, 0.0), IDENTITY_MATRIX, 1, Z_AXIS, 0.5, MATH::makeMatrix3(1,1,1,1,1,1,3,1,1));
  manipulator.addComponent(COMP2, COMP1, COMP3, MATH::makeVector3(0.0, 0.0, 0.050), IDENTITY_MATRIX, 2, Y_AXIS, 1.5);
  manipulator.addComponentChild(COMP2, COMP4);
  manipulator.addComponent(COMP3, COMP2, COMP4, MATH::makeVector3(0.0, 0.050, 0.0), IDENTITY_MATRIX, NONE);
  manipulator.addComponent(COMP4, COMP2, TOOL, MATH::makeVector3(0.0, 0.0, 0.050), IDENTITY_MATRIX, 3, Y_AXIS, 2.0);
  manipulator.addTool(TOOL, COMP4, MATH::makeVector3(0.0, 0.0, 0.025), IDENTITY_MATRIX, 4, 0.1);

  // manipulator.checkManipulatorSetting();
}

void loop()
{
}