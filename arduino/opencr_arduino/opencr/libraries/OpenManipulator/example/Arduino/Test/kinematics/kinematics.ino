#include <OMManager.h>
#include <OMMath.h>
#include <OMKinematics.h>
#include <OMDebug.h>

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
  while (!Serial)
    ;

  Manipulator manipulator;
  manipulator.addWorld(WORLD, COMP1);
  manipulator.addComponent(COMP1, WORLD, COMP2, MATH::makeVector3(-0.100, 0.0, 0.0), IDENTITY_MATRIX, 1, Z_AXIS, 0.5, MATH::makeMatrix3(1, 1, 1, 1, 1, 1, 3, 1, 1));
  manipulator.addComponent(COMP2, COMP1, COMP3, MATH::makeVector3(0.0, 0.0, 0.050), IDENTITY_MATRIX, 2, Y_AXIS, 1.5);
  manipulator.addComponentChild(COMP2, COMP4);
  manipulator.addComponent(COMP3, COMP2, TOOL, MATH::makeVector3(0.0, 0.050, 0.0), IDENTITY_MATRIX, 3, Y_AXIS, 2.0);
  manipulator.addComponent(COMP4, COMP2, TOOL, MATH::makeVector3(0.0, 0.0, 0.050), IDENTITY_MATRIX, NONE);
  manipulator.addTool(TOOL, COMP3, MATH::makeVector3(0.0, 0.0, 0.025), IDENTITY_MATRIX, 4, 0.1);

  Eigen::MatrixXf jacobian = KINEMATICS::CHAIN::jacobian(&manipulator, TOOL);
  LOG::INFO("Jacobian : ");
  PRINT::MATRIX(jacobian);

  KINEMATICS::CHAIN::forward(&manipulator, COMP1);
  manipulator.checkManipulatorSetting();

  Pose target;
  target.position = ZERO_VECTOR;
  target.orientation = IDENTITY_MATRIX;
  Eigen::VectorXf joint_angle = KINEMATICS::CHAIN::inverse(&manipulator, TOOL, target);
  LOG::INFO("Result of inverse : ");
  PRINT::VECTOR(joint_angle);
}

void loop()
{
}