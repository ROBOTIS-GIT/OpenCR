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
  manipulator.addComponent(COMP1, WORLD, COMP2, MATH::makeVector3(-0.100, 0.0, 0.0), IDENTITY_MATRIX, Z_AXIS, 1, 0.5, 1.0);
  manipulator.addComponent(COMP2, COMP1, COMP4, MATH::makeVector3(0.0, 0.0, 0.050), IDENTITY_MATRIX, Y_AXIS, 2, -1.5, 1.5);
  manipulator.addComponentChild(COMP2, COMP3);
  manipulator.addComponent(COMP3, COMP2, TOOL, MATH::makeVector3(0.0, 0.050, 0.0), IDENTITY_MATRIX);
  manipulator.addComponent(COMP4, COMP2, TOOL, MATH::makeVector3(0.050, 0.0, 0.0), IDENTITY_MATRIX, Y_AXIS, 3, 2.0);  
  manipulator.addTool(TOOL, COMP4, MATH::makeVector3(0.025, 0.0, 0.0), IDENTITY_MATRIX, 4, 1.0, 0.5);

  Eigen::MatrixXf jacobian = KINEMATICS::CHAIN::jacobian(&manipulator, TOOL);
  LOG::INFO("Jacobian : ");
  PRINT::MATRIX(jacobian);

  KINEMATICS::CHAIN::forward(&manipulator, COMP1);
  manipulator.checkManipulatorSetting();

  Pose target;
  target.position = MATH::makeVector3(-0.043, 0.0, 0.086);
  target.orientation = IDENTITY_MATRIX;
  std::vector<float> joint_angle = KINEMATICS::CHAIN::inverse(&manipulator, TOOL, target);
  LOG::INFO("Result of inverse : ");
  PRINT::VECTOR(joint_angle);
}

void loop()
{
}