#include <OMManager.h>
#include <Eigen.h>

Eigen::Vector3f makeEigenVector3(float v1, float v2, float v3);
Eigen::Matrix3f makeEigenMatrix3(float m11, float m12, float m13,
                                 float m21, float m22, float m23,
                                 float m31, float m32, float m33);

#define ZERO_VECTOR Vector3f::Zero()
#define IDENTITY_MATRIX Matrix3f::Identity(3, 3)

#define X_AXIS makeEigenVector3(1.0, 0.0, 0.0)
#define Y_AXIS makeEigenVector3(0.0, 1.0, 0.0)
#define Z_AXIS makeEigenVector3(0.0, 0.0, 1.0)

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
  manipulator.addComponent(COMP1, WORLD, COMP2, makeVector3(-0.100, 0.0, 0.0), IDENTITY_MATRIX, 1, Z_AXIS, 0.5, makeMatrix3(1, 1, 1, 1, 1, 1, 3, 1, 1));
  manipulator.addComponent(COMP2, COMP1, COMP3, makeVector3(0.0, 0.0, 0.050), IDENTITY_MATRIX, 2, Y_AXIS, 1.5);
  manipulator.addComponentChild(COMP2, COMP4);
  manipulator.addComponent(COMP3, COMP2, TOOL, makeVector3(0.0, 0.050, 0.0), IDENTITY_MATRIX, 3, Y_AXIS, 2.0);
  manipulator.addComponent(COMP4, COMP2, TOOL, makeVector3(0.0, 0.0, 0.050), IDENTITY_MATRIX, NONE);
  manipulator.addTool(TOOL, COMP3, makeVector3(0.0, 0.0, 0.025), IDENTITY_MATRIX, 4, 0.1);

  manipulator.checkManipulatorSetting();
}

void loop()
{
}

Eigen::Vector3f makeVector3(float v1, float v2, float v3)
{
  Eigen::Vector3f temp;
  temp << v1, v2, v3;
  return temp;
}

Eigen::Matrix3f makeMatrix3(float m11, float m12, float m13,
                                 float m21, float m22, float m23,
                                 float m31, float m32, float m33)
{
  Eigen::Matrix3f temp;
  temp << m11, m12, m13, m21, m22, m23, m31, m32, m33;
  return temp;
}
