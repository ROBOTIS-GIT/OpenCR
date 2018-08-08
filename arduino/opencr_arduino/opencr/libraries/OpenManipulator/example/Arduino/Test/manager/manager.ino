#include <OMManager.h>
#include <Eigen.h>

Eigen::Vector3f makeVector3(float v1, float v2, float v3);
Eigen::Matrix3f makeMatrix3(float m11, float m12, float m13,
                                 float m21, float m22, float m23,
                                 float m31, float m32, float m33);

#define ZERO_VECTOR Vector3f::Zero()
#define IDENTITY_MATRIX Matrix3f::Identity(3, 3)

#define X_AXIS makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS makeVector3(0.0, 0.0, 1.0)

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
  manipulator.addComponent(COMP1, WORLD, COMP2, makeVector3(-0.100, 0.0, 0.0), IDENTITY_MATRIX, Z_AXIS, 1, 0.5, 1.0);
  manipulator.addComponent(COMP2, COMP1, COMP4, makeVector3(0.0, 0.0, 0.050), IDENTITY_MATRIX, Y_AXIS, 2, -1.5, 1.5);
  manipulator.addComponentChild(COMP2, COMP3);
  manipulator.addComponent(COMP3, COMP2, TOOL, makeVector3(0.0, 0.050, 0.0), IDENTITY_MATRIX);
  manipulator.addComponent(COMP4, COMP2, TOOL, makeVector3(0.0, 0.0, 0.050), IDENTITY_MATRIX, Y_AXIS, 3, 2.0);  
  manipulator.addTool(TOOL, COMP4, makeVector3(0.0, 0.0, 0.025), IDENTITY_MATRIX, 4, 1.0, 0.5);

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
