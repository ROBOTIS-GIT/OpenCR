#include <OMMath.h>
#include <OMDebug.h>

#include <Eigen.h>

void setup()
{
  Serial.begin(57600);
  while (!Serial)
    ;

  LOG::INFO("sign");
  USB.println(MATH::sign(1.0));
  USB.println(MATH::sign(-2.0));
  USB.println();

  Eigen::Vector3f vTemp = ZERO_VECTOR;
  Eigen::Vector3f v1 = MATH::makeVector3(1, 0, 0);
  Eigen::Vector3f v2 = MATH::makeVector3(0, 0, 1);

  Eigen::Matrix3f mTemp = IDENTITY_MATRIX;
  Eigen::Matrix3f m1 = MATH::makeMatrix3(1, 0, 0,
                                         0, cos(45 * DEG2RAD), -sin(45 * DEG2RAD),
                                         0, sin(45 * DEG2RAD), cos(45 * DEG2RAD));

  Eigen::Matrix3f m2 = MATH::makeMatrix3(cos(45 * DEG2RAD), -sin(45 * DEG2RAD), 0,
                                         sin(45 * DEG2RAD), cos(45 * DEG2RAD), 0,
                                         0, 0, 1);

  LOG::INFO("makeVector3(float, float, float)");
  PRINT::VECTOR(v1);
  PRINT::VECTOR(v2);

  LOG::INFO("makeMatrix3(float, float, float, ...)");
  PRINT::MATRIX(m1);
  PRINT::MATRIX(m2);

  LOG::INFO("matrixLogarithm(m1)");
  vTemp = MATH::matrixLogarithm(m1);
  PRINT::VECTOR(vTemp);

  LOG::INFO("skewSymmetricMatrix(v1)");
  mTemp = MATH::skewSymmetricMatrix(v1);
  PRINT::MATRIX(mTemp);

  LOG::INFO("rodriguesRotationMatrix(v1, 45 deg)");
  mTemp = MATH::rodriguesRotationMatrix(v1, 45.0 * DEG2RAD);
  PRINT::MATRIX(mTemp);

  LOG::INFO("makeRotationMatrix(0.0, 0.0, 45 deg)");
  mTemp = MATH::makeRotationMatrix(0.0, 0.0, 45.0 * DEG2RAD);
  PRINT::MATRIX(mTemp);

  LOG::INFO("makeRotationMatrix(v1)");
  mTemp = MATH::makeRotationMatrix(v1);
  PRINT::MATRIX(mTemp);

  LOG::INFO("makeRotationVector(m1) == matrixLogarithm(m1)");
  vTemp = MATH::makeRotationVector(m1);
  PRINT::VECTOR(vTemp);

  LOG::INFO("positionDifference(v1, v2)");
  vTemp = MATH::positionDifference(v1, v2);
  PRINT::VECTOR(vTemp);

  LOG::INFO("orientationDifference(m1, m2)");
  vTemp = MATH::orientationDifference(m1, m2);
  PRINT::VECTOR(vTemp);

  LOG::INFO("poseDifference(v1, v2, m1, m2)");
  vTemp = MATH::poseDifference(v1, v2, m1, m2);
  PRINT::VECTOR(vTemp);
}

void loop()
{
}