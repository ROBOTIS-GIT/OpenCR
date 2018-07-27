#include <OMManager.h>
#include <OMMath.h>
#include <OMKinematics.h>

#define WORLD     0
#define BASE      1
#define SHOULDER  2
#define UPPER_ARM 3
#define LOWER_ARM 4
#define WRIST     5
#define GRIPPER   6

#define NONE      -1

#define FIST_MAni
#define se

void setup()
{
  // Manipulator manipulator;
  addManipulator();
  addworld(manipulator, WORLD,position, orientation);
  addComponent();
  addTool();

  manipulator.addComponent(WORLD, NONE, BASE, mass, inertia_tensor, relative_position, relative_orientation, axis_of_rotation);
  manipulator.addComponent(WORLD, NONE, BASE, mass, inertia_tensor, relative_position, relative_orientation, axis_of_rotation);
  manipulator.addComponent(WORLD, NONE, BASE, mass, inertia_tensor, relative_position, relative_orientation, axis_of_rotation);
  manipulator.addComponent(WORLD, NONE, BASE, mass, inertia_tensor, relative_position, relative_orientation, axis_of_rotation);
  manipulator.addComponent(WORLD, NONE, BASE, mass, inertia_tensor, relative_position, relative_orientation, axis_of_rotation);
  
  getManipulator(manipulator);

  OMLinkKinematics OMLinkKinematics;
  connectForward(OMLinkKinematics.forward(First_manipulator));
}

void loop()
{

}



#if 0
void setup()
{
  OMMath math_;
  Manipulator manipulator;
  bool error;

  manipulator.initManipulator(3, 11, 8, 1);
  manipulator.makeBase("base");
  manipulator.makeJoint("joint0", 1, math_.makeEigenVector3(0,0,1));
  manipulator.makeJoint("joint1", 2, math_.makeEigenVector3(0,1,0));
  manipulator.makeJoint("joint2", 3, math_.makeEigenVector3(0,1,0));
  manipulator.makeJoint("joint3");
  manipulator.makeJoint("joint4");
  manipulator.makeJoint("joint5");
  manipulator.makeJoint("joint6");
  manipulator.makeJoint("joint7");
  manipulator.makeJoint("joint8");
  manipulator.makeJoint("joint9");
  manipulator.makeJoint("joint10");
  manipulator.makeTool("suction", 4, math_.makeEigenVector3(1,0,0));

  manipulator.makeLink("Baselink",2);
  manipulator.addControlPoint("Baselink", "base", math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("Baselink", "joint0", math_.makeEigenVector3(-150, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

  manipulator.makeLink("link0", 4);
  manipulator.addControlPoint("link0", "joint0", math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("link0", "joint1", math_.makeEigenVector3(0, 22, 52), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("link0", "joint2", math_.makeEigenVector3(0,-22, 52), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("link0", "joint7", math_.makeEigenVector3(-45.31539, 6, 73.13091), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

  manipulator.makeLink("link1", 2);
  manipulator.addControlPoint("link1", "joint1", math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("link1", "joint5", math_.makeEigenVector3(200, -16, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  
  manipulator.makeLink("link2", 2);
  manipulator.addControlPoint("link2", "joint2", math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("link2", "joint3", math_.makeEigenVector3(50, 7, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

  manipulator.makeLink("link3", 2);
  manipulator.addControlPoint("link3", "joint3", math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("link3", "joint4", math_.makeEigenVector3(200, 6, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

  manipulator.makeLink("link4", 3);
  manipulator.addControlPoint("link4", "joint4", math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("link4", "joint5", math_.makeEigenVector3(50, 15, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("link4", "joint6", math_.makeEigenVector3(250, 6, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

  manipulator.makeLink("link5", 2);
  manipulator.addControlPoint("link5", "joint7", math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("link5", "joint8", math_.makeEigenVector3(200, 9, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));


  manipulator.makeLink("link6", 3);
  manipulator.addControlPoint("link6", "joint5", math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("link6", "joint8", math_.makeEigenVector3(-38.30222, 9, 32.13938), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("link6", "joint9", math_.makeEigenVector3(38.30222, 3, 32.13938), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));

  manipulator.makeLink("link7", 2);
  manipulator.addControlPoint("link7", "joint9", math_.makeEigenVector3(0, 0, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("link7", "joint10", math_.makeEigenVector3(200, -6, 0), math_.makeRotationMatrix(0*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD));


  manipulator.makeLink("link8", 3);
  manipulator.addControlPoint("link8", "joint6", math_.makeEigenVector3(-38.67882, -3, 13.37315), math_.makeRotationMatrix(0*DEG2RAD, -90*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("link8", "joint10", math_.makeEigenVector3(-10, 3, 54.33076), math_.makeRotationMatrix(0*DEG2RAD, -25*DEG2RAD, 0*DEG2RAD));
  manipulator.addControlPoint("link8", "suction", Eigen::Vector3f::Zero(), Eigen::Matrix3f::Identity(3,3));

  Serial.write(error);
}

void loop()
{

}
<<<<<<< HEAD
=======
#endif
>>>>>>> d5ad3ac41573ad5609aabe5d679a3f5c631fb9e9
