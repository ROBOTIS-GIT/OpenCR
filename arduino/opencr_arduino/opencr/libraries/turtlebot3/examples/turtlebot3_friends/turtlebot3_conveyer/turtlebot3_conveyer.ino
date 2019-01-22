#include "turtlebot3_conveyer.h"
#include "turtlebot3_conveyer_motor_driver.h"

#define LOOP_TIME_SEC 0.010f

RC100 rc100;
DynamixelStatus conveyer;
Turtlebot3MotorDriver motor_driver;

uint8_t conveyer_joint[4] = {JOINT_L_R, JOINT_R_R, JOINT_L_F, JOINT_R_F};
uint8_t conveyer_wheel[4] = {WHEEL_L_R, WHEEL_R_R, WHEEL_L_F, WHEEL_R_F};

void setup()
{
  Serial.begin(57600);
  // while(!Serial);

  rc100.begin(1);
  motor_driver.init();
}

void loop()
{
  static uint32_t previous_time = 0;
  uint32_t present_time = millis();

  getRC100Data();

  if((present_time - previous_time) >= (LOOP_TIME_SEC * 1000))
  {
    motor_driver.controlJoints(conveyer.setJointAngle());
    motor_driver.controlWheels(conveyer.setWheelVel());

    previous_time = millis();
  } 
}

void getRC100Data()
{
  if (rc100.available())
  {    
    int rcData = rc100.readData();

    conveyer.getDirection(rcData);
    delay(1);

    conveyer.setParams();
  }
}