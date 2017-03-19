#include <RC100.h>
#include <Dynamixel.h>

#define WHEEL_R 2
#define JOINT_F 1

#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_GOAL_POSITION 116

#define DYNAMIXEL_POWER_ENABLE_PIN 32

#define ON  1
#define OFF 0

#define BODY_WIDTH           12.8
#define BODY_LENGTH          19.0
#define SPEED_ADD_ON          2

class DynamixelStatus
{
 private:
   int velocity = 50;
  //  double distance_from_center = 25.6;

   int wheel_vel[2] = {0, };
   int joint_angle[2] = {0, };

   enum MODE
   {
     NONE = 0
     , FORWARD, BACKWARD, RLEFT, RRIGHT
     , RLEFT_FORWARD, RRIGHT_FORWARD, RLEFT_BACKWARD, RRIGHT_BACKWARD
     , RULEFT_FORWARD, RURIGHT_FORWARD, RULEFT_BACKWARD, RURIGHT_BACKWARD
   } mobile_mode;

 public:
   double distance_from_center = 12.2;

   int wheel_r_vel;
   int joint_f_angle;

   DynamixelStatus()
   : wheel_r_vel(0)
   , joint_f_angle(0)
   { }

   MODE getDirection(int rcData)
   {
     switch (rcData)
     {
      case (RC100_BTN_U): mobile_mode = FORWARD; break;
      case (RC100_BTN_D): mobile_mode = BACKWARD; break;
      case (RC100_BTN_L): mobile_mode = RLEFT; break;
      case (RC100_BTN_R): mobile_mode = RRIGHT; break;

      case (RC100_BTN_U + RC100_BTN_L): mobile_mode = RLEFT_FORWARD; break;
      case (RC100_BTN_U + RC100_BTN_R): mobile_mode = RRIGHT_FORWARD; break;
      case (RC100_BTN_D + RC100_BTN_L): mobile_mode = RLEFT_BACKWARD; break;
      case (RC100_BTN_D + RC100_BTN_R): mobile_mode = RRIGHT_BACKWARD; break;

      case (RC100_BTN_U + RC100_BTN_L + RC100_BTN_6): mobile_mode = RULEFT_FORWARD; break;
      case (RC100_BTN_U + RC100_BTN_R + RC100_BTN_5): mobile_mode = RURIGHT_FORWARD; break;
      case (RC100_BTN_D + RC100_BTN_L + RC100_BTN_6): mobile_mode = RULEFT_BACKWARD; break;
      case (RC100_BTN_D + RC100_BTN_R + RC100_BTN_5): mobile_mode = RURIGHT_BACKWARD; break;

      default: if (rcData & RC100_BTN_1) { if (velocity <= 245) velocity += 5; }
                else if (rcData & RC100_BTN_2) { if (distance_from_center >= (8.2)) distance_from_center -= 2.0; }
                else if (rcData & RC100_BTN_3) { if (velocity >= 5) velocity -= 5; }
                else if (rcData & RC100_BTN_4) { if (distance_from_center <= 18.2) distance_from_center += 2.0; }
                else { mobile_mode = NONE; }
                break;
     }

     return mobile_mode;
   }

   void setParams()
   {
     switch (mobile_mode)
     {
       case FORWARD:                wheel_r_vel = velocity * SPEED_ADD_ON;
                                    joint_f_angle = 0;
                                    break;
       case BACKWARD:               wheel_r_vel = -velocity * SPEED_ADD_ON;
                                    joint_f_angle = 0;
                                    break;
       case RLEFT:                  wheel_r_vel = velocity * SPEED_ADD_ON;
                                    joint_f_angle = 204;
                                    break;
       case RRIGHT:                 wheel_r_vel = velocity * SPEED_ADD_ON;
                                    joint_f_angle = -204;
                                    break;

       case RLEFT_FORWARD:          wheel_r_vel = velocity * SPEED_ADD_ON;
                                    joint_f_angle = 204;
                                    break;
       case RRIGHT_FORWARD:         wheel_r_vel = velocity * SPEED_ADD_ON;
                                    joint_f_angle = -204;
                                    break;
       case RLEFT_BACKWARD:         wheel_r_vel = -velocity * SPEED_ADD_ON;
                                    joint_f_angle = 204;
                                    break;
       case RRIGHT_BACKWARD:        wheel_r_vel = -velocity * SPEED_ADD_ON;
                                    joint_f_angle = -204;
                                    break;

       case RULEFT_FORWARD:         wheel_r_vel = velocity * SPEED_ADD_ON;
                                    joint_f_angle = 204;
                                    break;
       case RURIGHT_FORWARD:        wheel_r_vel = velocity * SPEED_ADD_ON;
                                    joint_f_angle = -204;
                                    break;
       case RULEFT_BACKWARD:        wheel_r_vel = -velocity * SPEED_ADD_ON;
                                    joint_f_angle = 204;
                                    break;
       case RURIGHT_BACKWARD:       wheel_r_vel = -velocity * SPEED_ADD_ON;
                                    joint_f_angle = -204;
                                    break;

       default:                     wheel_r_vel = 0;
                                    break;
     }
   }

   int showMode()
   {
     return (int)mobile_mode;
   }

   int* setJointAngle()
   {
     if (joint_f_angle < -204) joint_f_angle = -204;
     else if (joint_f_angle > 204) joint_f_angle = 204;

     joint_angle[0] = JOINT_F;
     joint_angle[1] = joint_f_angle + 2868; // 2664 ~ 3072

     return joint_angle;
   }

   int* setWheelVel()
   {
     wheel_vel[0] = WHEEL_R;
     wheel_vel[1] = wheel_r_vel;

     return wheel_vel;
   }
};

RC100 Controller;
int rcData = 0;

Dynamixel Dxl;

DynamixelStatus dynamixelStatus;

void setup()
{
  Serial.begin(57600);

  Controller.begin(1);

  Dxl.begin(3);
  Dxl.writeByte(WHEEL_R, ADDR_TORQUE_ENABLE, ON);
  Dxl.writeByte(JOINT_F, ADDR_TORQUE_ENABLE, ON);

  Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, dynamixelStatus.setWheelVel(), 2);
  Dxl.syncWrite(ADDR_GOAL_POSITION, 1, dynamixelStatus.setJointAngle(), 2);
}

void loop()
{
  if (Controller.available())
  {
    rcData = Controller.readData();
    Serial.print("Direction : ");
    dynamixelStatus.getDirection(rcData);
    Serial.println(dynamixelStatus.showMode());
    delay(1);

    dynamixelStatus.setParams();
    Dxl.syncWrite(ADDR_GOAL_POSITION, 1, dynamixelStatus.setJointAngle(), 2);
    Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, dynamixelStatus.setWheelVel(), 2);

    Serial.print("aaaa : ");
    Serial.println(dynamixelStatus.distance_from_center);
  }
}
