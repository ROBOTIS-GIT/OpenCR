#include <RC100.h>
#include <Dynamixel.h>

#define WHEEL_L_R 1
#define WHEEL_R_R 2
#define WHEEL_L_F 3
#define WHEEL_R_F 4

#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104

#define DYNAMIXEL_POWER_ENABLE_PIN 32

#define ON  1
#define OFF 0

#define BODY_LENGTH           22.0
#define SPEED_ADD_ON          2

class DynamixelStatus
{
 private:
   int velocity = 200;
  //  double distance_from_center = 25.6;

   int wheel_vel[8] = {0, };

   enum MODE
   {
     NONE = 0
     , FORWARD, BACKWARD, RLEFT, RRIGHT
     , RLEFT_FORWARD, RRIGHT_FORWARD, RLEFT_BACKWARD, RRIGHT_BACKWARD
   } mobile_mode;

 public:
   double distance_from_center = 25.6;

   int wheel_l_r_vel, wheel_r_r_vel, wheel_l_f_vel, wheel_r_f_vel;
   int joint_l_r_angle, joint_r_r_angle, joint_l_f_angle, joint_r_f_angle;

   DynamixelStatus()
   : wheel_l_r_vel(0), wheel_r_r_vel(0), wheel_l_f_vel(0), wheel_r_f_vel(0)
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

      default: if (rcData & RC100_BTN_1) { if (velocity <= 245) velocity += 5; }
                else if (rcData & RC100_BTN_2) { if (distance_from_center >= (12.8 * sqrt(2.0) + 2.0)) distance_from_center -= 2.0; }
                else if (rcData & RC100_BTN_3) { if (velocity >= 5) velocity -= 5; }
                else if (rcData & RC100_BTN_4) { if (distance_from_center <= 49.2) distance_from_center += 2.0; }
                else { mobile_mode = NONE; }
                break;
     }

     return mobile_mode;
   }

   void setParams()
   {
     switch (mobile_mode)
     {
       case FORWARD:                wheel_l_r_vel = velocity * SPEED_ADD_ON; wheel_r_r_vel = -velocity * SPEED_ADD_ON; wheel_l_f_vel = velocity * SPEED_ADD_ON; wheel_r_f_vel = -velocity * SPEED_ADD_ON;
                                    break;
       case BACKWARD:               wheel_l_r_vel = -velocity * SPEED_ADD_ON; wheel_r_r_vel = velocity * SPEED_ADD_ON; wheel_l_f_vel = -velocity * SPEED_ADD_ON; wheel_r_f_vel = velocity * SPEED_ADD_ON;
                                    break;
       case RLEFT:                  wheel_l_r_vel = -velocity; wheel_r_r_vel = -velocity; wheel_l_f_vel = -velocity; wheel_r_f_vel = -velocity;
                                    break;
       case RRIGHT:                 wheel_l_r_vel = velocity; wheel_r_r_vel = velocity; wheel_l_f_vel = velocity; wheel_r_f_vel = velocity;
                                    break;

       case RLEFT_FORWARD:          wheel_l_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_r_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_l_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_r_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    break;
       case RRIGHT_FORWARD:         wheel_l_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_r_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_l_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_r_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    break;
       case RLEFT_BACKWARD:         wheel_l_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_r_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_l_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_r_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    break;
       case RRIGHT_BACKWARD:        wheel_l_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_r_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_l_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_r_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    break;

       default:                     wheel_l_r_vel = 0; wheel_r_r_vel = 0; wheel_l_f_vel = 0; wheel_r_f_vel = 0;
                                    break;
     }
   }

   int showMode()
   {
     return (int)mobile_mode;
   }

   int* setWheelVel()
   {
     wheel_vel[0] = WHEEL_L_R;
     wheel_vel[1] = wheel_l_r_vel;
     wheel_vel[2] = WHEEL_R_R;
     wheel_vel[3] = wheel_r_r_vel;
     wheel_vel[4] = WHEEL_L_F;
     wheel_vel[5] = wheel_l_f_vel;
     wheel_vel[6] = WHEEL_R_F;
     wheel_vel[7] = wheel_r_f_vel;

     return wheel_vel;
   }
};

RC100 Controller;
int rcData = 0;

Dynamixel Dxl;
int vel[4] = {WHEEL_L_R, 0, WHEEL_R_R, 0};
int const_vel = 150;

DynamixelStatus dynamixelStatus;

void setup()
{
  Serial.begin(57600);

  Controller.begin(1);

  Dxl.begin(3);
  Dxl.writeByte(WHEEL_L_R, ADDR_TORQUE_ENABLE, ON);
  Dxl.writeByte(WHEEL_R_R, ADDR_TORQUE_ENABLE, ON);
  Dxl.writeByte(WHEEL_L_F, ADDR_TORQUE_ENABLE, ON);
  Dxl.writeByte(WHEEL_R_F, ADDR_TORQUE_ENABLE, ON);

  Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, dynamixelStatus.setWheelVel(), 8);
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
    Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, dynamixelStatus.setWheelVel(), 8);

    Serial.print("aaaa : ");
    Serial.println(dynamixelStatus.distance_from_center);

    // Serial.print("rcData = ");
    // Serial.print(rcData);
    // Serial.print(" LEFT_VEL = ");
    // Serial.print(vel[1]);
    // Serial.print(" RIGHT_VEL = ");
    // Serial.println(vel[3]);

    // dynamixelStatus.getDirection(rcData);
    //
    // if(rcData & RC100_BTN_U)
    // {
    //   vel[1] += velocity;
    //   vel[3] -= velocity;
    // }
    // else if(rcData & RC100_BTN_D)
    // {
    //   vel[1] -= velocity;
    //   vel[3] += velocity;
    // }
    // else if(rcData & RC100_BTN_L)
    // {
    //   vel[1] -= velocity;
    //   vel[3] -= velocity;
    // }
    // else if(rcData & RC100_BTN_R)
    // {
    //   vel[1] += velocity;
    //   vel[3] += velocity;
    // }
    // else if(rcData & RC100_BTN_1)
    // {
    //   const_vel += 10;
    // }
    // else if(rcData & RC100_BTN_3)
    // {
    //   const_vel -= 10;
    // }
    // else if(rcData & RC100_BTN_6)
    // {
    //   vel[1] = const_vel;
    //   vel[3] = -const_vel;
    // }
    // else if(rcData & RC100_BTN_5)
    // {
    //   vel[1] = 0;
    //   vel[3] = 0;
    // }
    // Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, vel, 4);
  }
}
