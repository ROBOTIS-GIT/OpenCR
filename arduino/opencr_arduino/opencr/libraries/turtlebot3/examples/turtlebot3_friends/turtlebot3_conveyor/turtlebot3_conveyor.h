#include <RC100.h>

#define BODY_LENGTH           25.6
#define SPEED_ADD_ON          2

class DynamixelStatus
{
 private:
   int velocity = 200;

   int32_t wheel_vel[4] = {0, };
   int32_t joint_angle[4] = {0, };

   enum MODE
   {
     NONE = 0
     , FORWARD, BACKWARD, RLEFT, RRIGHT, PLEFT, PRIGHT
     , PLEFT_FORWARD, PRIGHT_FORWARD, PLEFT_BACKWARD, PRIGHT_BACKWARD, RLEFT_FORWARD, RRIGHT_FORWARD, RLEFT_BACKWARD, RRIGHT_BACKWARD, RLEFT_PLEFT, RLEFT_PRIGHT, RRIGHT_PLEFT, RRIGHT_PRIGHT
     , RLEFT_PLEFT_FORWARD, RLEFT_PLEFT_BACKWARD, RRIGHT_PLEFT_FORWARD, RRIGHT_PLEFT_BACKWARD, RLEFT_PRIGHT_FORWARD, RLEFT_PRIGHT_BACKWARD, RRIGHT_PRIGHT_FORWARD, RRIGHT_PRIGHT_BACKWARD
   } mobile_mode;

 public:
   double distance_from_center = 25.6;

   int wheel_l_r_vel, wheel_r_r_vel, wheel_l_f_vel, wheel_r_f_vel;
   int joint_l_r_angle, joint_r_r_angle, joint_l_f_angle, joint_r_f_angle;

   DynamixelStatus()
   : wheel_l_r_vel(0), wheel_r_r_vel(0), wheel_l_f_vel(0), wheel_r_f_vel(0)
   , joint_l_r_angle(0), joint_r_r_angle(0), joint_l_f_angle(0), joint_r_f_angle(0)
   { }

   MODE getDirection(int rcData)
   {
     switch (rcData)
     {
      case (RC100_BTN_U): mobile_mode = FORWARD; break;
      case (RC100_BTN_D): mobile_mode = BACKWARD; break;
      case (RC100_BTN_L): mobile_mode = RLEFT; break;
      case (RC100_BTN_R): mobile_mode = RRIGHT; break;
      case (RC100_BTN_2): mobile_mode = PLEFT; break;
      case (RC100_BTN_4): mobile_mode = PRIGHT; break;

      case (RC100_BTN_U + RC100_BTN_1): mobile_mode = PLEFT_FORWARD; break;
      case (RC100_BTN_U + RC100_BTN_3): mobile_mode = PRIGHT_FORWARD; break;
      case (RC100_BTN_D + RC100_BTN_1): mobile_mode = PLEFT_BACKWARD; break;
      case (RC100_BTN_D + RC100_BTN_3): mobile_mode = PRIGHT_BACKWARD; break;
      case (RC100_BTN_U + RC100_BTN_L): mobile_mode = RLEFT_FORWARD; break;
      case (RC100_BTN_U + RC100_BTN_R): mobile_mode = RRIGHT_FORWARD; break;
      case (RC100_BTN_D + RC100_BTN_L): mobile_mode = RLEFT_BACKWARD; break;
      case (RC100_BTN_D + RC100_BTN_R): mobile_mode = RRIGHT_BACKWARD; break;
      case (RC100_BTN_L + RC100_BTN_6): mobile_mode = RLEFT_PLEFT; break;
      case (RC100_BTN_L + RC100_BTN_5): mobile_mode = RLEFT_PRIGHT; break;
      case (RC100_BTN_R + RC100_BTN_6): mobile_mode = RRIGHT_PLEFT; break;
      case (RC100_BTN_R + RC100_BTN_5): mobile_mode = RRIGHT_PRIGHT; break;

      case (RC100_BTN_L + RC100_BTN_6 + RC100_BTN_U): mobile_mode = RLEFT_PLEFT_FORWARD; break;
      case (RC100_BTN_L + RC100_BTN_6 + RC100_BTN_D): mobile_mode = RLEFT_PLEFT_BACKWARD; break;
      case (RC100_BTN_R + RC100_BTN_6 + RC100_BTN_U): mobile_mode = RRIGHT_PLEFT_FORWARD; break;
      case (RC100_BTN_R + RC100_BTN_6 + RC100_BTN_D): mobile_mode = RRIGHT_PLEFT_BACKWARD; break;
      case (RC100_BTN_L + RC100_BTN_5 + RC100_BTN_U): mobile_mode = RLEFT_PRIGHT_FORWARD; break;
      case (RC100_BTN_L + RC100_BTN_5 + RC100_BTN_D): mobile_mode = RLEFT_PRIGHT_BACKWARD; break;
      case (RC100_BTN_R + RC100_BTN_5 + RC100_BTN_U): mobile_mode = RRIGHT_PRIGHT_FORWARD; break;
      case (RC100_BTN_R + RC100_BTN_5 + RC100_BTN_D): mobile_mode = RRIGHT_PRIGHT_BACKWARD; break;

      default: 
                // if (rcData & RC100_BTN_1) { if (velocity <= 245) velocity += 5; }
                if (rcData & RC100_BTN_2) { if (distance_from_center >= (12.8 * sqrt(2.0) + 2.0)) distance_from_center -= 2.0; }
                // else if (rcData & RC100_BTN_3) { if (velocity >= 5) velocity -= 5; }
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
       case FORWARD:                wheel_l_r_vel = -velocity * SPEED_ADD_ON; wheel_r_r_vel = velocity * SPEED_ADD_ON; wheel_l_f_vel = -velocity * SPEED_ADD_ON; wheel_r_f_vel = velocity * SPEED_ADD_ON;
                                    joint_l_r_angle = 512; joint_r_r_angle = -512; joint_l_f_angle = -512; joint_r_f_angle = 512;
                                    break;
       case BACKWARD:               wheel_l_r_vel = velocity * SPEED_ADD_ON; wheel_r_r_vel = -velocity * SPEED_ADD_ON; wheel_l_f_vel = velocity * SPEED_ADD_ON; wheel_r_f_vel = -velocity * SPEED_ADD_ON;
                                    joint_l_r_angle = 512; joint_r_r_angle = -512; joint_l_f_angle = -512; joint_r_f_angle = 512;
                                    break;
       case RLEFT:                  wheel_l_r_vel = velocity; wheel_r_r_vel = velocity; wheel_l_f_vel = velocity; wheel_r_f_vel = velocity;
                                    joint_l_r_angle = 0; joint_r_r_angle = 0; joint_l_f_angle = 0; joint_r_f_angle = 0;
                                    break;
       case RRIGHT:                 wheel_l_r_vel = -velocity; wheel_r_r_vel = -velocity; wheel_l_f_vel = -velocity; wheel_r_f_vel = -velocity;
                                    joint_l_r_angle = 0; joint_r_r_angle = 0; joint_l_f_angle = 0; joint_r_f_angle = 0;
                                    break;
       case PLEFT:                  wheel_l_r_vel = -velocity * SPEED_ADD_ON; wheel_r_r_vel = -velocity * SPEED_ADD_ON; wheel_l_f_vel = velocity * SPEED_ADD_ON; wheel_r_f_vel = velocity * SPEED_ADD_ON;
                                    joint_l_r_angle = -512; joint_r_r_angle = 512; joint_l_f_angle = 512; joint_r_f_angle = -512;
                                    break;
       case PRIGHT:                 wheel_l_r_vel = velocity * SPEED_ADD_ON; wheel_r_r_vel = velocity * SPEED_ADD_ON; wheel_l_f_vel = -velocity * SPEED_ADD_ON; wheel_r_f_vel = -velocity * SPEED_ADD_ON;
                                    joint_l_r_angle = -512; joint_r_r_angle = 512; joint_l_f_angle = 512; joint_r_f_angle = -512;
                                    break;

       case PLEFT_FORWARD:          wheel_l_r_vel = -velocity * SPEED_ADD_ON; wheel_r_r_vel = -velocity * SPEED_ADD_ON; wheel_l_f_vel = velocity * SPEED_ADD_ON; wheel_r_f_vel = velocity * SPEED_ADD_ON;
                                    joint_l_r_angle = 0; joint_r_r_angle = 1024; joint_l_f_angle = 1024; joint_r_f_angle = 0;
                                    break;
       case PRIGHT_FORWARD:         wheel_l_r_vel = velocity * SPEED_ADD_ON; wheel_r_r_vel = velocity * SPEED_ADD_ON; wheel_l_f_vel = -velocity * SPEED_ADD_ON; wheel_r_f_vel = -velocity * SPEED_ADD_ON;
                                    joint_l_r_angle = -1024; joint_r_r_angle = 0; joint_l_f_angle = 0; joint_r_f_angle = -1024;
                                    break;
       case PLEFT_BACKWARD:         wheel_l_r_vel = -velocity * SPEED_ADD_ON; wheel_r_r_vel = -velocity * SPEED_ADD_ON; wheel_l_f_vel = velocity * SPEED_ADD_ON; wheel_r_f_vel = velocity * SPEED_ADD_ON;
                                    joint_l_r_angle = -1024; joint_r_r_angle = 0; joint_l_f_angle = 0; joint_r_f_angle = -1024;
                                    break;
       case PRIGHT_BACKWARD:        wheel_l_r_vel = velocity * SPEED_ADD_ON; wheel_r_r_vel = velocity * SPEED_ADD_ON; wheel_l_f_vel = -velocity * SPEED_ADD_ON; wheel_r_f_vel = -velocity * SPEED_ADD_ON;
                                    joint_l_r_angle = 0; joint_r_r_angle = 1024; joint_l_f_angle = 1024; joint_r_f_angle = 0;
                                    break;
       case RLEFT_FORWARD:          wheel_l_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_r_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_l_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_r_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    joint_l_r_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_r_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    joint_l_f_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_f_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    break;
       case RRIGHT_FORWARD:         wheel_l_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_r_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_l_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_r_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    joint_l_r_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_r_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    joint_l_f_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_f_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    break;
       case RLEFT_BACKWARD:         wheel_l_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_r_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_l_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_r_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    joint_l_r_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_r_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    joint_l_f_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_f_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    break;
       case RRIGHT_BACKWARD:        wheel_l_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_r_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_l_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_r_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    joint_l_r_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_r_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    joint_l_f_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_f_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    break;
       case RLEFT_PLEFT:            wheel_l_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_r_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_l_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_r_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    joint_l_r_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_r_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    joint_l_f_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_f_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    break;
       case RLEFT_PRIGHT:           wheel_l_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_r_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_l_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_r_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    joint_l_r_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_r_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    joint_l_f_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_f_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    break;
       case RRIGHT_PLEFT:           wheel_l_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_r_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_l_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_r_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    joint_l_r_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_r_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    joint_l_f_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_f_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    break;
       case RRIGHT_PRIGHT:          wheel_l_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_r_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                    wheel_l_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    wheel_r_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                    joint_l_r_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_r_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                    joint_l_f_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    joint_r_f_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                    break;

       case RLEFT_PLEFT_FORWARD:    wheel_l_r_vel = -(int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    wheel_r_r_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    wheel_l_f_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    wheel_r_f_vel = (int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    joint_l_r_angle = 0;
                                    joint_r_r_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    joint_l_f_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    joint_r_f_angle = 0;
                                    break;
       case RLEFT_PLEFT_BACKWARD:   wheel_l_r_vel = (int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    wheel_r_r_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    wheel_l_f_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    wheel_r_f_vel = -(int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    joint_l_r_angle = 0;
                                    joint_r_r_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    joint_l_f_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    joint_r_f_angle = 0;
                                    break;
       case RRIGHT_PLEFT_FORWARD:   wheel_l_r_vel = -(int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    wheel_r_r_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    wheel_l_f_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    wheel_r_f_vel = (int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    joint_l_r_angle = 0;
                                    joint_r_r_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    joint_l_f_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    joint_r_f_angle = 0;
                                    break;
       case RRIGHT_PLEFT_BACKWARD:  wheel_l_r_vel = (int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    wheel_r_r_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    wheel_l_f_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    wheel_r_f_vel = -(int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    joint_l_r_angle = 0;
                                    joint_r_r_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    joint_l_f_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    joint_r_f_angle = 0;
                                    break;
       case RLEFT_PRIGHT_FORWARD:   wheel_l_r_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    wheel_r_r_vel = (int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    wheel_l_f_vel = -(int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    wheel_r_f_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    joint_l_r_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    joint_r_r_angle = 0;
                                    joint_l_f_angle = 0;
                                    joint_r_f_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    break;
       case RLEFT_PRIGHT_BACKWARD:  wheel_l_r_vel = (int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    wheel_r_r_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    wheel_l_f_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    wheel_r_f_vel = -(int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    joint_l_r_angle = 0;
                                    joint_r_r_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    joint_l_f_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    joint_r_f_angle = 0;
                                    break;
       case RRIGHT_PRIGHT_FORWARD:  wheel_l_r_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    wheel_r_r_vel = (int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    wheel_l_f_vel = -(int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    wheel_r_f_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    joint_l_r_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    joint_r_r_angle = 0;
                                    joint_l_f_angle = 0;
                                    joint_r_f_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    break;
       case RRIGHT_PRIGHT_BACKWARD: wheel_l_r_vel = (int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    wheel_r_r_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    wheel_l_f_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                    wheel_r_f_vel = -(int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                    joint_l_r_angle = 0;
                                    joint_r_r_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    joint_l_f_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                    joint_r_f_angle = 0;
                                    break;

       default:                     wheel_l_r_vel = 0; wheel_r_r_vel = 0; wheel_l_f_vel = 0; wheel_r_f_vel = 0;
                                    break;
     }
   }

   int showMode()
   {
     return (int)mobile_mode;
   }

   int32_t* setJointAngle()
   {
     if (joint_l_r_angle < -1024) joint_l_r_angle = -1024;
     else if (joint_l_r_angle > 1024) joint_l_r_angle = 1024;
     if (joint_r_r_angle < -1024) joint_r_r_angle = -1024;
     else if (joint_r_r_angle > 1024) joint_r_r_angle = 1024;
     if (joint_l_f_angle < -1024) joint_l_f_angle = -1024;
     else if (joint_l_f_angle > 1024) joint_l_f_angle = 1024;
     if (joint_r_f_angle < -1024) joint_r_f_angle = -1024;
     else if (joint_r_f_angle > 1024) joint_r_f_angle = 1024;

     joint_angle[0] = joint_l_r_angle + 2048;
     joint_angle[1] = joint_r_r_angle + 2048;
     joint_angle[2] = joint_l_f_angle + 2048;
     joint_angle[3] = joint_r_f_angle + 2048;

    //  joint_angle[0] = JOINT_L_R;
    //  joint_angle[1] = joint_l_r_angle + 2048;
    //  joint_angle[2] = JOINT_R_R;
    //  joint_angle[3] = joint_r_r_angle + 2048;
    //  joint_angle[4] = JOINT_L_F;
    //  joint_angle[5] = joint_l_f_angle + 2048;
    //  joint_angle[6] = JOINT_R_F;
    //  joint_angle[7] = joint_r_f_angle + 2048;

     return joint_angle;
   }

   int32_t* setWheelVel()
   {
     wheel_vel[0] = wheel_l_r_vel;
     wheel_vel[1] = wheel_r_r_vel;
     wheel_vel[2] = wheel_l_f_vel;
     wheel_vel[3] = wheel_r_f_vel;

    //  wheel_vel[0] = WHEEL_L_R;
    //  wheel_vel[1] = wheel_l_r_vel;
    //  wheel_vel[2] = WHEEL_R_R;
    //  wheel_vel[3] = wheel_r_r_vel;
    //  wheel_vel[4] = WHEEL_L_F;
    //  wheel_vel[5] = wheel_l_f_vel;
    //  wheel_vel[6] = WHEEL_R_F;
    //  wheel_vel[7] = wheel_r_f_vel;

     return wheel_vel;
   }
};