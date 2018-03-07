#include <RC100.h>
#include <Dynamixel.h>
#include <DynamixelSDK.h>

#define LEG_L_R 1
#define LEG_R_R 2
#define LEG_L_F 3
#define LEG_R_F 4
#define SHOULDER_L_R 5
#define SHOULDER_R_R 6
#define SHOULDER_L_F 7
#define SHOULDER_R_F 8
#define HEAD_YAW 9
#define HEAD_PITCH 10

#define ADDR_TORQUE_ENABLE 64
#define ADDR_PROFILE_ACCELERATION 108
#define ADDR_PROFILE_VELOCITY 112
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

#define DYNAMIXEL_POWER_ENABLE_PIN 32

#define ON  1
#define OFF 0

#define DEVICENAME        ""
#define PROTOCOL_VERSION 2.0
#define BAUDRATE          1000000

#define BODY_LENGTH           25.6
#define SPEED_ADD_ON          2
#define MOTION_NUM            20

#define FASTER            4.0

class DynamixelStatus
{
 private:
   int velocity = 200;
  //  double distance_from_center = 25.6;

   int joint_angle[8] = {0, };

   enum MODE
   {
     NONE = 0
     , FORWARD1, FORWARD2, FORWARD3, FORWARD4
     , STANDING_POSITION2, STANDING_POSITION3
     , GRUMBLE1, GRUMBLE2, RLEFT, RRIGHT, PLEFT, PRIGHT
     , LOOK_AROUND_LEFT, LOOK_AROUND_RIGHT
     , PLEFT_FORWARD, PRIGHT_FORWARD, PLEFT_BACKWARD, PRIGHT_BACKWARD
     , RLEFT_FORWARD2, RLEFT_FORWARD3
     , RRIGHT_FORWARD2, RRIGHT_FORWARD3
     , RLEFT_BACKWARD, RRIGHT_BACKWARD, RLEFT_PLEFT, RLEFT_PRIGHT, RRIGHT_PLEFT, RRIGHT_PRIGHT
   } mobile_mode, pres_mobile_mode;

 public:
   double distance_from_center = 25.6;

   int shoulder_l_r_angle[MOTION_NUM] = {0, }, shoulder_r_r_angle[MOTION_NUM] = {0, }, shoulder_l_f_angle[MOTION_NUM] = {0, }, shoulder_r_f_angle[MOTION_NUM] = {0, };
   int leg_l_r_angle[MOTION_NUM] = {0, }, leg_r_r_angle[MOTION_NUM] = {0, }, leg_l_f_angle[MOTION_NUM] = {0, }, leg_r_f_angle[MOTION_NUM] = {0, };
   int head_yaw = 2048, head_pitch = 3072; // 1848 ~ 2248 : 3072 ~ 2560
   int motion_all_num;
   double time_duration[MOTION_NUM] = {0.0, }, time_space[MOTION_NUM] = {0.0, };

   DynamixelStatus()
   : motion_all_num(1)
   { }

   MODE getDirection(int rcData)
   {
     switch (rcData)
     {
      case (RC100_BTN_U): if (pres_mobile_mode == NONE) { mobile_mode = FORWARD1; }
                          // else if (pres_mobile_mode == FORWARD1) { mobile_mode = FORWARD1; }
                          else if (pres_mobile_mode == FORWARD1) { mobile_mode = FORWARD2; }
                          else if (pres_mobile_mode == FORWARD2) { mobile_mode = FORWARD3; }
                          else if (pres_mobile_mode == FORWARD3) { mobile_mode = FORWARD2; }
                          // else if (pres_mobile_mode == FORWARD4) { mobile_mode = FORWARD1; }
                          pres_mobile_mode = mobile_mode;
                          break;
      // case (RC100_BTN_D): if (pres_mobile_mode == NONE) { mobile_mode = GRUMBLE1; }
      //                     // else if (pres_mobile_mode == FORWARD1) { mobile_mode = FORWARD1; }
      //                     // else if (pres_mobile_mode == FORWARD1) { mobile_mode = FORWARD2; }
      //                     // else if (pres_mobile_mode == FORWARD2) { mobile_mode = FORWARD3; }
      //                     // else if (pres_mobile_mode == FORWARD3) { mobile_mode = FORWARD2; }
      //                     // else if (pres_mobile_mode == FORWARD4) { mobile_mode = FORWARD1; }
      //                     pres_mobile_mode = mobile_mode;
      //                     break;
      case (RC100_BTN_L): if (pres_mobile_mode == NONE) { mobile_mode = RLEFT; }
                          else if (pres_mobile_mode == FORWARD2) { mobile_mode = STANDING_POSITION2; }
                          else if (pres_mobile_mode == FORWARD3) { mobile_mode = STANDING_POSITION3; }
                          pres_mobile_mode = mobile_mode;
                          break;
      case (RC100_BTN_R): if (pres_mobile_mode == NONE) { mobile_mode = RRIGHT; }
                          else if (pres_mobile_mode == FORWARD2) { mobile_mode = STANDING_POSITION2; }
                          else if (pres_mobile_mode == FORWARD3) { mobile_mode = STANDING_POSITION3; }
                          pres_mobile_mode = mobile_mode;
                          break;
      case (RC100_BTN_2): if (head_yaw < 2228) // 2248
                          {
                            head_yaw += 20;
                          }
                          break;
      case (RC100_BTN_4): if (head_yaw > 1868) // 1848
                          {
                            head_yaw -= 20;
                          }
                          break;
      case (RC100_BTN_1): if (head_pitch > 2580) // 2560
                          {
                            head_pitch -= 20;
                          }
                          break;
      case (RC100_BTN_3): if (head_pitch < 3052) // 3072
                          {
                            head_pitch += 20;
                          }
                          break;
      case (RC100_BTN_6): if (pres_mobile_mode == NONE) { mobile_mode = LOOK_AROUND_LEFT; }
                          pres_mobile_mode = mobile_mode;
                          break;
      case (RC100_BTN_5): if (pres_mobile_mode == NONE) { mobile_mode = LOOK_AROUND_RIGHT; }
                          pres_mobile_mode = mobile_mode;
                          break;

      case (RC100_BTN_U + RC100_BTN_L): if (pres_mobile_mode == NONE) { mobile_mode = FORWARD1; }
                          else if (pres_mobile_mode == FORWARD1) { mobile_mode = RLEFT_FORWARD3; }
                          else if (pres_mobile_mode == FORWARD2) { mobile_mode = RLEFT_FORWARD2; }
                          else if (pres_mobile_mode == FORWARD3) { mobile_mode = RLEFT_FORWARD3; }
                          pres_mobile_mode = mobile_mode;
                          break;

      case (RC100_BTN_U + RC100_BTN_R): if (pres_mobile_mode == NONE) { mobile_mode = FORWARD1; }
                          else if (pres_mobile_mode == FORWARD1) { mobile_mode = RRIGHT_FORWARD3; }
                          else if (pres_mobile_mode == FORWARD2) { mobile_mode = RRIGHT_FORWARD2; }
                          else if (pres_mobile_mode == FORWARD3) { mobile_mode = RRIGHT_FORWARD3; }
                          pres_mobile_mode = mobile_mode;
                          break;


      case (RC100_BTN_U + RC100_BTN_D): if (pres_mobile_mode == NONE) { mobile_mode = NONE; }
                                        else if (pres_mobile_mode == FORWARD2) { mobile_mode = STANDING_POSITION2; }
                                        else if (pres_mobile_mode == FORWARD3) { mobile_mode = STANDING_POSITION3; }
                                        else if ((pres_mobile_mode == STANDING_POSITION2)
                                                 || (pres_mobile_mode == STANDING_POSITION3)) { mobile_mode = NONE; }
                                        pres_mobile_mode = mobile_mode;
                                        break;

      case (RC100_BTN_D + RC100_BTN_1): if (pres_mobile_mode == NONE) { mobile_mode = GRUMBLE1; }
                                        pres_mobile_mode = mobile_mode;
                                        break;
      case (RC100_BTN_D + RC100_BTN_2): if (pres_mobile_mode == NONE) { mobile_mode = GRUMBLE2; }
                                        pres_mobile_mode = mobile_mode;
                                        break;



      // case (RC100_BTN_U + RC100_BTN_6): mobile_mode = PLEFT_FORWARD; break;
      // case (RC100_BTN_U + RC100_BTN_5): mobile_mode = PRIGHT_FORWARD; break;
      // case (RC100_BTN_D + RC100_BTN_6): mobile_mode = PLEFT_BACKWARD; break;
      // case (RC100_BTN_D + RC100_BTN_5): mobile_mode = PRIGHT_BACKWARD; break;
      // case (RC100_BTN_U + RC100_BTN_L): mobile_mode = RLEFT_FORWARD; break;
      // case (RC100_BTN_U + RC100_BTN_R): mobile_mode = RRIGHT_FORWARD; break;
      // case (RC100_BTN_D + RC100_BTN_L): mobile_mode = RLEFT_BACKWARD; break;
      // case (RC100_BTN_D + RC100_BTN_R): mobile_mode = RRIGHT_BACKWARD; break;
      // case (RC100_BTN_L + RC100_BTN_6): mobile_mode = RLEFT_PLEFT; break;
      // case (RC100_BTN_L + RC100_BTN_5): mobile_mode = RLEFT_PRIGHT; break;
      // case (RC100_BTN_R + RC100_BTN_6): mobile_mode = RRIGHT_PLEFT; break;
      // case (RC100_BTN_R + RC100_BTN_5): mobile_mode = RRIGHT_PRIGHT; break;

      default: mobile_mode = NONE;
               pres_mobile_mode = mobile_mode;
               break;
      // default: if (rcData & RC100_BTN_1) { if (velocity <= 245) velocity += 5; }
      //           else if (rcData & RC100_BTN_2) { if (distance_from_center >= (12.8 * sqrt(2.0) + 2.0)) distance_from_center -= 2.0; }
      //           else if (rcData & RC100_BTN_3) { if (velocity >= 5) velocity -= 5; }
      //           else if (rcData & RC100_BTN_4) { if (distance_from_center <= 49.2) distance_from_center += 2.0; }
      //           else { mobile_mode = NONE; }
      //           break;
     }

     return mobile_mode;
   }

   void setParams()
   {
     switch (mobile_mode)
     {
       case FORWARD1:               motion_all_num = 8;
                                  // for Smooth Motion
                                  //  leg_l_r_angle[0] = 0;       leg_r_r_angle[0] = 0;         leg_l_f_angle[0] = 0;         leg_r_f_angle[0] = 0;
                                  //  time_duration[0] = 0.2;       time_space[0] = .0;
                                  //  // shoulder_l_r_angle[1] = 0;    shoulder_r_r_angle[1] = 0;    shoulder_l_f_angle[1] = 0;    shoulder_r_f_angle[1] = 0;
                                  //  // leg_l_r_angle[1] = 0;       leg_r_r_angle[1] = 0;         leg_l_f_angle[1] = 0;         leg_r_f_angle[1] = 0;
                                  //  // time_duration[1] = 0.8;       time_space[1] = .0;
                                  //  shoulder_l_r_angle[1] = -50;    shoulder_r_r_angle[1] = 50;    shoulder_l_f_angle[1] = -50;    shoulder_r_f_angle[1] = 50;
                                  //  leg_l_r_angle[1] = 256;       leg_r_r_angle[1] = 0;         leg_l_f_angle[1] = 0;         leg_r_f_angle[1] = 0;
                                  //  time_duration[1] = 0.1;       time_space[1] = .0;
                                  //  shoulder_l_r_angle[2] = 600;  shoulder_r_r_angle[2] = 100;    shoulder_l_f_angle[2] = -100;    shoulder_r_f_angle[2] = 100;
                                  //  leg_l_r_angle[2] = 512;       leg_r_r_angle[2] = 0;         leg_l_f_angle[2] = 0;         leg_r_f_angle[2] = 0;
                                  //  time_duration[2] = 0.1;       time_space[2] = .0;
                                  //  shoulder_l_r_angle[3] = 550;  shoulder_r_r_angle[3] = 150;    shoulder_l_f_angle[3] = -150;    shoulder_r_f_angle[3] = 150;
                                  //  leg_l_r_angle[3] = 0;         leg_r_r_angle[3] = 0;         leg_l_f_angle[3] = 0;         leg_r_f_angle[3] = 0;
                                  //  time_duration[3] = 0.1;       time_space[3] = .0;
                                   //
                                   //
                                  //  shoulder_l_r_angle[4] = 500;  shoulder_r_r_angle[4] = 200;    shoulder_l_f_angle[4] = -200;    shoulder_r_f_angle[4] = 200;
                                  //  leg_l_r_angle[4] = 0;         leg_r_r_angle[4] = 0;         leg_l_f_angle[4] = -512;      leg_r_f_angle[4] = 0;
                                  //  time_duration[4] = 0.1;       time_space[4] = .0;
                                  //  shoulder_l_r_angle[5] = 400;  shoulder_r_r_angle[5] = 300;    shoulder_l_f_angle[5] = 400;    shoulder_r_f_angle[5] = 300;
                                  //  leg_l_r_angle[5] = 0;         leg_r_r_angle[5] = 0;         leg_l_f_angle[5] = -512;      leg_r_f_angle[5] = 0;
                                  //  time_duration[5] = 0.2;       time_space[5] = .0;
                                  //  shoulder_l_r_angle[6] = 300;  shoulder_r_r_angle[6] = 400;    shoulder_l_f_angle[6] = 300;    shoulder_r_f_angle[6] = 400;
                                  //  leg_l_r_angle[6] = 0;         leg_r_r_angle[6] = -256;      leg_l_f_angle[6] = 0;         leg_r_f_angle[6] = 0;
                                  //  time_duration[6] = 0.2;       time_space[6] = .0;
                                  //  shoulder_l_r_angle[7] = 200;  shoulder_r_r_angle[7] = -900; shoulder_l_f_angle[7] = 200;    shoulder_r_f_angle[7] = 500;
                                  //  leg_l_r_angle[7] = 0;         leg_r_r_angle[7] = -512;      leg_l_f_angle[7] = 0;         leg_r_f_angle[7] = 0;
                                  //  time_duration[7] = 0.2;       time_space[7] = .0;
                                  //  shoulder_l_r_angle[8] = 100;  shoulder_r_r_angle[8] = -800; shoulder_l_f_angle[8] = 100;    shoulder_r_f_angle[8] = 600;
                                  //  leg_l_r_angle[8] = 0;         leg_r_r_angle[8] = 0;         leg_l_f_angle[8] = 0;         leg_r_f_angle[8] = 0;
                                  //  time_duration[8] = 0.2;       time_space[8] = .0;
                                  //  shoulder_l_r_angle[9] = 0;    shoulder_r_r_angle[9] = -700; shoulder_l_f_angle[9] = 0;    shoulder_r_f_angle[9] = 700;
                                  //  leg_l_r_angle[9] = 0;         leg_r_r_angle[9] = 0;         leg_l_f_angle[9] = 0;         leg_r_f_angle[9] = 0;
                                  //  time_duration[9] = 0.2;       time_space[9] = .0;

                                  // for Fast Motion
                                    shoulder_l_r_angle[0] = 0;    shoulder_r_r_angle[0] = 0;    shoulder_l_f_angle[0] = 0;    shoulder_r_f_angle[0] = 0;
                                    leg_l_r_angle[0] = 0;         leg_r_r_angle[0] = 0;         leg_l_f_angle[0] = 0;         leg_r_f_angle[0] = 0;
                                    time_duration[0] = 0.4 / FASTER;       time_space[0] = .0;
                                    shoulder_l_r_angle[1] = 0;    shoulder_r_r_angle[1] = 0;    shoulder_l_f_angle[1] = 0;    shoulder_r_f_angle[1] = 0;
                                    leg_l_r_angle[1] = 256;       leg_r_r_angle[1] = 0;         leg_l_f_angle[1] = 0;         leg_r_f_angle[1] = 0;
                                    time_duration[1] = 0.1 / FASTER;       time_space[1] = .0;
                                    shoulder_l_r_angle[2] = 700;  shoulder_r_r_angle[2] = 0;    shoulder_l_f_angle[2] = 0;    shoulder_r_f_angle[2] = 0;
                                    leg_l_r_angle[2] = 512;       leg_r_r_angle[2] = 0;         leg_l_f_angle[2] = 0;         leg_r_f_angle[2] = 0;
                                    time_duration[2] = 0.2 / FASTER;       time_space[2] = .0;
                                    shoulder_l_r_angle[3] = 700;  shoulder_r_r_angle[3] = 0;    shoulder_l_f_angle[3] = 0;    shoulder_r_f_angle[3] = 0;
                                    leg_l_r_angle[3] = 0;         leg_r_r_angle[3] = 0;         leg_l_f_angle[3] = 0;         leg_r_f_angle[3] = 0;
                                    time_duration[3] = 0.1 / FASTER;       time_space[3] = .0;


                                    shoulder_l_r_angle[4] = 700;  shoulder_r_r_angle[4] = 0;    shoulder_l_f_angle[4] = 0;    shoulder_r_f_angle[4] = 0;
                                    leg_l_r_angle[4] = 0;         leg_r_r_angle[4] = 0;         leg_l_f_angle[4] = -512;      leg_r_f_angle[4] = 0;
                                    time_duration[4] = 0.1 / FASTER;       time_space[4] = .0;
                                    shoulder_l_r_angle[5] = 0;    shoulder_r_r_angle[5] = 0;    shoulder_l_f_angle[5] = 0;    shoulder_r_f_angle[5] = 700;
                                    leg_l_r_angle[5] = 0;         leg_r_r_angle[5] = -256;      leg_l_f_angle[5] = 0;         leg_r_f_angle[5] = 0;
                                    time_duration[5] = 0.4 / FASTER;       time_space[5] = .0;
                                    shoulder_l_r_angle[6] = 0;    shoulder_r_r_angle[6] = -700; shoulder_l_f_angle[6] = 0;    shoulder_r_f_angle[6] = 700;
                                    leg_l_r_angle[6] = 0;         leg_r_r_angle[6] = -512;      leg_l_f_angle[6] = 0;         leg_r_f_angle[6] = 0;
                                    time_duration[6] = 0.2 / FASTER;       time_space[6] = .0;
                                    shoulder_l_r_angle[7] = 0;    shoulder_r_r_angle[7] = -700; shoulder_l_f_angle[7] = 0;    shoulder_r_f_angle[7] = 700;
                                    leg_l_r_angle[7] = 0;         leg_r_r_angle[7] = 0;         leg_l_f_angle[7] = 0;         leg_r_f_angle[7] = 0;
                                    time_duration[7] = 0.1 / FASTER;       time_space[7] = .0;

                                    mobile_mode = FORWARD3;
                                    pres_mobile_mode = mobile_mode;
                                    // Serial.print("after FORWARD1  ");
                                    // Serial.println(pres_mobile_mode);

                                    break;
       case FORWARD2:               motion_all_num = 5;
                                    // shoulder_l_r_angle[0] = 0;    shoulder_r_r_angle[0] = -700; shoulder_l_f_angle[0] = 0;    shoulder_r_f_angle[0] = 700;
                                    // leg_l_r_angle[0] = 0;         leg_r_r_angle[0] = 0;         leg_l_f_angle[0] = 0;         leg_r_f_angle[0] = 0;
                                    // time_duration[0] = 0.4 / FASTER;       time_space[0] = .2;
                                    shoulder_l_r_angle[0] = 0;    shoulder_r_r_angle[0] = -700; shoulder_l_f_angle[0] = 0;    shoulder_r_f_angle[0] = 700;
                                    leg_l_r_angle[0] = 0;         leg_r_r_angle[0] = 0;         leg_l_f_angle[0] = 0;         leg_r_f_angle[0] = 256;
                                    time_duration[0] = 0.1 / FASTER;       time_space[0] = .0;
                                    shoulder_l_r_angle[1] = 0;    shoulder_r_r_angle[1] = -700; shoulder_l_f_angle[1] = 0;    shoulder_r_f_angle[1] = 0;
                                    leg_l_r_angle[1] = 0;         leg_r_r_angle[1] = 0;         leg_l_f_angle[1] = 0;         leg_r_f_angle[1] = 512;
                                    time_duration[1] = 0.2 / FASTER;       time_space[1] = .0;
                                    shoulder_l_r_angle[2] = 0;    shoulder_r_r_angle[2] = 0;    shoulder_l_f_angle[2] = -700; shoulder_r_f_angle[2] = 0;
                                    leg_l_r_angle[2] = 512;       leg_r_r_angle[2] = 0;         leg_l_f_angle[2] = 0;         leg_r_f_angle[2] = 0;
                                    time_duration[2] = 0.4 / FASTER;       time_space[2] = .0;
                                    shoulder_l_r_angle[3] = 700;  shoulder_r_r_angle[3] = 0;    shoulder_l_f_angle[3] = -700; shoulder_r_f_angle[3] = 0;
                                    leg_l_r_angle[3] = 512;       leg_r_r_angle[3] = 0;         leg_l_f_angle[3] = 0;         leg_r_f_angle[3] = 0;
                                    time_duration[3] = 0.2 / FASTER;       time_space[3] = .0;
                                    shoulder_l_r_angle[4] = 700;  shoulder_r_r_angle[4] = 0;    shoulder_l_f_angle[4] = -700; shoulder_r_f_angle[4] = 0;
                                    leg_l_r_angle[4] = 0;         leg_r_r_angle[4] = 0;         leg_l_f_angle[4] = 0;         leg_r_f_angle[4] = 0;
                                    time_duration[4] = 0.1 / FASTER;       time_space[4] = .0;
                                    break;
       case FORWARD3:               motion_all_num = 5;
                                    // shoulder_l_r_angle[4] = 700; shoulder_r_r_angle[4] = 0;   shoulder_l_f_angle[4] = -700; shoulder_r_f_angle[4] = 0;
                                    // leg_l_r_angle[4] = 0;      leg_r_r_angle[4] = 0;        leg_l_f_angle[4] = 0;           leg_r_f_angle[4] = 0;
                                    // time_duration[4] = 0.4 / FASTER;      time_space[4] = .2;
                                    shoulder_l_r_angle[0] = 700; shoulder_r_r_angle[0] = 0;     shoulder_l_f_angle[0] = -700; shoulder_r_f_angle[0] = 0;
                                    leg_l_r_angle[0] = 0;        leg_r_r_angle[0] = 0;          leg_l_f_angle[0] = -256;      leg_r_f_angle[0] = 0;
                                    time_duration[0] = 0.1 / FASTER;      time_space[0] = .0;
                                    shoulder_l_r_angle[1] = 700; shoulder_r_r_angle[1] = 0;     shoulder_l_f_angle[1] = 0;    shoulder_r_f_angle[1] = 0;
                                    leg_l_r_angle[1] = 0;        leg_r_r_angle[1] = 0;          leg_l_f_angle[1] = -512;      leg_r_f_angle[1] = 0;
                                    time_duration[1] = 0.2 / FASTER;      time_space[1] = .0;
                                    shoulder_l_r_angle[2] = 0;   shoulder_r_r_angle[2] = 0;     shoulder_l_f_angle[2] = 0;    shoulder_r_f_angle[2] = 700;
                                    leg_l_r_angle[2] = 0;        leg_r_r_angle[2] = -512;       leg_l_f_angle[2] = 0;         leg_r_f_angle[2] = 0;
                                    time_duration[2] = 0.4 / FASTER;      time_space[2] = .0;
                                    shoulder_l_r_angle[3] = 0;   shoulder_r_r_angle[3] = -700;  shoulder_l_f_angle[3] = 0;    shoulder_r_f_angle[3] = 700;
                                    leg_l_r_angle[3] = 0;        leg_r_r_angle[3] = -512;       leg_l_f_angle[3] = 0;         leg_r_f_angle[3] = 0;
                                    time_duration[3] = 0.2 / FASTER;      time_space[3] = .0;
                                    shoulder_l_r_angle[4] = 0;   shoulder_r_r_angle[4] = -700;  shoulder_l_f_angle[4] = 0;    shoulder_r_f_angle[4] = 700;
                                    leg_l_r_angle[4] = 0;        leg_r_r_angle[4] = 0;          leg_l_f_angle[4] = 0;         leg_r_f_angle[4] = 0;
                                    time_duration[4] = 0.1 / FASTER;      time_space[4] = .0;
                                    break;

      case RLEFT_FORWARD2:          motion_all_num = 9;
                                   //  shoulder_l_r_angle[4] = 0;   shoulder_r_r_angle[4] = -700;  shoulder_l_f_angle[4] = 0;    shoulder_r_f_angle[4] = 700;
                                   //  leg_l_r_angle[4] = 0;        leg_r_r_angle[4] = 0;          leg_l_f_angle[4] = 0;         leg_r_f_angle[4] = 0;
                                   //  time_duration[4] = 0.1 / FASTER;      time_space[4] = .0;
                                    shoulder_l_r_angle[0] = 700;    shoulder_r_r_angle[0] = 0; shoulder_l_f_angle[0] = -700;    shoulder_r_f_angle[0] = 0;
                                    leg_l_r_angle[0] = 0;         leg_r_r_angle[0] = 0;         leg_l_f_angle[0] = -256;         leg_r_f_angle[0] = 0;
                                    time_duration[0] = 0.1 / FASTER;       time_space[0] = .0;
                                    shoulder_l_r_angle[1] = 700;    shoulder_r_r_angle[1] = 0; shoulder_l_f_angle[1] = 0;    shoulder_r_f_angle[1] = 0;
                                    leg_l_r_angle[1] = 0;         leg_r_r_angle[1] = 0;         leg_l_f_angle[1] = -512;         leg_r_f_angle[1] = 0;
                                    time_duration[1] = 0.2 / FASTER;       time_space[1] = .0;

                                    shoulder_l_r_angle[2] = 700;    shoulder_r_r_angle[2] = 500;    shoulder_l_f_angle[2] = 0; shoulder_r_f_angle[2] = 700;
                                    leg_l_r_angle[2] = 0;       leg_r_r_angle[2] = 0;         leg_l_f_angle[2] = 0;         leg_r_f_angle[2] = 0;
                                    time_duration[2] = 0.4 / FASTER;       time_space[2] = .0;
                                    shoulder_l_r_angle[3] = 700;    shoulder_r_r_angle[3] = 500;    shoulder_l_f_angle[3] = 0; shoulder_r_f_angle[3] = 700;
                                    leg_l_r_angle[3] = 256;       leg_r_r_angle[3] = 0;         leg_l_f_angle[3] = 0;         leg_r_f_angle[3] = 0;
                                    time_duration[3] = 0.1 / FASTER;       time_space[3] = .0;
                                    shoulder_l_r_angle[4] = 0;  shoulder_r_r_angle[4] = 500;    shoulder_l_f_angle[4] = 0; shoulder_r_f_angle[4] = 700;
                                    leg_l_r_angle[4] = 512;       leg_r_r_angle[4] = 0;         leg_l_f_angle[4] = 0;         leg_r_f_angle[4] = 0;
                                    time_duration[4] = 0.2 / FASTER;       time_space[4] = .0;
                                    shoulder_l_r_angle[5] = 0;  shoulder_r_r_angle[5] = 500;    shoulder_l_f_angle[5] = 0; shoulder_r_f_angle[5] = 700;
                                    leg_l_r_angle[5] = 0;       leg_r_r_angle[5] = 0;         leg_l_f_angle[5] = 0;         leg_r_f_angle[5] = 0;
                                    time_duration[5] = 0.1 / FASTER;       time_space[5] = .0;
                                    shoulder_l_r_angle[6] = 0;  shoulder_r_r_angle[6] = 500;    shoulder_l_f_angle[6] = 0; shoulder_r_f_angle[6] = 700;
                                    leg_l_r_angle[6] = 256;         leg_r_r_angle[6] = 0;         leg_l_f_angle[6] = 0;         leg_r_f_angle[6] = 0;
                                    time_duration[6] = 0.1 / FASTER;       time_space[6] = .0;

                                    shoulder_l_r_angle[7] = 0;  shoulder_r_r_angle[7] = -700;    shoulder_l_f_angle[7] = 0; shoulder_r_f_angle[7] = 700;
                                    leg_l_r_angle[7] = 0;         leg_r_r_angle[7] = -512;         leg_l_f_angle[7] = 0;         leg_r_f_angle[7] = 0;
                                    time_duration[7] = 0.2 / FASTER;       time_space[7] = .0;
                                    shoulder_l_r_angle[8] = 0;  shoulder_r_r_angle[8] = -700;    shoulder_l_f_angle[8] = 0; shoulder_r_f_angle[8] = 700;
                                    leg_l_r_angle[8] = 0;         leg_r_r_angle[8] = 0;         leg_l_f_angle[8] = 0;         leg_r_f_angle[8] = 0;
                                    time_duration[8] = 0.1 / FASTER;       time_space[8] = .0;

                                    mobile_mode = FORWARD3;
                                    pres_mobile_mode = mobile_mode;
                                    break;

      case RLEFT_FORWARD3:          motion_all_num = 6;
                                   //  shoulder_l_r_angle[4] = 700;  shoulder_r_r_angle[4] = 0;    shoulder_l_f_angle[4] = -700; shoulder_r_f_angle[4] = 0;
                                   //  leg_l_r_angle[4] = 0;         leg_r_r_angle[4] = 0;         leg_l_f_angle[4] = 0;         leg_r_f_angle[4] = 0;
                                   //  time_duration[4] = 0.1 / FASTER;       time_space[4] = .0;
                                    shoulder_l_r_angle[0] = 0; shoulder_r_r_angle[0] = -700;     shoulder_l_f_angle[0] = 0; shoulder_r_f_angle[0] = 700;
                                    leg_l_r_angle[0] = 0;        leg_r_r_angle[0] = 0;          leg_l_f_angle[0] = 0;      leg_r_f_angle[0] = 256;
                                    time_duration[0] = 0.1 / FASTER;      time_space[0] = .0;
                                    shoulder_l_r_angle[1] = 0; shoulder_r_r_angle[1] = -700;     shoulder_l_f_angle[1] = 0;    shoulder_r_f_angle[1] = -700;
                                    leg_l_r_angle[1] = 0;        leg_r_r_angle[1] = 0;          leg_l_f_angle[1] = 0;      leg_r_f_angle[1] = 512;
                                    time_duration[1] = 0.2 / FASTER;      time_space[1] = .0;
                                    shoulder_l_r_angle[2] = 0; shoulder_r_r_angle[2] = -700;     shoulder_l_f_angle[2] = 0;    shoulder_r_f_angle[2] = -700;
                                    leg_l_r_angle[2] = 0;        leg_r_r_angle[2] = 0;          leg_l_f_angle[2] = 0;      leg_r_f_angle[2] = 0;
                                    time_duration[2] = 0.1 / FASTER;      time_space[2] = .0;
                                    shoulder_l_r_angle[2 + 1] = 700;   shoulder_r_r_angle[2 + 1] = 0;     shoulder_l_f_angle[2 + 1] = 0;    shoulder_r_f_angle[2 + 1] = 0;
                                    leg_l_r_angle[2 + 1] = 0;        leg_r_r_angle[2 + 1] = 0;       leg_l_f_angle[2 + 1] = -256;         leg_r_f_angle[2 + 1] = 0;
                                    time_duration[2 + 1] = 0.4 / FASTER;      time_space[2 + 1] = .0;
                                    shoulder_l_r_angle[3 + 1] = 700;   shoulder_r_r_angle[3 + 1] = 0;  shoulder_l_f_angle[3 + 1] = -700;    shoulder_r_f_angle[3 + 1] = 0;
                                    leg_l_r_angle[3 + 1] = 0;        leg_r_r_angle[3 + 1] = 0;       leg_l_f_angle[3 + 1] = -512;         leg_r_f_angle[3 + 1] = 0;
                                    time_duration[3 + 1] = 0.2 / FASTER;      time_space[3 + 1] = .0;
                                    shoulder_l_r_angle[4 + 1] = 700;   shoulder_r_r_angle[4 + 1] = 0;  shoulder_l_f_angle[4 + 1] = -700;    shoulder_r_f_angle[4 + 1] = 0;
                                    leg_l_r_angle[4 + 1] = 0;        leg_r_r_angle[4 + 1] = 0;          leg_l_f_angle[4 + 1] = 0;         leg_r_f_angle[4 + 1] = 0;
                                    time_duration[4 + 1] = 0.1 / FASTER;      time_space[4 + 1] = .0;

                                    mobile_mode = FORWARD2;
                                    pres_mobile_mode = mobile_mode;
                                    break;

     case RRIGHT_FORWARD2:         motion_all_num = 6;
                                  //  shoulder_l_r_angle[4] = 700;  shoulder_r_r_angle[4] = 0;    shoulder_l_f_angle[4] = -700; shoulder_r_f_angle[4] = 0;
                                  //  leg_l_r_angle[4] = 0;         leg_r_r_angle[4] = 0;         leg_l_f_angle[4] = 0;         leg_r_f_angle[4] = 0;
                                  //  time_duration[4] = 0.1 / FASTER;       time_space[4] = .0;
                                   shoulder_l_r_angle[0] = 700; shoulder_r_r_angle[0] = 0;     shoulder_l_f_angle[0] = -700; shoulder_r_f_angle[0] = 0;
                                   leg_l_r_angle[0] = 0;        leg_r_r_angle[0] = 0;          leg_l_f_angle[0] = -256;      leg_r_f_angle[0] = 0;
                                   time_duration[0] = 0.1 / FASTER;      time_space[0] = .0;
                                   shoulder_l_r_angle[1] = 700; shoulder_r_r_angle[1] = 0;     shoulder_l_f_angle[1] = 700;    shoulder_r_f_angle[1] = 0;
                                   leg_l_r_angle[1] = 0;        leg_r_r_angle[1] = 0;          leg_l_f_angle[1] = -512;      leg_r_f_angle[1] = 0;
                                   time_duration[1] = 0.2 / FASTER;      time_space[1] = .0;
                                   shoulder_l_r_angle[2] = 700; shoulder_r_r_angle[2] = 0;     shoulder_l_f_angle[2] = 700;    shoulder_r_f_angle[2] = 0;
                                   leg_l_r_angle[2] = 0;        leg_r_r_angle[2] = 0;          leg_l_f_angle[2] = 0;      leg_r_f_angle[2] = 0;
                                   time_duration[2] = 0.1 / FASTER;      time_space[2] = .0;
                                   shoulder_l_r_angle[2 + 1] = 0;   shoulder_r_r_angle[2 + 1] = -700;     shoulder_l_f_angle[2 + 1] = 0;    shoulder_r_f_angle[2 + 1] = 0;
                                   leg_l_r_angle[2 + 1] = 0;        leg_r_r_angle[2 + 1] = 0;       leg_l_f_angle[2 + 1] = 0;         leg_r_f_angle[2 + 1] = 256;
                                   time_duration[2 + 1] = 0.4 / FASTER;      time_space[2 + 1] = .0;
                                   shoulder_l_r_angle[3 + 1] = 0;   shoulder_r_r_angle[3 + 1] = -700;  shoulder_l_f_angle[3 + 1] = 0;    shoulder_r_f_angle[3 + 1] = 700;
                                   leg_l_r_angle[3 + 1] = 0;        leg_r_r_angle[3 + 1] = 0;       leg_l_f_angle[3 + 1] = 0;         leg_r_f_angle[3 + 1] = 512;
                                   time_duration[3 + 1] = 0.2 / FASTER;      time_space[3 + 1] = .0;
                                   shoulder_l_r_angle[4 + 1] = 0;   shoulder_r_r_angle[4 + 1] = -700;  shoulder_l_f_angle[4 + 1] = 0;    shoulder_r_f_angle[4 + 1] = 700;
                                   leg_l_r_angle[4 + 1] = 0;        leg_r_r_angle[4 + 1] = 0;          leg_l_f_angle[4 + 1] = 0;         leg_r_f_angle[4 + 1] = 0;
                                   time_duration[4 + 1] = 0.1 / FASTER;      time_space[4 + 1] = .0;

                                   mobile_mode = FORWARD3;
                                   pres_mobile_mode = mobile_mode;
                                   break;
     case RRIGHT_FORWARD3:         motion_all_num = 9;
                                  //  shoulder_l_r_angle[4] = 0;   shoulder_r_r_angle[4] = -700;  shoulder_l_f_angle[4] = 0;    shoulder_r_f_angle[4] = 700;
                                  //  leg_l_r_angle[4] = 0;        leg_r_r_angle[4] = 0;          leg_l_f_angle[4] = 0;         leg_r_f_angle[4] = 0;
                                  //  time_duration[4] = 0.1 / FASTER;      time_space[4] = .0;
                                   shoulder_l_r_angle[0] = 0;    shoulder_r_r_angle[0] = -700; shoulder_l_f_angle[0] = 0;    shoulder_r_f_angle[0] = 700;
                                   leg_l_r_angle[0] = 0;         leg_r_r_angle[0] = 0;         leg_l_f_angle[0] = 0;         leg_r_f_angle[0] = 256;
                                   time_duration[0] = 0.1 / FASTER;       time_space[0] = .0;
                                   shoulder_l_r_angle[1] = 0;    shoulder_r_r_angle[1] = -700; shoulder_l_f_angle[1] = 0;    shoulder_r_f_angle[1] = 0;
                                   leg_l_r_angle[1] = 0;         leg_r_r_angle[1] = 0;         leg_l_f_angle[1] = 0;         leg_r_f_angle[1] = 512;
                                   time_duration[1] = 0.2 / FASTER;       time_space[1] = .0;
                                   shoulder_l_r_angle[2] = -500;    shoulder_r_r_angle[2] = -700;    shoulder_l_f_angle[2] = -700; shoulder_r_f_angle[2] = 0;
                                   leg_l_r_angle[2] = 0;       leg_r_r_angle[2] = 0;         leg_l_f_angle[2] = 0;         leg_r_f_angle[2] = 0;
                                   time_duration[2] = 0.4 / FASTER;       time_space[2] = .0;
                                   shoulder_l_r_angle[3] = -500;    shoulder_r_r_angle[3] = -700;    shoulder_l_f_angle[3] = -700; shoulder_r_f_angle[3] = 0;
                                   leg_l_r_angle[3] = 0;       leg_r_r_angle[3] = -256;         leg_l_f_angle[3] = 0;         leg_r_f_angle[3] = 0;
                                   time_duration[3] = 0.1 / FASTER;       time_space[3] = .0;
                                   shoulder_l_r_angle[4] = -500;  shoulder_r_r_angle[4] = 0;    shoulder_l_f_angle[4] = -700; shoulder_r_f_angle[4] = 0;
                                   leg_l_r_angle[4] = 0;       leg_r_r_angle[4] = -512;         leg_l_f_angle[4] = 0;         leg_r_f_angle[4] = 0;
                                   time_duration[4] = 0.2 / FASTER;       time_space[4] = .0;
                                   shoulder_l_r_angle[5] = -500;  shoulder_r_r_angle[5] = 0;    shoulder_l_f_angle[5] = -700; shoulder_r_f_angle[5] = 0;
                                   leg_l_r_angle[5] = 0;       leg_r_r_angle[5] = 0;         leg_l_f_angle[5] = 0;         leg_r_f_angle[5] = 0;
                                   time_duration[5] = 0.1 / FASTER;       time_space[5] = .0;
                                   shoulder_l_r_angle[6] = -500;  shoulder_r_r_angle[6] = 0;    shoulder_l_f_angle[6] = -700; shoulder_r_f_angle[6] = 0;
                                   leg_l_r_angle[6] = 256;         leg_r_r_angle[6] = 0;         leg_l_f_angle[6] = 0;         leg_r_f_angle[6] = 0;
                                   time_duration[6] = 0.1 / FASTER;       time_space[6] = .0;
                                   shoulder_l_r_angle[7] = 700;  shoulder_r_r_angle[7] = 0;    shoulder_l_f_angle[7] = -700; shoulder_r_f_angle[7] = 0;
                                   leg_l_r_angle[7] = 512;         leg_r_r_angle[7] = 0;         leg_l_f_angle[7] = 0;         leg_r_f_angle[7] = 0;
                                   time_duration[7] = 0.2 / FASTER;       time_space[7] = .0;
                                   shoulder_l_r_angle[8] = 700;  shoulder_r_r_angle[8] = 0;    shoulder_l_f_angle[8] = -700; shoulder_r_f_angle[8] = 0;
                                   leg_l_r_angle[8] = 0;         leg_r_r_angle[8] = 0;         leg_l_f_angle[8] = 0;         leg_r_f_angle[8] = 0;
                                   time_duration[8] = 0.1 / FASTER;       time_space[8] = .0;

                                   mobile_mode = FORWARD2;
                                   pres_mobile_mode = mobile_mode;
                                   break;

       case NONE:                   motion_all_num = 1;
                                    // shoulder_l_r_angle[4] = 700; shoulder_r_r_angle[4] = 0;   shoulder_l_f_angle[4] = -700; shoulder_r_f_angle[4] = 0;
                                    // leg_l_r_angle[4] = 0;      leg_r_r_angle[4] = 0;        leg_l_f_angle[4] = 0;           leg_r_f_angle[4] = 0;
                                    // time_duration[4] = 0.4 / FASTER;      time_space[4] = .2;
                                    shoulder_l_r_angle[0] = 0;   shoulder_r_r_angle[0] = 0;     shoulder_l_f_angle[0] = 0;    shoulder_r_f_angle[0] = 0;
                                    leg_l_r_angle[0] = 0;        leg_r_r_angle[0] = 0;          leg_l_f_angle[0] = 0;         leg_r_f_angle[0] = 0;
                                    time_duration[0] = 2.0;      time_space[0] = .0;
                                    break;

       case STANDING_POSITION2:     motion_all_num = 6;
                                    // shoulder_l_r_angle[4] = 700; shoulder_r_r_angle[4] = 0;   shoulder_l_f_angle[4] = -700; shoulder_r_f_angle[4] = 0;
                                    // leg_l_r_angle[4] = 0;      leg_r_r_angle[4] = 0;        leg_l_f_angle[4] = 0;           leg_r_f_angle[4] = 0;
                                    // time_duration[4] = 0.4 / FASTER;      time_space[4] = .2;
                                    shoulder_l_r_angle[0] = 700; shoulder_r_r_angle[0] = 0;     shoulder_l_f_angle[0] = -700; shoulder_r_f_angle[0] = 0;
                                    leg_l_r_angle[0] = 0;        leg_r_r_angle[0] = 0;          leg_l_f_angle[0] = -512;      leg_r_f_angle[0] = 0;
                                    time_duration[0] = 0.1 / FASTER;      time_space[0] = .0;
                                    shoulder_l_r_angle[1] = 700; shoulder_r_r_angle[1] = 0;     shoulder_l_f_angle[1] = 0;    shoulder_r_f_angle[1] = 0;
                                    leg_l_r_angle[1] = 0;        leg_r_r_angle[1] = 0;          leg_l_f_angle[1] = -512;      leg_r_f_angle[1] = 0;
                                    time_duration[1] = 0.2 / FASTER;      time_space[1] = .0;
                                    shoulder_l_r_angle[2] = 700; shoulder_r_r_angle[2] = 0;     shoulder_l_f_angle[2] = 0;    shoulder_r_f_angle[2] = 0;
                                    leg_l_r_angle[2] = 0;        leg_r_r_angle[2] = 0;          leg_l_f_angle[2] = 0;         leg_r_f_angle[2] = 0;
                                    time_duration[2] = 0.1 / FASTER;      time_space[2] = .0;
                                    shoulder_l_r_angle[3] = 700; shoulder_r_r_angle[3] = 0;     shoulder_l_f_angle[3] = 0;    shoulder_r_f_angle[3] = 0;
                                    leg_l_r_angle[3] = 512;      leg_r_r_angle[3] = 0;          leg_l_f_angle[3] = 0;         leg_r_f_angle[3] = 0;
                                    time_duration[3] = 0.2 / FASTER;      time_space[3] = .0;
                                    shoulder_l_r_angle[4] = 0;   shoulder_r_r_angle[4] = 0;     shoulder_l_f_angle[4] = 0;    shoulder_r_f_angle[4] = 0;
                                    leg_l_r_angle[4] = 512;      leg_r_r_angle[4] = 0;          leg_l_f_angle[4] = 0;         leg_r_f_angle[4] = 0;
                                    time_duration[4] = 0.1 / FASTER;      time_space[4] = .0;
                                    shoulder_l_r_angle[5] = 0;   shoulder_r_r_angle[5] = 0;     shoulder_l_f_angle[5] = 0;    shoulder_r_f_angle[5] = 0;
                                    leg_l_r_angle[5] = 0;        leg_r_r_angle[5] = 0;          leg_l_f_angle[5] = 0;         leg_r_f_angle[5] = 0;
                                    time_duration[5] = 0.1 / FASTER;      time_space[5] = .0;
                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;
                                    break;

       case STANDING_POSITION3:     motion_all_num = 6;
                                    // shoulder_l_r_angle[4] = 700; shoulder_r_r_angle[4] = 0;   shoulder_l_f_angle[4] = -700; shoulder_r_f_angle[4] = 0;
                                    // leg_l_r_angle[4] = 0;      leg_r_r_angle[4] = 0;        leg_l_f_angle[4] = 0;           leg_r_f_angle[4] = 0;
                                    // time_duration[4] = 0.4 / FASTER;      time_space[4] = .2;
                                    shoulder_l_r_angle[0] = 0;   shoulder_r_r_angle[0] = -700;  shoulder_l_f_angle[0] = 0;    shoulder_r_f_angle[0] = 700;
                                    leg_l_r_angle[0] = 0;        leg_r_r_angle[0] = 0;          leg_l_f_angle[0] = 0;         leg_r_f_angle[0] = 512;
                                    time_duration[0] = 0.1 / FASTER;      time_space[0] = .0;
                                    shoulder_l_r_angle[1] = 0;   shoulder_r_r_angle[1] = -700;  shoulder_l_f_angle[1] = 0;    shoulder_r_f_angle[1] = 0;
                                    leg_l_r_angle[1] = 0;        leg_r_r_angle[1] = 0;          leg_l_f_angle[1] = 0;         leg_r_f_angle[1] = 512;
                                    time_duration[1] = 0.2 / FASTER;      time_space[1] = .0;
                                    shoulder_l_r_angle[2] = 0;   shoulder_r_r_angle[2] = -700;  shoulder_l_f_angle[2] = 0;    shoulder_r_f_angle[2] = 0;
                                    leg_l_r_angle[2] = 0;        leg_r_r_angle[2] = 0;          leg_l_f_angle[2] = 0;         leg_r_f_angle[2] = 0;
                                    time_duration[2] = 0.1 / FASTER;      time_space[2] = .0;
                                    shoulder_l_r_angle[3] = 0;   shoulder_r_r_angle[3] = -700;  shoulder_l_f_angle[3] = 0;    shoulder_r_f_angle[3] = 0;
                                    leg_l_r_angle[3] = 0;        leg_r_r_angle[3] = -512;       leg_l_f_angle[3] = 0;         leg_r_f_angle[3] = 0;
                                    time_duration[3] = 0.1 / FASTER;      time_space[3] = .0;
                                    shoulder_l_r_angle[4] = 0;   shoulder_r_r_angle[4] = 0;     shoulder_l_f_angle[4] = 0;    shoulder_r_f_angle[4] = 0;
                                    leg_l_r_angle[4] = 0;        leg_r_r_angle[4] = -512;       leg_l_f_angle[4] = 0;         leg_r_f_angle[4] = 0;
                                    time_duration[4] = 0.1 / FASTER;      time_space[4] = .0;
                                    shoulder_l_r_angle[5] = 0;   shoulder_r_r_angle[5] = 0;     shoulder_l_f_angle[5] = 0;    shoulder_r_f_angle[5] = 0;
                                    leg_l_r_angle[5] = 0;        leg_r_r_angle[5] = 0;          leg_l_f_angle[5] = 0;         leg_r_f_angle[5] = 0;
                                    time_duration[5] = 0.1 / FASTER;      time_space[5] = .0;
                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;
                                    // Serial.print("after STANDING_POSITION3  ");
                                    // Serial.println(pres_mobile_mode);
                                    break;


       case GRUMBLE1:               motion_all_num = 6;
                                    shoulder_l_r_angle[0] = 0;  shoulder_r_r_angle[0] = 0;  shoulder_l_f_angle[0] = 0;  shoulder_r_f_angle[0] = 0;
                                    leg_l_r_angle[0] = 256;       leg_r_r_angle[0] = -256;      leg_l_f_angle[0] = -256;      leg_r_f_angle[0] = 256;
                                    time_duration[0] = 0.1 / FASTER;       time_space[0] = .0;
                                    shoulder_l_r_angle[1] = 0;  shoulder_r_r_angle[1] = 0;  shoulder_l_f_angle[1] = 0;  shoulder_r_f_angle[1] = 0;
                                    leg_l_r_angle[1] = 0;       leg_r_r_angle[1] = 0;      leg_l_f_angle[1] = 0;      leg_r_f_angle[1] = 0;
                                    time_duration[1] = 0.1 / FASTER;       time_space[1] = .0;
                                    shoulder_l_r_angle[0 + 2] = 0;  shoulder_r_r_angle[0 + 2] = 0;  shoulder_l_f_angle[0 + 2] = 0;  shoulder_r_f_angle[0 + 2] = 0;
                                    leg_l_r_angle[0 + 2] = 256;       leg_r_r_angle[0 + 2] = -256;      leg_l_f_angle[0 + 2] = -256;      leg_r_f_angle[0 + 2] = 256;
                                    time_duration[0 + 2] = 0.1 / FASTER;       time_space[0 + 2] = .0;
                                    shoulder_l_r_angle[1 + 2] = 0;  shoulder_r_r_angle[1 + 2] = 0;  shoulder_l_f_angle[1 + 2] = 0;  shoulder_r_f_angle[1 + 2] = 0;
                                    leg_l_r_angle[1 + 2] = 0;       leg_r_r_angle[1 + 2] = 0;      leg_l_f_angle[1 + 2] = 0;      leg_r_f_angle[1 + 2] = 0;
                                    time_duration[1 + 2] = 0.1 / FASTER;       time_space[1 + 2] = .0;
                                    shoulder_l_r_angle[0 + 4] = 0;  shoulder_r_r_angle[0 + 4] = 0;  shoulder_l_f_angle[0 + 4] = 0;  shoulder_r_f_angle[0 + 4] = 0;
                                    leg_l_r_angle[0 + 4] = 256;       leg_r_r_angle[0 + 4] = -256;      leg_l_f_angle[0 + 4] = -256;      leg_r_f_angle[0 + 4] = 256;
                                    time_duration[0 + 4] = 0.1 / FASTER;       time_space[0 + 4] = .0;
                                    shoulder_l_r_angle[1 + 4] = 0;  shoulder_r_r_angle[1 + 4] = 0;  shoulder_l_f_angle[1 + 4] = 0;  shoulder_r_f_angle[1 + 4] = 0;
                                    leg_l_r_angle[1 + 4] = 0;       leg_r_r_angle[1 + 4] = 0;      leg_l_f_angle[1 + 4] = 0;      leg_r_f_angle[1 + 4] = 0;
                                    time_duration[1 + 4] = 0.1 / FASTER;       time_space[1 + 4] = .0;
                                    // shoulder_l_r_angle[2] = 500;  shoulder_r_r_angle[2] = 500;  shoulder_l_f_angle[2] = 500;  shoulder_r_f_angle[2] = 500;
                                    // leg_l_r_angle[2] = 256;       leg_r_r_angle[2] = -256;      leg_l_f_angle[2] = -256;      leg_r_f_angle[2] = 256;
                                    // time_duration[2] = 0.1 / FASTER;       time_space[2] = .0;
                                    // shoulder_l_r_angle[3] = 500;  shoulder_r_r_angle[3] = 500;  shoulder_l_f_angle[3] = 500;  shoulder_r_f_angle[3] = 500;
                                    // leg_l_r_angle[3] = 256;       leg_r_r_angle[3] = -256;      leg_l_f_angle[3] = -256;      leg_r_f_angle[3] = 256;
                                    // time_duration[3] = 0.1 / FASTER;       time_space[3] = .0;
                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;
                                    break;

       case GRUMBLE2:               motion_all_num = 6;
                                    shoulder_l_r_angle[0] = 256;  shoulder_r_r_angle[0] = -256;  shoulder_l_f_angle[0] = -256;  shoulder_r_f_angle[0] = 256;
                                    leg_l_r_angle[0] = 0;       leg_r_r_angle[0] = 0;      leg_l_f_angle[0] = 0;      leg_r_f_angle[0] = 0;
                                    time_duration[0] = 0.1 / FASTER;       time_space[0] = .0;
                                    shoulder_l_r_angle[1] = 0;  shoulder_r_r_angle[1] = 0;  shoulder_l_f_angle[1] = 0;  shoulder_r_f_angle[1] = 0;
                                    leg_l_r_angle[1] = 0;       leg_r_r_angle[1] = 0;      leg_l_f_angle[1] = 0;      leg_r_f_angle[1] = 0;
                                    time_duration[1] = 0.1 / FASTER;       time_space[1] = .0;
                                    shoulder_l_r_angle[0 + 2] = 256;  shoulder_r_r_angle[0 + 2] = -256;  shoulder_l_f_angle[0 + 2] = -256;  shoulder_r_f_angle[0 + 2] = 256;
                                    leg_l_r_angle[0 + 2] = 0;       leg_r_r_angle[0 + 2] = 0;      leg_l_f_angle[0 + 2] = 0;      leg_r_f_angle[0 + 2] = 0;
                                    time_duration[0 + 2] = 0.1 / FASTER;       time_space[0 + 2] = .0;
                                    shoulder_l_r_angle[1 + 2] = 0;  shoulder_r_r_angle[1 + 2] = 0;  shoulder_l_f_angle[1 + 2] = 0;  shoulder_r_f_angle[1 + 2] = 0;
                                    leg_l_r_angle[1 + 2] = 0;       leg_r_r_angle[1 + 2] = 0;      leg_l_f_angle[1 + 2] = 0;      leg_r_f_angle[1 + 2] = 0;
                                    time_duration[1 + 2] = 0.1 / FASTER;       time_space[1 + 2] = .0;
                                    shoulder_l_r_angle[0 + 4] = 256;  shoulder_r_r_angle[0 + 4] = -256;  shoulder_l_f_angle[0 + 4] = -256;  shoulder_r_f_angle[0 + 4] = 256;
                                    leg_l_r_angle[0 + 4] = 0;       leg_r_r_angle[0 + 4] = 0;      leg_l_f_angle[0 + 4] = 0;      leg_r_f_angle[0 + 4] = 0;
                                    time_duration[0 + 4] = 0.1 / FASTER;       time_space[0 + 4] = .0;
                                    shoulder_l_r_angle[1 + 4] = 0;  shoulder_r_r_angle[1 + 4] = 0;  shoulder_l_f_angle[1 + 4] = 0;  shoulder_r_f_angle[1 + 4] = 0;
                                    leg_l_r_angle[1 + 4] = 0;       leg_r_r_angle[1 + 4] = 0;      leg_l_f_angle[1 + 4] = 0;      leg_r_f_angle[1 + 4] = 0;
                                    time_duration[1 + 4] = 0.1 / FASTER;       time_space[1 + 4] = .0;
                                    // shoulder_l_r_angle[2] = 500;  shoulder_r_r_angle[2] = 500;  shoulder_l_f_angle[2] = 500;  shoulder_r_f_angle[2] = 500;
                                    // leg_l_r_angle[2] = 256;       leg_r_r_angle[2] = -256;      leg_l_f_angle[2] = -256;      leg_r_f_angle[2] = 256;
                                    // time_duration[2] = 0.1 / FASTER;       time_space[2] = .0;
                                    // shoulder_l_r_angle[3] = 500;  shoulder_r_r_angle[3] = 500;  shoulder_l_f_angle[3] = 500;  shoulder_r_f_angle[3] = 500;
                                    // leg_l_r_angle[3] = 256;       leg_r_r_angle[3] = -256;      leg_l_f_angle[3] = -256;      leg_r_f_angle[3] = 256;
                                    // time_duration[3] = 0.1 / FASTER;       time_space[3] = .0;
                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;
                                    break;

       case LOOK_AROUND_LEFT:       motion_all_num = 2;
                                    shoulder_l_r_angle[0] = 700;  shoulder_r_r_angle[0] = 700;  shoulder_l_f_angle[0] = 700;  shoulder_r_f_angle[0] = 700;
                                    leg_l_r_angle[0] = 0;       leg_r_r_angle[0] = 0;      leg_l_f_angle[0] = 0;      leg_r_f_angle[0] = 0;
                                    time_duration[0] = 2.0 / FASTER;       time_space[0] = 2.0;
                                    shoulder_l_r_angle[1] = 0;  shoulder_r_r_angle[1] = 0;  shoulder_l_f_angle[1] = 0;  shoulder_r_f_angle[1] = 0;
                                    leg_l_r_angle[1] = 0;       leg_r_r_angle[1] = 0;      leg_l_f_angle[1] = 0;      leg_r_f_angle[1] = 0;
                                    time_duration[1] = 2.0 / FASTER;       time_space[1] = .0;
                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;
                                    break;
       case LOOK_AROUND_RIGHT:       motion_all_num = 2;
                                    shoulder_l_r_angle[0] = -700;  shoulder_r_r_angle[0] = -700;  shoulder_l_f_angle[0] = -700;  shoulder_r_f_angle[0] = -700;
                                    leg_l_r_angle[0] = 0;       leg_r_r_angle[0] = 0;      leg_l_f_angle[0] = 0;      leg_r_f_angle[0] = 0;
                                    time_duration[0] = 2.0 / FASTER;       time_space[0] = 2.0;
                                    shoulder_l_r_angle[1] = 0;  shoulder_r_r_angle[1] = 0;  shoulder_l_f_angle[1] = 0;  shoulder_r_f_angle[1] = 0;
                                    leg_l_r_angle[1] = 0;       leg_r_r_angle[1] = 0;      leg_l_f_angle[1] = 0;      leg_r_f_angle[1] = 0;
                                    time_duration[1] = 2.0 / FASTER;       time_space[1] = .0;
                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;
                                    break;

       case RLEFT:                  motion_all_num = 13;
                                    shoulder_l_r_angle[0] = 500;  shoulder_r_r_angle[0] = 500;  shoulder_l_f_angle[0] = 500;  shoulder_r_f_angle[0] = 500;
                                    leg_l_r_angle[0] = 0;         leg_r_r_angle[0] = 0;         leg_l_f_angle[0] = 0;         leg_r_f_angle[0] = 0;
                                    time_duration[0] = 0.1 / FASTER;       time_space[0] = .0;

                                    shoulder_l_r_angle[1] = 500;  shoulder_r_r_angle[1] = 500;  shoulder_l_f_angle[1] = 500;  shoulder_r_f_angle[1] = 500;
                                    leg_l_r_angle[1] = 0;         leg_r_r_angle[1] = -256;      leg_l_f_angle[1] = 0;         leg_r_f_angle[1] = 0;
                                    time_duration[1] = 0.1 / FASTER;       time_space[1] = .0;
                                    shoulder_l_r_angle[2] = 500;  shoulder_r_r_angle[2] = 0;    shoulder_l_f_angle[2] = 500;  shoulder_r_f_angle[2] = 500;
                                    leg_l_r_angle[2] = 0;         leg_r_r_angle[2] = -512;      leg_l_f_angle[2] = 0;         leg_r_f_angle[2] = 0;
                                    time_duration[2] = 0.2 / FASTER;       time_space[2] = .0;
                                    shoulder_l_r_angle[3] = 500;  shoulder_r_r_angle[3] = 0;    shoulder_l_f_angle[3] = 500;  shoulder_r_f_angle[3] = 500;
                                    leg_l_r_angle[3] = 0;         leg_r_r_angle[3] = 0;         leg_l_f_angle[3] = 0;         leg_r_f_angle[3] = 0;
                                    time_duration[3] = 0.1 / FASTER;       time_space[3] = .0;

                                    shoulder_l_r_angle[4] = 500;  shoulder_r_r_angle[4] = 0;    shoulder_l_f_angle[4] = 500;  shoulder_r_f_angle[4] = 500;
                                    leg_l_r_angle[4] = 0;         leg_r_r_angle[4] = 0;         leg_l_f_angle[4] = 0;         leg_r_f_angle[4] = 256;
                                    time_duration[4] = 0.1 / FASTER;       time_space[4] = .0;
                                    shoulder_l_r_angle[5] = 500;  shoulder_r_r_angle[5] = 0;    shoulder_l_f_angle[5] = 500;  shoulder_r_f_angle[5] = 0;
                                    leg_l_r_angle[5] = 0;         leg_r_r_angle[5] = 0;         leg_l_f_angle[5] = 0;         leg_r_f_angle[5] = 512;
                                    time_duration[5] = 0.2 / FASTER;       time_space[5] = .0;
                                    shoulder_l_r_angle[6] = 500;  shoulder_r_r_angle[6] = 0;    shoulder_l_f_angle[6] = 500;  shoulder_r_f_angle[6] = 0;
                                    leg_l_r_angle[6] = 0;         leg_r_r_angle[6] = 0;         leg_l_f_angle[6] = 0;         leg_r_f_angle[6] = 0;
                                    time_duration[6] = 0.1 / FASTER;       time_space[6] = .0;

                                    shoulder_l_r_angle[7] = 500;    shoulder_r_r_angle[7] = 0;    shoulder_l_f_angle[7] = 500;    shoulder_r_f_angle[7] = 0;
                                    leg_l_r_angle[7] = 0;       leg_r_r_angle[7] = 0;         leg_l_f_angle[7] = -256;         leg_r_f_angle[7] = 0;
                                    time_duration[7] = 0.1 / FASTER;       time_space[7] = .0;
                                    shoulder_l_r_angle[8] = 500;  shoulder_r_r_angle[8] = 0;    shoulder_l_f_angle[8] = 0;    shoulder_r_f_angle[8] = 0;
                                    leg_l_r_angle[8] = 0;       leg_r_r_angle[8] = 0;         leg_l_f_angle[8] = -512;         leg_r_f_angle[8] = 0;
                                    time_duration[8] = 0.2 / FASTER;       time_space[8] = .0;
                                    shoulder_l_r_angle[9] = 500;  shoulder_r_r_angle[9] = 0;    shoulder_l_f_angle[9] = 0;    shoulder_r_f_angle[9] = 0;
                                    leg_l_r_angle[9] = 0;         leg_r_r_angle[9] = 0;         leg_l_f_angle[9] = 0;         leg_r_f_angle[9] = 0;
                                    time_duration[9] = 0.1 / FASTER;       time_space[9] = .0;

                                    shoulder_l_r_angle[10] = 500;    shoulder_r_r_angle[10] = 0;    shoulder_l_f_angle[10] = 0;    shoulder_r_f_angle[10] = 0;
                                    leg_l_r_angle[10] = 256;       leg_r_r_angle[10] = 0;         leg_l_f_angle[10] = 0;         leg_r_f_angle[10] = 0;
                                    time_duration[10] = 0.1 / FASTER;       time_space[10] = .0;
                                    shoulder_l_r_angle[11] = 0;  shoulder_r_r_angle[11] = 0;    shoulder_l_f_angle[11] = 0;    shoulder_r_f_angle[11] = 0;
                                    leg_l_r_angle[11] = 512;       leg_r_r_angle[11] = 0;         leg_l_f_angle[11] = 0;         leg_r_f_angle[11] = 0;
                                    time_duration[11] = 0.2 / FASTER;       time_space[11] = .0;
                                    shoulder_l_r_angle[12] = 0;  shoulder_r_r_angle[12] = 0;    shoulder_l_f_angle[12] = 0;    shoulder_r_f_angle[12] = 0;
                                    leg_l_r_angle[12] = 0;         leg_r_r_angle[12] = 0;         leg_l_f_angle[12] = 0;         leg_r_f_angle[12] = 0;
                                    time_duration[12] = 0.1 / FASTER;       time_space[12] = .0;

                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;

                                    break;
       case RRIGHT:                 motion_all_num = 13;
                                    shoulder_l_r_angle[0] = -500;    shoulder_r_r_angle[0] = -500;    shoulder_l_f_angle[0] = -500;    shoulder_r_f_angle[0] = -500;
                                    leg_l_r_angle[0] = 0;         leg_r_r_angle[0] = 0;         leg_l_f_angle[0] = 0;         leg_r_f_angle[0] = 0;
                                    time_duration[0] = 0.1 / FASTER;       time_space[0] = .0;

                                    shoulder_l_r_angle[1] = -500;    shoulder_r_r_angle[1] = -500;    shoulder_l_f_angle[1] = -500;    shoulder_r_f_angle[1] = -500;
                                    leg_l_r_angle[1] = 256;       leg_r_r_angle[1] = 0;         leg_l_f_angle[1] = 0;         leg_r_f_angle[1] = 0;
                                    time_duration[1] = 0.1 / FASTER;       time_space[1] = .0;
                                    shoulder_l_r_angle[2] = 0;  shoulder_r_r_angle[2] = -500;    shoulder_l_f_angle[2] = -500;    shoulder_r_f_angle[2] = -500;
                                    leg_l_r_angle[2] = 512;       leg_r_r_angle[2] = 0;         leg_l_f_angle[2] = 0;         leg_r_f_angle[2] = 0;
                                    time_duration[2] = 0.2 / FASTER;       time_space[2] = .0;
                                    shoulder_l_r_angle[3] = 0;  shoulder_r_r_angle[3] = -500;    shoulder_l_f_angle[3] = -500;    shoulder_r_f_angle[3] = -500;
                                    leg_l_r_angle[3] = 0;         leg_r_r_angle[3] = 0;         leg_l_f_angle[3] = 0;         leg_r_f_angle[3] = 0;
                                    time_duration[3] = 0.1 / FASTER;       time_space[3] = .0;

                                    shoulder_l_r_angle[4] = 0;    shoulder_r_r_angle[4] = -500;    shoulder_l_f_angle[4] = -500;    shoulder_r_f_angle[4] = -500;
                                    leg_l_r_angle[4] = 0;       leg_r_r_angle[4] = 0;         leg_l_f_angle[4] = 0;         leg_r_f_angle[4] = 0;
                                    time_duration[4] = 0.1 / FASTER;       time_space[4] = .0;
                                    shoulder_l_r_angle[5] = 0;  shoulder_r_r_angle[5] = -500;    shoulder_l_f_angle[5] = 0;    shoulder_r_f_angle[5] = -500;
                                    leg_l_r_angle[5] = 0;       leg_r_r_angle[5] = 0;         leg_l_f_angle[5] = -256;         leg_r_f_angle[5] = 0;
                                    time_duration[5] = 0.2 / FASTER;       time_space[5] = .0;
                                    shoulder_l_r_angle[6] = 0;  shoulder_r_r_angle[6] = -500;    shoulder_l_f_angle[6] = 0;    shoulder_r_f_angle[6] = -500;
                                    leg_l_r_angle[6] = 0;         leg_r_r_angle[6] = 0;         leg_l_f_angle[6] = -512;         leg_r_f_angle[6] = 0;
                                    time_duration[6] = 0.1 / FASTER;       time_space[6] = .0;

                                    shoulder_l_r_angle[7] = 0;    shoulder_r_r_angle[7] = -500;    shoulder_l_f_angle[7] = 0;    shoulder_r_f_angle[7] = -500;
                                    leg_l_r_angle[7] = 0;       leg_r_r_angle[7] = 0;         leg_l_f_angle[7] = 0;         leg_r_f_angle[7] = 0;
                                    time_duration[7] = 0.1 / FASTER;       time_space[7] = .0;
                                    shoulder_l_r_angle[8] = 0;  shoulder_r_r_angle[8] = -500;    shoulder_l_f_angle[8] = 0;    shoulder_r_f_angle[8] = 0;
                                    leg_l_r_angle[8] = 0;       leg_r_r_angle[8] = 0;         leg_l_f_angle[8] = 0;         leg_r_f_angle[8] = 256;
                                    time_duration[8] = 0.2 / FASTER;       time_space[8] = .0;
                                    shoulder_l_r_angle[9] = 0;  shoulder_r_r_angle[9] = -500;    shoulder_l_f_angle[9] = 0;    shoulder_r_f_angle[9] = 0;
                                    leg_l_r_angle[9] = 0;         leg_r_r_angle[9] = 0;         leg_l_f_angle[9] = 0;         leg_r_f_angle[9] = 512;
                                    time_duration[9] = 0.1 / FASTER;       time_space[9] = .0;

                                    shoulder_l_r_angle[10] = 0;    shoulder_r_r_angle[10] = -500;    shoulder_l_f_angle[10] = 0;    shoulder_r_f_angle[10] = 0;
                                    leg_l_r_angle[10] = 0;       leg_r_r_angle[10] = -256;         leg_l_f_angle[10] = 0;         leg_r_f_angle[10] = 0;
                                    time_duration[10] = 0.1 / FASTER;       time_space[10] = .0;
                                    shoulder_l_r_angle[11] = 0;  shoulder_r_r_angle[11] = 0;    shoulder_l_f_angle[11] = 0;    shoulder_r_f_angle[11] = 0;
                                    leg_l_r_angle[11] = 0;       leg_r_r_angle[11] = -512;         leg_l_f_angle[11] = 0;         leg_r_f_angle[11] = 0;
                                    time_duration[11] = 0.2 / FASTER;       time_space[11] = .0;
                                    shoulder_l_r_angle[12] = 0;  shoulder_r_r_angle[12] = 0;    shoulder_l_f_angle[12] = 0;    shoulder_r_f_angle[12] = 0;
                                    leg_l_r_angle[12] = 0;         leg_r_r_angle[12] = 0;         leg_l_f_angle[12] = 0;         leg_r_f_angle[12] = 0;
                                    time_duration[12] = 0.1 / FASTER;       time_space[12] = .0;

                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;

                                    break;
      //  case RRIGHT:                 leg_l_r_angle = -velocity; leg_r_r_angle = -velocity; leg_l_f_angle = -velocity; leg_r_f_angle = -velocity;
      //                               shoulder_l_r_angle = 0; shoulder_r_r_angle = 0; shoulder_l_f_angle = 0; shoulder_r_f_angle = 0;
      //                               break;
      //  case PLEFT:                  leg_l_r_angle = -velocity * SPEED_ADD_ON; leg_r_r_angle = -velocity * SPEED_ADD_ON; leg_l_f_angle = velocity * SPEED_ADD_ON; leg_r_f_angle = velocity * SPEED_ADD_ON;
      //                               shoulder_l_r_angle = -512; shoulder_r_r_angle = 512; shoulder_l_f_angle = 512; shoulder_r_f_angle = -512;
      //                               break;
      //  case PRIGHT:                 leg_l_r_angle = velocity * SPEED_ADD_ON; leg_r_r_angle = velocity * SPEED_ADD_ON; leg_l_f_angle = -velocity * SPEED_ADD_ON; leg_r_f_angle = -velocity * SPEED_ADD_ON;
      //                               shoulder_l_r_angle = -512; shoulder_r_r_angle = 512; shoulder_l_f_angle = 512; shoulder_r_f_angle = -512;
      //                               break;
       //
      //  case PLEFT_FORWARD:          leg_l_r_angle = -velocity * SPEED_ADD_ON; leg_r_r_angle = -velocity * SPEED_ADD_ON; leg_l_f_angle = velocity * SPEED_ADD_ON; leg_r_f_angle = velocity * SPEED_ADD_ON;
      //                               shoulder_l_r_angle = 0; shoulder_r_r_angle = 1024; shoulder_l_f_angle = 1024; shoulder_r_f_angle = 0;
      //                               break;
      //  case PRIGHT_FORWARD:         leg_l_r_angle = velocity * SPEED_ADD_ON; leg_r_r_angle = velocity * SPEED_ADD_ON; leg_l_f_angle = -velocity * SPEED_ADD_ON; leg_r_f_angle = -velocity * SPEED_ADD_ON;
      //                               shoulder_l_r_angle = -1024; shoulder_r_r_angle = 0; shoulder_l_f_angle = 0; shoulder_r_f_angle = -1024;
      //                               break;
      //  case PLEFT_BACKWARD:         leg_l_r_angle = -velocity * SPEED_ADD_ON; leg_r_r_angle = -velocity * SPEED_ADD_ON; leg_l_f_angle = velocity * SPEED_ADD_ON; leg_r_f_angle = velocity * SPEED_ADD_ON;
      //                               shoulder_l_r_angle = -1024; shoulder_r_r_angle = 0; shoulder_l_f_angle = 0; shoulder_r_f_angle = -1024;
      //                               break;
      //  case PRIGHT_BACKWARD:        leg_l_r_angle = velocity * SPEED_ADD_ON; leg_r_r_angle = velocity * SPEED_ADD_ON; leg_l_f_angle = -velocity * SPEED_ADD_ON; leg_r_f_angle = -velocity * SPEED_ADD_ON;
      //                               shoulder_l_r_angle = 0; shoulder_r_r_angle = 1024; shoulder_l_f_angle = 1024; shoulder_r_f_angle = 0;
      //                               break;
      //  case RLEFT_FORWARD:          leg_l_r_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               leg_r_r_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               leg_l_f_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               leg_r_f_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               shoulder_l_r_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_r_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_l_f_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_f_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               break;
      //  case RRIGHT_FORWARD:         leg_l_r_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               leg_r_r_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               leg_l_f_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               leg_r_f_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               shoulder_l_r_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_r_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_l_f_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_f_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               break;
      //  case RLEFT_BACKWARD:         leg_l_r_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               leg_r_r_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               leg_l_f_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               leg_r_f_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               shoulder_l_r_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_r_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_l_f_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_f_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               break;
      //  case RRIGHT_BACKWARD:        leg_l_r_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               leg_r_r_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               leg_l_f_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               leg_r_f_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               shoulder_l_r_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_r_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_l_f_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_f_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               break;
      //  case RLEFT_PLEFT:            leg_l_r_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               leg_r_r_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               leg_l_f_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               leg_r_f_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               shoulder_l_r_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_r_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_l_f_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_f_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               break;
      //  case RLEFT_PRIGHT:           leg_l_r_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               leg_r_r_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               leg_l_f_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               leg_r_f_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               shoulder_l_r_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_r_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_l_f_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_f_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               break;
      //  case RRIGHT_PLEFT:           leg_l_r_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               leg_r_r_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               leg_l_f_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               leg_r_f_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               shoulder_l_r_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_r_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_l_f_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_f_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               break;
      //  case RRIGHT_PRIGHT:          leg_l_r_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               leg_r_r_angle = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
      //                               leg_l_f_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               leg_r_f_angle = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
      //                               shoulder_l_r_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_r_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_l_f_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               shoulder_r_f_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
      //                               break;

      default:              break;
      // default:              motion_all_num = 1;
      //                       shoulder_l_r_angle[0] = 0; shoulder_r_r_angle[0] = 0; shoulder_l_f_angle[0] = 0; shoulder_r_f_angle[0] = 0;
      //                       leg_l_r_angle[0] = 0; leg_r_r_angle[0] = 0; leg_l_f_angle[0] = 0; leg_r_f_angle[0] = 0;
      //                       time_duration[0] = 1.0; time_space[0] = 1.0;
      //                       break;
     }


   }

   int showMode()
   {
     return (int)mobile_mode;
   }

   int* setJointAngle(int motion_num)
   {
     if (leg_l_r_angle[motion_num] < 0) leg_l_r_angle[motion_num] = 0;
     else if (leg_l_r_angle[motion_num] > 512) leg_l_r_angle[motion_num] = 512;
     if (leg_r_r_angle[motion_num] < -512) leg_r_r_angle[motion_num] = -512;
     else if (leg_r_r_angle[motion_num] > 0) leg_r_r_angle[motion_num] = 0;
     if (leg_l_f_angle[motion_num] < -512) leg_l_f_angle[motion_num] = -512;
     else if (leg_l_f_angle[motion_num] > 0) leg_l_f_angle[motion_num] = 0;
     if (leg_r_f_angle[motion_num] < 0) leg_r_f_angle[motion_num] = 0;
     else if (leg_r_f_angle[motion_num] > 512) leg_r_f_angle[motion_num] = 512;
     if (shoulder_l_r_angle[motion_num] < -1024) shoulder_l_r_angle[motion_num] = -1024;
     else if (shoulder_l_r_angle[motion_num] > 1024) shoulder_l_r_angle[motion_num] = 1024;
     if (shoulder_r_r_angle[motion_num] < -1024) shoulder_r_r_angle[motion_num] = -1024;
     else if (shoulder_r_r_angle[motion_num] > 1024) shoulder_r_r_angle[motion_num] = 1024;
     if (shoulder_l_f_angle[motion_num] < -1024) shoulder_l_f_angle[motion_num] = -1024;
     else if (shoulder_l_f_angle[motion_num] > 768) shoulder_l_f_angle[motion_num] = 768;
     if (shoulder_r_f_angle[motion_num] < -768) shoulder_r_f_angle[motion_num] = -768;
     else if (shoulder_r_f_angle[motion_num] > 1024) shoulder_r_f_angle[motion_num] = 1024;

     joint_angle[0] = leg_l_r_angle[motion_num] + 1024;
     joint_angle[1] = leg_r_r_angle[motion_num] + 3072;
     joint_angle[2] = leg_l_f_angle[motion_num] + 3072;
     joint_angle[3] = leg_r_f_angle[motion_num] + 1024;
     joint_angle[4] = shoulder_l_r_angle[motion_num] + 2048;
     joint_angle[5] = shoulder_r_r_angle[motion_num] + 2048;
     joint_angle[6] = shoulder_l_f_angle[motion_num] + 2048;
     joint_angle[7] = shoulder_r_f_angle[motion_num] + 2048;

     return joint_angle;
   }
};

RC100 Controller;
int rcData = 0;

DynamixelStatus dynamixelStatus;

int dxl_comm_result = COMM_TX_FAIL;             // Communication result
bool dxl_getdata_result = false;                // GetParam result

uint8_t dxl_error = 0;                          // Dynamixel error

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

void setup()
{
  Serial.begin(57600);

  Controller.begin(1);

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = dynamixel::PortHandler::getPortHandler("");

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  portHandler->openPort();

  // Set port baudrate
  portHandler->setBaudRate(BAUDRATE);

  // Torque Enable
  syncWrite(ADDR_TORQUE_ENABLE, 1, ON);

  packetHandler->write1ByteTxRx(portHandler, HEAD_PITCH, ADDR_TORQUE_ENABLE, ON, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, HEAD_YAW, ADDR_TORQUE_ENABLE, ON, &dxl_error);

  packetHandler->write4ByteTxRx(portHandler, HEAD_PITCH, ADDR_GOAL_POSITION, dynamixelStatus.head_pitch, &dxl_error);
  packetHandler->write4ByteTxRx(portHandler, HEAD_YAW, ADDR_GOAL_POSITION, dynamixelStatus.head_yaw, &dxl_error);


  // Init Motion
  writeMotion();
}

void loop()
{
  if (Controller.available())
  {
    rcData = Controller.readData();
    dynamixelStatus.getDirection(rcData);
    if ((rcData == RC100_BTN_2) || (rcData == RC100_BTN_4))
    {
      packetHandler->write4ByteTxRx(portHandler, HEAD_YAW, ADDR_GOAL_POSITION, dynamixelStatus.head_yaw, &dxl_error);
    }
    else if ((rcData == RC100_BTN_1) || (rcData == RC100_BTN_3))
    {
      packetHandler->write4ByteTxRx(portHandler, HEAD_PITCH, ADDR_GOAL_POSITION, dynamixelStatus.head_pitch, &dxl_error);
    }
    else
    {
      writeMotion();

      Controller.begin(1);
    }
    // Serial.print("Direction : ");
    // dynamixelStatus.getDirection(rcData);
    // Serial.println(dynamixelStatus.showMode());
  }

  // if (Controller.available())
  // {
  //   rcData = Controller.readData();
  //   Serial.print("Direction : ");
  //   dynamixelStatus.getDirection(rcData);
  //   Serial.println(dynamixelStatus.showMode());
  //
  //   writeMotion();
  //
  //   packetHandler->write4ByteTxRx(portHandler, HEAD_YAW, ADDR_GOAL_POSITION, dynamixelStatus.head_yaw, &dxl_error);
  //   packetHandler->write4ByteTxRx(portHandler, HEAD_PITCH, ADDR_GOAL_POSITION, dynamixelStatus.head_pitch, &dxl_error);
  //
  //
  //   Controller.begin(1);
  // }
}

void writeMotion()
{
  int presPos[8] = {0, };
  int* goalPos;
  double dist[8] = {0.0, };
  int goalPos_dist[8] = {0, };
  double goalPos_dist_db[8] = {0.0, };

  dynamixelStatus.setParams();

  for (int motion_num = 0; motion_num < dynamixelStatus.motion_all_num; motion_num++)
  {
    double dist_number = dynamixelStatus.time_duration[motion_num] / 0.008;

    goalPos = dynamixelStatus.setJointAngle(motion_num);
    syncRead(ADDR_PRESENT_POSITION, 4, presPos);

    //
    //
    // // Initialize Groupsyncread instance for Present Position
    // dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);
    //
    // // Add parameter storage for Dynamixel#1 present position value
    // groupSyncRead.addParam(LEG_L_R);
    // groupSyncRead.addParam(LEG_R_R);
    // groupSyncRead.addParam(LEG_L_F);
    // groupSyncRead.addParam(LEG_R_F);
    // groupSyncRead.addParam(SHOULDER_L_R);
    // groupSyncRead.addParam(SHOULDER_R_R);
    // groupSyncRead.addParam(SHOULDER_L_F);
    // groupSyncRead.addParam(SHOULDER_R_F);
    //
    // groupSyncRead.txRxPacket();
    //
    // presPos[0] = (int)groupSyncRead.getData(LEG_L_R, ADDR_PRESENT_POSITION, 4);
    // presPos[1] = (int)groupSyncRead.getData(LEG_R_R, ADDR_PRESENT_POSITION, 4);
    // presPos[2] = (int)groupSyncRead.getData(LEG_L_F, ADDR_PRESENT_POSITION, 4);
    // presPos[3] = (int)groupSyncRead.getData(LEG_R_F, ADDR_PRESENT_POSITION, 4);
    // presPos[4] = (int)groupSyncRead.getData(SHOULDER_L_R, ADDR_PRESENT_POSITION, 4);
    // presPos[5] = (int)groupSyncRead.getData(SHOULDER_R_R, ADDR_PRESENT_POSITION, 4);
    // presPos[6] = (int)groupSyncRead.getData(SHOULDER_L_F, ADDR_PRESENT_POSITION, 4);
    // presPos[7] = (int)groupSyncRead.getData(SHOULDER_R_F, ADDR_PRESENT_POSITION, 4);
    //
    // groupSyncRead.clearParam();

    for (int joint_num = 0; joint_num < 8; joint_num++)
    {
      dist[joint_num] = (goalPos[joint_num] * 1.0 - presPos[joint_num] * 1.0) / dist_number;
      goalPos_dist_db[joint_num] = presPos[joint_num] * 1.0;
    }

    for (int res_Num = 0; res_Num < (int)dist_number; res_Num++)
  	{
      for (int joint_num = 0; joint_num < 8; joint_num++)
  		{
  			// if (all_dist[joint_num] > 0)
        // {
  			// 	if (vProfile[joint_num] > 0)
        //   {
  			// 		vProfile[joint_num] -= aArray[joint_num];
  			// 		if (vProfile[joint_num] < 0) vProfile[joint_num] = 0;
  			// 	}
  			// }
  			// else {
  			// 	if (vProfile[joint_num] < 0)
        //   {
  			// 		vProfile[joint_num] -= aArray[joint_num];
  			// 		if (vProfile[joint_num] > 0) vProfile[joint_num] = 0;
  			// 	}
  			// }
  			//tArray[joint_num] += vArray[joint_num];
  			goalPos_dist_db[joint_num] += dist[joint_num];
  			goalPos_dist[joint_num] = (int)goalPos_dist_db[joint_num];
  		}

      syncWrite(ADDR_GOAL_POSITION, 4, goalPos_dist);

      delay(8);
    }

    delay(dynamixelStatus.time_space[motion_num] * 1000.0);
  }
}

void syncWrite(int address, int length, int value)
{
  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, address, length);

  // Add Dynamixels goal values to the Syncwrite storage
  groupSyncWrite.addParam(LEG_L_R, (uint8_t*)&value);
  groupSyncWrite.addParam(LEG_R_R, (uint8_t*)&value);
  groupSyncWrite.addParam(LEG_L_F, (uint8_t*)&value);
  groupSyncWrite.addParam(LEG_R_F, (uint8_t*)&value);
  groupSyncWrite.addParam(SHOULDER_L_R, (uint8_t*)&value);
  groupSyncWrite.addParam(SHOULDER_R_R, (uint8_t*)&value);
  groupSyncWrite.addParam(SHOULDER_L_F, (uint8_t*)&value);
  groupSyncWrite.addParam(SHOULDER_R_F, (uint8_t*)&value);

  // Syncwrite goal position
  groupSyncWrite.txPacket();

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();
}

void syncWrite(int address, int length, int* value)
{
  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, address, length);

  // Add Dynamixels goal values to the Syncwrite storage
  groupSyncWrite.addParam(LEG_L_R, (uint8_t*)&value[0]);
  groupSyncWrite.addParam(LEG_R_R, (uint8_t*)&value[1]);
  groupSyncWrite.addParam(LEG_L_F, (uint8_t*)&value[2]);
  groupSyncWrite.addParam(LEG_R_F, (uint8_t*)&value[3]);
  groupSyncWrite.addParam(SHOULDER_L_R, (uint8_t*)&value[4]);
  groupSyncWrite.addParam(SHOULDER_R_R, (uint8_t*)&value[5]);
  groupSyncWrite.addParam(SHOULDER_L_F, (uint8_t*)&value[6]);
  groupSyncWrite.addParam(SHOULDER_R_F, (uint8_t*)&value[7]);

  // Syncwrite goal position
  groupSyncWrite.txPacket();

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();
}

void syncRead(int address, int length, int* readValues)
{
  // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, address, length);

  // Add parameter storage for Dynamixel#1 present position value
  groupSyncRead.addParam(LEG_L_R);
  groupSyncRead.addParam(LEG_R_R);
  groupSyncRead.addParam(LEG_L_F);
  groupSyncRead.addParam(LEG_R_F);
  groupSyncRead.addParam(SHOULDER_L_R);
  groupSyncRead.addParam(SHOULDER_R_R);
  groupSyncRead.addParam(SHOULDER_L_F);
  groupSyncRead.addParam(SHOULDER_R_F);

  groupSyncRead.txRxPacket();

  readValues[0] = (int)groupSyncRead.getData(LEG_L_R, address, length);
  readValues[1] = (int)groupSyncRead.getData(LEG_R_R, address, length);
  readValues[2] = (int)groupSyncRead.getData(LEG_L_F, address, length);
  readValues[3] = (int)groupSyncRead.getData(LEG_R_F, address, length);
  readValues[4] = (int)groupSyncRead.getData(SHOULDER_L_R, address, length);
  readValues[5] = (int)groupSyncRead.getData(SHOULDER_R_R, address, length);
  readValues[6] = (int)groupSyncRead.getData(SHOULDER_L_F, address, length);
  readValues[7] = (int)groupSyncRead.getData(SHOULDER_R_F, address, length);

  groupSyncRead.clearParam();
}

void wait(double ms)
{
  unsigned long time = millis();
  while(1)
  {
    if (millis() - time > (unsigned long)ms) break;
  }
}
