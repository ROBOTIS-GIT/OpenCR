/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#ifndef TURTLEBOT3_REALTURTLEBOT_MOTION_H_
#define TURTLEBOT3_REALTURTLEBOT_MOTION_H_

#include <RC100.h>

#define SPEED_ADD_ON      2
#define MOTION_NUM        20

#define FASTER            4.0

class TurtlebotMotion
{
 private:
   int joint_angle[8] = {0, };

   enum MODE
   {
     NONE = 0
     , FORWARD1, FORWARD2, FORWARD3, FORWARD4
     , STANDING_POSE2, STANDING_POSE3
     , GRUMBLE1, GRUMBLE2, RLEFT, RRIGHT, PLEFT, PRIGHT
     , LOOK_AROUND_LEFT, LOOK_AROUND_RIGHT
     , PLEFT_FORWARD, PRIGHT_FORWARD, PLEFT_BACKWARD, PRIGHT_BACKWARD
     , RLEFT_FORWARD2, RLEFT_FORWARD3
     , RRIGHT_FORWARD2, RRIGHT_FORWARD3
     , RLEFT_BACKWARD, RRIGHT_BACKWARD, RLEFT_PLEFT, RLEFT_PRIGHT, RRIGHT_PLEFT, RRIGHT_PRIGHT
   } mobile_mode, pres_mobile_mode;

 public:
   int shoulder_left_rear_angle[MOTION_NUM] = {0, }, shoulder_right_rear_angle[MOTION_NUM] = {0, }, shoulder_left_front_angle[MOTION_NUM] = {0, }, shoulder_right_front_angle[MOTION_NUM] = {0, };
   int leg_left_rear_angle[MOTION_NUM] = {0, }, leg_right_rear_angle[MOTION_NUM] = {0, }, leg_left_front_angle[MOTION_NUM] = {0, }, leg_right_front_angle[MOTION_NUM] = {0, };
   int head_yaw = 2048, head_pitch = 3072; // 1848 ~ 2248 : 3072 ~ 2560
   int motion_all_num;
   double time_duration[MOTION_NUM] = {0.0, }, time_space[MOTION_NUM] = {0.0, };

   TurtlebotMotion()
   : motion_all_num(1)
   { }

   MODE getDirection(int rcData)
   {
     switch (rcData)
     {
      case (RC100_BTN_U): if (pres_mobile_mode == NONE)
                          {
                            mobile_mode = FORWARD1;
                          }
                          else if (pres_mobile_mode == FORWARD1)
                          {
                            mobile_mode = FORWARD2;
                          }
                          else if (pres_mobile_mode == FORWARD2)
                          {
                            mobile_mode = FORWARD3;
                          }
                          else if (pres_mobile_mode == FORWARD3)
                          {
                            mobile_mode = FORWARD2;
                          }
                          pres_mobile_mode = mobile_mode;
                          break;
      case (RC100_BTN_L): if (pres_mobile_mode == NONE)
                          {
                            mobile_mode = RLEFT;
                          }
                          else if (pres_mobile_mode == FORWARD2)
                          {
                            mobile_mode = STANDING_POSE2;
                          }
                          else if (pres_mobile_mode == FORWARD3)
                          {
                            mobile_mode = STANDING_POSE3;
                          }
                          pres_mobile_mode = mobile_mode;
                          break;
      case (RC100_BTN_R): if (pres_mobile_mode == NONE)
                          {
                            mobile_mode = RRIGHT;
                          }
                          else if (pres_mobile_mode == FORWARD2)
                          {
                            mobile_mode = STANDING_POSE2;
                          }
                          else if (pres_mobile_mode == FORWARD3)
                          {
                            mobile_mode = STANDING_POSE3;
                          }
                          pres_mobile_mode = mobile_mode;
                          break;
      case (RC100_BTN_2): if (head_yaw < 2228) // 2248
                            head_yaw += 20;
                          break;
      case (RC100_BTN_4): if (head_yaw > 1868) // 1848
                            head_yaw -= 20;
                          break;
      case (RC100_BTN_1): if (head_pitch > 2580) // 2560
                            head_pitch -= 20;
                          break;
      case (RC100_BTN_3): if (head_pitch < 3052) // 3072
                            head_pitch += 20;
                          break;
      case (RC100_BTN_6): if (pres_mobile_mode == NONE)
                            mobile_mode = LOOK_AROUND_LEFT;
                          pres_mobile_mode = mobile_mode;
                          break;
      case (RC100_BTN_5): if (pres_mobile_mode == NONE)
                            mobile_mode = LOOK_AROUND_RIGHT;
                          pres_mobile_mode = mobile_mode;
                          break;

      case (RC100_BTN_U + RC100_BTN_L): if (pres_mobile_mode == NONE)
                                        {
                                          mobile_mode = FORWARD1;
                                        }
                                        else if (pres_mobile_mode == FORWARD1)
                                        {
                                          mobile_mode = RLEFT_FORWARD3;
                                        }
                                        else if (pres_mobile_mode == FORWARD2)
                                        {
                                          mobile_mode = RLEFT_FORWARD2;
                                        }
                                        else if (pres_mobile_mode == FORWARD3)
                                        {
                                          mobile_mode = RLEFT_FORWARD3;
                                        }
                                        pres_mobile_mode = mobile_mode;
                                        break;

      case (RC100_BTN_U + RC100_BTN_R): if (pres_mobile_mode == NONE)
                                        {
                                          mobile_mode = FORWARD1;
                                        }
                                        else if (pres_mobile_mode == FORWARD1)
                                        {
                                          mobile_mode = RRIGHT_FORWARD3;
                                        }
                                        else if (pres_mobile_mode == FORWARD2)
                                        {
                                          mobile_mode = RRIGHT_FORWARD2;
                                        }
                                        else if (pres_mobile_mode == FORWARD3)
                                        {
                                          mobile_mode = RRIGHT_FORWARD3;
                                        }
                                        pres_mobile_mode = mobile_mode;
                                        break;


      case (RC100_BTN_U + RC100_BTN_D): if (pres_mobile_mode == NONE)
                                        {
                                          mobile_mode = NONE;
                                        }
                                        else if (pres_mobile_mode == FORWARD2)
                                        {
                                          mobile_mode = STANDING_POSE2;
                                        }
                                        else if (pres_mobile_mode == FORWARD3)
                                        {
                                          mobile_mode = STANDING_POSE3;
                                        }
                                        else if ((pres_mobile_mode == STANDING_POSE2) || (pres_mobile_mode == STANDING_POSE3))
                                        {
                                          mobile_mode = NONE;
                                        }
                                        pres_mobile_mode = mobile_mode;
                                        break;

      case (RC100_BTN_D + RC100_BTN_1): if (pres_mobile_mode == NONE)
                                        {
                                          mobile_mode = GRUMBLE1;
                                        }
                                        pres_mobile_mode = mobile_mode;
                                        break;
      case (RC100_BTN_D + RC100_BTN_2): if (pres_mobile_mode == NONE)
                                        {
                                          mobile_mode = GRUMBLE2;
                                        }
                                        pres_mobile_mode = mobile_mode;
                                        break;

      default: mobile_mode = NONE;
               pres_mobile_mode = mobile_mode;
               break;
     }

     return mobile_mode;
   }

   void setParams()
   {
     switch (mobile_mode)
     {
       case FORWARD1:               motion_all_num = 8;
                                    shoulder_left_rear_angle[0] = 0;    shoulder_right_rear_angle[0] = 0;    shoulder_left_front_angle[0] = 0;    shoulder_right_front_angle[0] = 0;
                                    leg_left_rear_angle[0] = 0;         leg_right_rear_angle[0] = 0;         leg_left_front_angle[0] = 0;         leg_right_front_angle[0] = 0;
                                    time_duration[0] = 0.4 / FASTER;    time_space[0] = .0;
                                    shoulder_left_rear_angle[1] = 0;    shoulder_right_rear_angle[1] = 0;    shoulder_left_front_angle[1] = 0;    shoulder_right_front_angle[1] = 0;
                                    leg_left_rear_angle[1] = 256;       leg_right_rear_angle[1] = 0;         leg_left_front_angle[1] = 0;         leg_right_front_angle[1] = 0;
                                    time_duration[1] = 0.1 / FASTER;    time_space[1] = .0;
                                    shoulder_left_rear_angle[2] = 700;  shoulder_right_rear_angle[2] = 0;    shoulder_left_front_angle[2] = 0;    shoulder_right_front_angle[2] = 0;
                                    leg_left_rear_angle[2] = 512;       leg_right_rear_angle[2] = 0;         leg_left_front_angle[2] = 0;         leg_right_front_angle[2] = 0;
                                    time_duration[2] = 0.2 / FASTER;    time_space[2] = .0;
                                    shoulder_left_rear_angle[3] = 700;  shoulder_right_rear_angle[3] = 0;    shoulder_left_front_angle[3] = 0;    shoulder_right_front_angle[3] = 0;
                                    leg_left_rear_angle[3] = 0;         leg_right_rear_angle[3] = 0;         leg_left_front_angle[3] = 0;         leg_right_front_angle[3] = 0;
                                    time_duration[3] = 0.1 / FASTER;    time_space[3] = .0;


                                    shoulder_left_rear_angle[4] = 700;  shoulder_right_rear_angle[4] = 0;    shoulder_left_front_angle[4] = 0;    shoulder_right_front_angle[4] = 0;
                                    leg_left_rear_angle[4] = 0;         leg_right_rear_angle[4] = 0;         leg_left_front_angle[4] = -512;      leg_right_front_angle[4] = 0;
                                    time_duration[4] = 0.1 / FASTER;    time_space[4] = .0;
                                    shoulder_left_rear_angle[5] = 0;    shoulder_right_rear_angle[5] = 0;    shoulder_left_front_angle[5] = 0;    shoulder_right_front_angle[5] = 700;
                                    leg_left_rear_angle[5] = 0;         leg_right_rear_angle[5] = -256;      leg_left_front_angle[5] = 0;         leg_right_front_angle[5] = 0;
                                    time_duration[5] = 0.4 / FASTER;    time_space[5] = .0;
                                    shoulder_left_rear_angle[6] = 0;    shoulder_right_rear_angle[6] = -700; shoulder_left_front_angle[6] = 0;    shoulder_right_front_angle[6] = 700;
                                    leg_left_rear_angle[6] = 0;         leg_right_rear_angle[6] = -512;      leg_left_front_angle[6] = 0;         leg_right_front_angle[6] = 0;
                                    time_duration[6] = 0.2 / FASTER;    time_space[6] = .0;
                                    shoulder_left_rear_angle[7] = 0;    shoulder_right_rear_angle[7] = -700; shoulder_left_front_angle[7] = 0;    shoulder_right_front_angle[7] = 700;
                                    leg_left_rear_angle[7] = 0;         leg_right_rear_angle[7] = 0;         leg_left_front_angle[7] = 0;         leg_right_front_angle[7] = 0;
                                    time_duration[7] = 0.1 / FASTER;    time_space[7] = .0;

                                    mobile_mode = FORWARD3;
                                    pres_mobile_mode = mobile_mode;

                                    break;
       case FORWARD2:               motion_all_num = 5;
                                    shoulder_left_rear_angle[0] = 0;    shoulder_right_rear_angle[0] = -700; shoulder_left_front_angle[0] = 0;    shoulder_right_front_angle[0] = 700;
                                    leg_left_rear_angle[0] = 0;         leg_right_rear_angle[0] = 0;         leg_left_front_angle[0] = 0;         leg_right_front_angle[0] = 256;
                                    time_duration[0] = 0.1 / FASTER;       time_space[0] = .0;
                                    shoulder_left_rear_angle[1] = 0;    shoulder_right_rear_angle[1] = -700; shoulder_left_front_angle[1] = 0;    shoulder_right_front_angle[1] = 0;
                                    leg_left_rear_angle[1] = 0;         leg_right_rear_angle[1] = 0;         leg_left_front_angle[1] = 0;         leg_right_front_angle[1] = 512;
                                    time_duration[1] = 0.2 / FASTER;       time_space[1] = .0;
                                    shoulder_left_rear_angle[2] = 0;    shoulder_right_rear_angle[2] = 0;    shoulder_left_front_angle[2] = -700; shoulder_right_front_angle[2] = 0;
                                    leg_left_rear_angle[2] = 512;       leg_right_rear_angle[2] = 0;         leg_left_front_angle[2] = 0;         leg_right_front_angle[2] = 0;
                                    time_duration[2] = 0.4 / FASTER;       time_space[2] = .0;
                                    shoulder_left_rear_angle[3] = 700;  shoulder_right_rear_angle[3] = 0;    shoulder_left_front_angle[3] = -700; shoulder_right_front_angle[3] = 0;
                                    leg_left_rear_angle[3] = 512;       leg_right_rear_angle[3] = 0;         leg_left_front_angle[3] = 0;         leg_right_front_angle[3] = 0;
                                    time_duration[3] = 0.2 / FASTER;       time_space[3] = .0;
                                    shoulder_left_rear_angle[4] = 700;  shoulder_right_rear_angle[4] = 0;    shoulder_left_front_angle[4] = -700; shoulder_right_front_angle[4] = 0;
                                    leg_left_rear_angle[4] = 0;         leg_right_rear_angle[4] = 0;         leg_left_front_angle[4] = 0;         leg_right_front_angle[4] = 0;
                                    time_duration[4] = 0.1 / FASTER;       time_space[4] = .0;
                                    break;
       case FORWARD3:               motion_all_num = 5;
                                    shoulder_left_rear_angle[0] = 700; shoulder_right_rear_angle[0] = 0;     shoulder_left_front_angle[0] = -700; shoulder_right_front_angle[0] = 0;
                                    leg_left_rear_angle[0] = 0;        leg_right_rear_angle[0] = 0;          leg_left_front_angle[0] = -256;      leg_right_front_angle[0] = 0;
                                    time_duration[0] = 0.1 / FASTER;      time_space[0] = .0;
                                    shoulder_left_rear_angle[1] = 700; shoulder_right_rear_angle[1] = 0;     shoulder_left_front_angle[1] = 0;    shoulder_right_front_angle[1] = 0;
                                    leg_left_rear_angle[1] = 0;        leg_right_rear_angle[1] = 0;          leg_left_front_angle[1] = -512;      leg_right_front_angle[1] = 0;
                                    time_duration[1] = 0.2 / FASTER;      time_space[1] = .0;
                                    shoulder_left_rear_angle[2] = 0;   shoulder_right_rear_angle[2] = 0;     shoulder_left_front_angle[2] = 0;    shoulder_right_front_angle[2] = 700;
                                    leg_left_rear_angle[2] = 0;        leg_right_rear_angle[2] = -512;       leg_left_front_angle[2] = 0;         leg_right_front_angle[2] = 0;
                                    time_duration[2] = 0.4 / FASTER;      time_space[2] = .0;
                                    shoulder_left_rear_angle[3] = 0;   shoulder_right_rear_angle[3] = -700;  shoulder_left_front_angle[3] = 0;    shoulder_right_front_angle[3] = 700;
                                    leg_left_rear_angle[3] = 0;        leg_right_rear_angle[3] = -512;       leg_left_front_angle[3] = 0;         leg_right_front_angle[3] = 0;
                                    time_duration[3] = 0.2 / FASTER;      time_space[3] = .0;
                                    shoulder_left_rear_angle[4] = 0;   shoulder_right_rear_angle[4] = -700;  shoulder_left_front_angle[4] = 0;    shoulder_right_front_angle[4] = 700;
                                    leg_left_rear_angle[4] = 0;        leg_right_rear_angle[4] = 0;          leg_left_front_angle[4] = 0;         leg_right_front_angle[4] = 0;
                                    time_duration[4] = 0.1 / FASTER;      time_space[4] = .0;
                                    break;

      case RLEFT_FORWARD2:          motion_all_num = 9;
                                    shoulder_left_rear_angle[0] = 700;    shoulder_right_rear_angle[0] = 0; shoulder_left_front_angle[0] = -700;    shoulder_right_front_angle[0] = 0;
                                    leg_left_rear_angle[0] = 0;         leg_right_rear_angle[0] = 0;         leg_left_front_angle[0] = -256;         leg_right_front_angle[0] = 0;
                                    time_duration[0] = 0.1 / FASTER;       time_space[0] = .0;
                                    shoulder_left_rear_angle[1] = 700;    shoulder_right_rear_angle[1] = 0; shoulder_left_front_angle[1] = 0;    shoulder_right_front_angle[1] = 0;
                                    leg_left_rear_angle[1] = 0;         leg_right_rear_angle[1] = 0;         leg_left_front_angle[1] = -512;         leg_right_front_angle[1] = 0;
                                    time_duration[1] = 0.2 / FASTER;       time_space[1] = .0;

                                    shoulder_left_rear_angle[2] = 700;    shoulder_right_rear_angle[2] = 500;    shoulder_left_front_angle[2] = 0; shoulder_right_front_angle[2] = 700;
                                    leg_left_rear_angle[2] = 0;       leg_right_rear_angle[2] = 0;         leg_left_front_angle[2] = 0;         leg_right_front_angle[2] = 0;
                                    time_duration[2] = 0.4 / FASTER;       time_space[2] = .0;
                                    shoulder_left_rear_angle[3] = 700;    shoulder_right_rear_angle[3] = 500;    shoulder_left_front_angle[3] = 0; shoulder_right_front_angle[3] = 700;
                                    leg_left_rear_angle[3] = 256;       leg_right_rear_angle[3] = 0;         leg_left_front_angle[3] = 0;         leg_right_front_angle[3] = 0;
                                    time_duration[3] = 0.1 / FASTER;       time_space[3] = .0;
                                    shoulder_left_rear_angle[4] = 0;  shoulder_right_rear_angle[4] = 500;    shoulder_left_front_angle[4] = 0; shoulder_right_front_angle[4] = 700;
                                    leg_left_rear_angle[4] = 512;       leg_right_rear_angle[4] = 0;         leg_left_front_angle[4] = 0;         leg_right_front_angle[4] = 0;
                                    time_duration[4] = 0.2 / FASTER;       time_space[4] = .0;
                                    shoulder_left_rear_angle[5] = 0;  shoulder_right_rear_angle[5] = 500;    shoulder_left_front_angle[5] = 0; shoulder_right_front_angle[5] = 700;
                                    leg_left_rear_angle[5] = 0;       leg_right_rear_angle[5] = 0;         leg_left_front_angle[5] = 0;         leg_right_front_angle[5] = 0;
                                    time_duration[5] = 0.1 / FASTER;       time_space[5] = .0;
                                    shoulder_left_rear_angle[6] = 0;  shoulder_right_rear_angle[6] = 500;    shoulder_left_front_angle[6] = 0; shoulder_right_front_angle[6] = 700;
                                    leg_left_rear_angle[6] = 256;         leg_right_rear_angle[6] = 0;         leg_left_front_angle[6] = 0;         leg_right_front_angle[6] = 0;
                                    time_duration[6] = 0.1 / FASTER;       time_space[6] = .0;

                                    shoulder_left_rear_angle[7] = 0;  shoulder_right_rear_angle[7] = -700;    shoulder_left_front_angle[7] = 0; shoulder_right_front_angle[7] = 700;
                                    leg_left_rear_angle[7] = 0;         leg_right_rear_angle[7] = -512;         leg_left_front_angle[7] = 0;         leg_right_front_angle[7] = 0;
                                    time_duration[7] = 0.2 / FASTER;       time_space[7] = .0;
                                    shoulder_left_rear_angle[8] = 0;  shoulder_right_rear_angle[8] = -700;    shoulder_left_front_angle[8] = 0; shoulder_right_front_angle[8] = 700;
                                    leg_left_rear_angle[8] = 0;         leg_right_rear_angle[8] = 0;         leg_left_front_angle[8] = 0;         leg_right_front_angle[8] = 0;
                                    time_duration[8] = 0.1 / FASTER;       time_space[8] = .0;

                                    mobile_mode = FORWARD3;
                                    pres_mobile_mode = mobile_mode;
                                    break;

      case RLEFT_FORWARD3:          motion_all_num = 6;
                                    shoulder_left_rear_angle[0] = 0; shoulder_right_rear_angle[0] = -700;     shoulder_left_front_angle[0] = 0; shoulder_right_front_angle[0] = 700;
                                    leg_left_rear_angle[0] = 0;        leg_right_rear_angle[0] = 0;          leg_left_front_angle[0] = 0;      leg_right_front_angle[0] = 256;
                                    time_duration[0] = 0.1 / FASTER;      time_space[0] = .0;
                                    shoulder_left_rear_angle[1] = 0; shoulder_right_rear_angle[1] = -700;     shoulder_left_front_angle[1] = 0;    shoulder_right_front_angle[1] = -700;
                                    leg_left_rear_angle[1] = 0;        leg_right_rear_angle[1] = 0;          leg_left_front_angle[1] = 0;      leg_right_front_angle[1] = 512;
                                    time_duration[1] = 0.2 / FASTER;      time_space[1] = .0;
                                    shoulder_left_rear_angle[2] = 0; shoulder_right_rear_angle[2] = -700;     shoulder_left_front_angle[2] = 0;    shoulder_right_front_angle[2] = -700;
                                    leg_left_rear_angle[2] = 0;        leg_right_rear_angle[2] = 0;          leg_left_front_angle[2] = 0;      leg_right_front_angle[2] = 0;
                                    time_duration[2] = 0.1 / FASTER;      time_space[2] = .0;
                                    shoulder_left_rear_angle[2 + 1] = 700;   shoulder_right_rear_angle[2 + 1] = 0;     shoulder_left_front_angle[2 + 1] = 0;    shoulder_right_front_angle[2 + 1] = 0;
                                    leg_left_rear_angle[2 + 1] = 0;        leg_right_rear_angle[2 + 1] = 0;       leg_left_front_angle[2 + 1] = -256;         leg_right_front_angle[2 + 1] = 0;
                                    time_duration[2 + 1] = 0.4 / FASTER;      time_space[2 + 1] = .0;
                                    shoulder_left_rear_angle[3 + 1] = 700;   shoulder_right_rear_angle[3 + 1] = 0;  shoulder_left_front_angle[3 + 1] = -700;    shoulder_right_front_angle[3 + 1] = 0;
                                    leg_left_rear_angle[3 + 1] = 0;        leg_right_rear_angle[3 + 1] = 0;       leg_left_front_angle[3 + 1] = -512;         leg_right_front_angle[3 + 1] = 0;
                                    time_duration[3 + 1] = 0.2 / FASTER;      time_space[3 + 1] = .0;
                                    shoulder_left_rear_angle[4 + 1] = 700;   shoulder_right_rear_angle[4 + 1] = 0;  shoulder_left_front_angle[4 + 1] = -700;    shoulder_right_front_angle[4 + 1] = 0;
                                    leg_left_rear_angle[4 + 1] = 0;        leg_right_rear_angle[4 + 1] = 0;          leg_left_front_angle[4 + 1] = 0;         leg_right_front_angle[4 + 1] = 0;
                                    time_duration[4 + 1] = 0.1 / FASTER;      time_space[4 + 1] = .0;

                                    mobile_mode = FORWARD2;
                                    pres_mobile_mode = mobile_mode;
                                    break;

     case RRIGHT_FORWARD2:         motion_all_num = 6;
                                   shoulder_left_rear_angle[0] = 700; shoulder_right_rear_angle[0] = 0;     shoulder_left_front_angle[0] = -700; shoulder_right_front_angle[0] = 0;
                                   leg_left_rear_angle[0] = 0;        leg_right_rear_angle[0] = 0;          leg_left_front_angle[0] = -256;      leg_right_front_angle[0] = 0;
                                   time_duration[0] = 0.1 / FASTER;      time_space[0] = .0;
                                   shoulder_left_rear_angle[1] = 700; shoulder_right_rear_angle[1] = 0;     shoulder_left_front_angle[1] = 700;    shoulder_right_front_angle[1] = 0;
                                   leg_left_rear_angle[1] = 0;        leg_right_rear_angle[1] = 0;          leg_left_front_angle[1] = -512;      leg_right_front_angle[1] = 0;
                                   time_duration[1] = 0.2 / FASTER;      time_space[1] = .0;
                                   shoulder_left_rear_angle[2] = 700; shoulder_right_rear_angle[2] = 0;     shoulder_left_front_angle[2] = 700;    shoulder_right_front_angle[2] = 0;
                                   leg_left_rear_angle[2] = 0;        leg_right_rear_angle[2] = 0;          leg_left_front_angle[2] = 0;      leg_right_front_angle[2] = 0;
                                   time_duration[2] = 0.1 / FASTER;      time_space[2] = .0;
                                   shoulder_left_rear_angle[2 + 1] = 0;   shoulder_right_rear_angle[2 + 1] = -700;     shoulder_left_front_angle[2 + 1] = 0;    shoulder_right_front_angle[2 + 1] = 0;
                                   leg_left_rear_angle[2 + 1] = 0;        leg_right_rear_angle[2 + 1] = 0;       leg_left_front_angle[2 + 1] = 0;         leg_right_front_angle[2 + 1] = 256;
                                   time_duration[2 + 1] = 0.4 / FASTER;      time_space[2 + 1] = .0;
                                   shoulder_left_rear_angle[3 + 1] = 0;   shoulder_right_rear_angle[3 + 1] = -700;  shoulder_left_front_angle[3 + 1] = 0;    shoulder_right_front_angle[3 + 1] = 700;
                                   leg_left_rear_angle[3 + 1] = 0;        leg_right_rear_angle[3 + 1] = 0;       leg_left_front_angle[3 + 1] = 0;         leg_right_front_angle[3 + 1] = 512;
                                   time_duration[3 + 1] = 0.2 / FASTER;      time_space[3 + 1] = .0;
                                   shoulder_left_rear_angle[4 + 1] = 0;   shoulder_right_rear_angle[4 + 1] = -700;  shoulder_left_front_angle[4 + 1] = 0;    shoulder_right_front_angle[4 + 1] = 700;
                                   leg_left_rear_angle[4 + 1] = 0;        leg_right_rear_angle[4 + 1] = 0;          leg_left_front_angle[4 + 1] = 0;         leg_right_front_angle[4 + 1] = 0;
                                   time_duration[4 + 1] = 0.1 / FASTER;      time_space[4 + 1] = .0;

                                   mobile_mode = FORWARD3;
                                   pres_mobile_mode = mobile_mode;
                                   break;
     case RRIGHT_FORWARD3:         motion_all_num = 9;
                                   shoulder_left_rear_angle[0] = 0;    shoulder_right_rear_angle[0] = -700; shoulder_left_front_angle[0] = 0;    shoulder_right_front_angle[0] = 700;
                                   leg_left_rear_angle[0] = 0;         leg_right_rear_angle[0] = 0;         leg_left_front_angle[0] = 0;         leg_right_front_angle[0] = 256;
                                   time_duration[0] = 0.1 / FASTER;       time_space[0] = .0;
                                   shoulder_left_rear_angle[1] = 0;    shoulder_right_rear_angle[1] = -700; shoulder_left_front_angle[1] = 0;    shoulder_right_front_angle[1] = 0;
                                   leg_left_rear_angle[1] = 0;         leg_right_rear_angle[1] = 0;         leg_left_front_angle[1] = 0;         leg_right_front_angle[1] = 512;
                                   time_duration[1] = 0.2 / FASTER;       time_space[1] = .0;
                                   shoulder_left_rear_angle[2] = -500;    shoulder_right_rear_angle[2] = -700;    shoulder_left_front_angle[2] = -700; shoulder_right_front_angle[2] = 0;
                                   leg_left_rear_angle[2] = 0;       leg_right_rear_angle[2] = 0;         leg_left_front_angle[2] = 0;         leg_right_front_angle[2] = 0;
                                   time_duration[2] = 0.4 / FASTER;       time_space[2] = .0;
                                   shoulder_left_rear_angle[3] = -500;    shoulder_right_rear_angle[3] = -700;    shoulder_left_front_angle[3] = -700; shoulder_right_front_angle[3] = 0;
                                   leg_left_rear_angle[3] = 0;       leg_right_rear_angle[3] = -256;         leg_left_front_angle[3] = 0;         leg_right_front_angle[3] = 0;
                                   time_duration[3] = 0.1 / FASTER;       time_space[3] = .0;
                                   shoulder_left_rear_angle[4] = -500;  shoulder_right_rear_angle[4] = 0;    shoulder_left_front_angle[4] = -700; shoulder_right_front_angle[4] = 0;
                                   leg_left_rear_angle[4] = 0;       leg_right_rear_angle[4] = -512;         leg_left_front_angle[4] = 0;         leg_right_front_angle[4] = 0;
                                   time_duration[4] = 0.2 / FASTER;       time_space[4] = .0;
                                   shoulder_left_rear_angle[5] = -500;  shoulder_right_rear_angle[5] = 0;    shoulder_left_front_angle[5] = -700; shoulder_right_front_angle[5] = 0;
                                   leg_left_rear_angle[5] = 0;       leg_right_rear_angle[5] = 0;         leg_left_front_angle[5] = 0;         leg_right_front_angle[5] = 0;
                                   time_duration[5] = 0.1 / FASTER;       time_space[5] = .0;
                                   shoulder_left_rear_angle[6] = -500;  shoulder_right_rear_angle[6] = 0;    shoulder_left_front_angle[6] = -700; shoulder_right_front_angle[6] = 0;
                                   leg_left_rear_angle[6] = 256;         leg_right_rear_angle[6] = 0;         leg_left_front_angle[6] = 0;         leg_right_front_angle[6] = 0;
                                   time_duration[6] = 0.1 / FASTER;       time_space[6] = .0;
                                   shoulder_left_rear_angle[7] = 700;  shoulder_right_rear_angle[7] = 0;    shoulder_left_front_angle[7] = -700; shoulder_right_front_angle[7] = 0;
                                   leg_left_rear_angle[7] = 512;         leg_right_rear_angle[7] = 0;         leg_left_front_angle[7] = 0;         leg_right_front_angle[7] = 0;
                                   time_duration[7] = 0.2 / FASTER;       time_space[7] = .0;
                                   shoulder_left_rear_angle[8] = 700;  shoulder_right_rear_angle[8] = 0;    shoulder_left_front_angle[8] = -700; shoulder_right_front_angle[8] = 0;
                                   leg_left_rear_angle[8] = 0;         leg_right_rear_angle[8] = 0;         leg_left_front_angle[8] = 0;         leg_right_front_angle[8] = 0;
                                   time_duration[8] = 0.1 / FASTER;       time_space[8] = .0;

                                   mobile_mode = FORWARD2;
                                   pres_mobile_mode = mobile_mode;
                                   break;

       case NONE:                   motion_all_num = 1;
                                    shoulder_left_rear_angle[0] = 0;   shoulder_right_rear_angle[0] = 0;     shoulder_left_front_angle[0] = 0;    shoulder_right_front_angle[0] = 0;
                                    leg_left_rear_angle[0] = 0;        leg_right_rear_angle[0] = 0;          leg_left_front_angle[0] = 0;         leg_right_front_angle[0] = 0;
                                    time_duration[0] = 2.0;      time_space[0] = .0;
                                    break;

       case STANDING_POSE2:     motion_all_num = 6;
                                    shoulder_left_rear_angle[0] = 700; shoulder_right_rear_angle[0] = 0;     shoulder_left_front_angle[0] = -700; shoulder_right_front_angle[0] = 0;
                                    leg_left_rear_angle[0] = 0;        leg_right_rear_angle[0] = 0;          leg_left_front_angle[0] = -512;      leg_right_front_angle[0] = 0;
                                    time_duration[0] = 0.1 / FASTER;      time_space[0] = .0;
                                    shoulder_left_rear_angle[1] = 700; shoulder_right_rear_angle[1] = 0;     shoulder_left_front_angle[1] = 0;    shoulder_right_front_angle[1] = 0;
                                    leg_left_rear_angle[1] = 0;        leg_right_rear_angle[1] = 0;          leg_left_front_angle[1] = -512;      leg_right_front_angle[1] = 0;
                                    time_duration[1] = 0.2 / FASTER;      time_space[1] = .0;
                                    shoulder_left_rear_angle[2] = 700; shoulder_right_rear_angle[2] = 0;     shoulder_left_front_angle[2] = 0;    shoulder_right_front_angle[2] = 0;
                                    leg_left_rear_angle[2] = 0;        leg_right_rear_angle[2] = 0;          leg_left_front_angle[2] = 0;         leg_right_front_angle[2] = 0;
                                    time_duration[2] = 0.1 / FASTER;      time_space[2] = .0;
                                    shoulder_left_rear_angle[3] = 700; shoulder_right_rear_angle[3] = 0;     shoulder_left_front_angle[3] = 0;    shoulder_right_front_angle[3] = 0;
                                    leg_left_rear_angle[3] = 512;      leg_right_rear_angle[3] = 0;          leg_left_front_angle[3] = 0;         leg_right_front_angle[3] = 0;
                                    time_duration[3] = 0.2 / FASTER;      time_space[3] = .0;
                                    shoulder_left_rear_angle[4] = 0;   shoulder_right_rear_angle[4] = 0;     shoulder_left_front_angle[4] = 0;    shoulder_right_front_angle[4] = 0;
                                    leg_left_rear_angle[4] = 512;      leg_right_rear_angle[4] = 0;          leg_left_front_angle[4] = 0;         leg_right_front_angle[4] = 0;
                                    time_duration[4] = 0.1 / FASTER;      time_space[4] = .0;
                                    shoulder_left_rear_angle[5] = 0;   shoulder_right_rear_angle[5] = 0;     shoulder_left_front_angle[5] = 0;    shoulder_right_front_angle[5] = 0;
                                    leg_left_rear_angle[5] = 0;        leg_right_rear_angle[5] = 0;          leg_left_front_angle[5] = 0;         leg_right_front_angle[5] = 0;
                                    time_duration[5] = 0.1 / FASTER;      time_space[5] = .0;

                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;
                                    break;

       case STANDING_POSE3:     motion_all_num = 6;
                                    shoulder_left_rear_angle[0] = 0;   shoulder_right_rear_angle[0] = -700;  shoulder_left_front_angle[0] = 0;    shoulder_right_front_angle[0] = 700;
                                    leg_left_rear_angle[0] = 0;        leg_right_rear_angle[0] = 0;          leg_left_front_angle[0] = 0;         leg_right_front_angle[0] = 512;
                                    time_duration[0] = 0.1 / FASTER;      time_space[0] = .0;
                                    shoulder_left_rear_angle[1] = 0;   shoulder_right_rear_angle[1] = -700;  shoulder_left_front_angle[1] = 0;    shoulder_right_front_angle[1] = 0;
                                    leg_left_rear_angle[1] = 0;        leg_right_rear_angle[1] = 0;          leg_left_front_angle[1] = 0;         leg_right_front_angle[1] = 512;
                                    time_duration[1] = 0.2 / FASTER;      time_space[1] = .0;
                                    shoulder_left_rear_angle[2] = 0;   shoulder_right_rear_angle[2] = -700;  shoulder_left_front_angle[2] = 0;    shoulder_right_front_angle[2] = 0;
                                    leg_left_rear_angle[2] = 0;        leg_right_rear_angle[2] = 0;          leg_left_front_angle[2] = 0;         leg_right_front_angle[2] = 0;
                                    time_duration[2] = 0.1 / FASTER;      time_space[2] = .0;
                                    shoulder_left_rear_angle[3] = 0;   shoulder_right_rear_angle[3] = -700;  shoulder_left_front_angle[3] = 0;    shoulder_right_front_angle[3] = 0;
                                    leg_left_rear_angle[3] = 0;        leg_right_rear_angle[3] = -512;       leg_left_front_angle[3] = 0;         leg_right_front_angle[3] = 0;
                                    time_duration[3] = 0.1 / FASTER;      time_space[3] = .0;
                                    shoulder_left_rear_angle[4] = 0;   shoulder_right_rear_angle[4] = 0;     shoulder_left_front_angle[4] = 0;    shoulder_right_front_angle[4] = 0;
                                    leg_left_rear_angle[4] = 0;        leg_right_rear_angle[4] = -512;       leg_left_front_angle[4] = 0;         leg_right_front_angle[4] = 0;
                                    time_duration[4] = 0.1 / FASTER;      time_space[4] = .0;
                                    shoulder_left_rear_angle[5] = 0;   shoulder_right_rear_angle[5] = 0;     shoulder_left_front_angle[5] = 0;    shoulder_right_front_angle[5] = 0;
                                    leg_left_rear_angle[5] = 0;        leg_right_rear_angle[5] = 0;          leg_left_front_angle[5] = 0;         leg_right_front_angle[5] = 0;
                                    time_duration[5] = 0.1 / FASTER;      time_space[5] = .0;

                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;
                                    break;

       case GRUMBLE1:               motion_all_num = 6;
                                    shoulder_left_rear_angle[0] = 0;  shoulder_right_rear_angle[0] = 0;  shoulder_left_front_angle[0] = 0;  shoulder_right_front_angle[0] = 0;
                                    leg_left_rear_angle[0] = 256;       leg_right_rear_angle[0] = -256;      leg_left_front_angle[0] = -256;      leg_right_front_angle[0] = 256;
                                    time_duration[0] = 0.1 / FASTER;       time_space[0] = .0;
                                    shoulder_left_rear_angle[1] = 0;  shoulder_right_rear_angle[1] = 0;  shoulder_left_front_angle[1] = 0;  shoulder_right_front_angle[1] = 0;
                                    leg_left_rear_angle[1] = 0;       leg_right_rear_angle[1] = 0;      leg_left_front_angle[1] = 0;      leg_right_front_angle[1] = 0;
                                    time_duration[1] = 0.1 / FASTER;       time_space[1] = .0;
                                    shoulder_left_rear_angle[0 + 2] = 0;  shoulder_right_rear_angle[0 + 2] = 0;  shoulder_left_front_angle[0 + 2] = 0;  shoulder_right_front_angle[0 + 2] = 0;
                                    leg_left_rear_angle[0 + 2] = 256;       leg_right_rear_angle[0 + 2] = -256;      leg_left_front_angle[0 + 2] = -256;      leg_right_front_angle[0 + 2] = 256;
                                    time_duration[0 + 2] = 0.1 / FASTER;       time_space[0 + 2] = .0;
                                    shoulder_left_rear_angle[1 + 2] = 0;  shoulder_right_rear_angle[1 + 2] = 0;  shoulder_left_front_angle[1 + 2] = 0;  shoulder_right_front_angle[1 + 2] = 0;
                                    leg_left_rear_angle[1 + 2] = 0;       leg_right_rear_angle[1 + 2] = 0;      leg_left_front_angle[1 + 2] = 0;      leg_right_front_angle[1 + 2] = 0;
                                    time_duration[1 + 2] = 0.1 / FASTER;       time_space[1 + 2] = .0;
                                    shoulder_left_rear_angle[0 + 4] = 0;  shoulder_right_rear_angle[0 + 4] = 0;  shoulder_left_front_angle[0 + 4] = 0;  shoulder_right_front_angle[0 + 4] = 0;
                                    leg_left_rear_angle[0 + 4] = 256;       leg_right_rear_angle[0 + 4] = -256;      leg_left_front_angle[0 + 4] = -256;      leg_right_front_angle[0 + 4] = 256;
                                    time_duration[0 + 4] = 0.1 / FASTER;       time_space[0 + 4] = .0;
                                    shoulder_left_rear_angle[1 + 4] = 0;  shoulder_right_rear_angle[1 + 4] = 0;  shoulder_left_front_angle[1 + 4] = 0;  shoulder_right_front_angle[1 + 4] = 0;
                                    leg_left_rear_angle[1 + 4] = 0;       leg_right_rear_angle[1 + 4] = 0;      leg_left_front_angle[1 + 4] = 0;      leg_right_front_angle[1 + 4] = 0;
                                    time_duration[1 + 4] = 0.1 / FASTER;       time_space[1 + 4] = .0;

                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;
                                    break;

       case GRUMBLE2:               motion_all_num = 6;
                                    shoulder_left_rear_angle[0] = 256;  shoulder_right_rear_angle[0] = -256;  shoulder_left_front_angle[0] = -256;  shoulder_right_front_angle[0] = 256;
                                    leg_left_rear_angle[0] = 0;       leg_right_rear_angle[0] = 0;      leg_left_front_angle[0] = 0;      leg_right_front_angle[0] = 0;
                                    time_duration[0] = 0.1 / FASTER;       time_space[0] = .0;
                                    shoulder_left_rear_angle[1] = 0;  shoulder_right_rear_angle[1] = 0;  shoulder_left_front_angle[1] = 0;  shoulder_right_front_angle[1] = 0;
                                    leg_left_rear_angle[1] = 0;       leg_right_rear_angle[1] = 0;      leg_left_front_angle[1] = 0;      leg_right_front_angle[1] = 0;
                                    time_duration[1] = 0.1 / FASTER;       time_space[1] = .0;
                                    shoulder_left_rear_angle[0 + 2] = 256;  shoulder_right_rear_angle[0 + 2] = -256;  shoulder_left_front_angle[0 + 2] = -256;  shoulder_right_front_angle[0 + 2] = 256;
                                    leg_left_rear_angle[0 + 2] = 0;       leg_right_rear_angle[0 + 2] = 0;      leg_left_front_angle[0 + 2] = 0;      leg_right_front_angle[0 + 2] = 0;
                                    time_duration[0 + 2] = 0.1 / FASTER;       time_space[0 + 2] = .0;
                                    shoulder_left_rear_angle[1 + 2] = 0;  shoulder_right_rear_angle[1 + 2] = 0;  shoulder_left_front_angle[1 + 2] = 0;  shoulder_right_front_angle[1 + 2] = 0;
                                    leg_left_rear_angle[1 + 2] = 0;       leg_right_rear_angle[1 + 2] = 0;      leg_left_front_angle[1 + 2] = 0;      leg_right_front_angle[1 + 2] = 0;
                                    time_duration[1 + 2] = 0.1 / FASTER;       time_space[1 + 2] = .0;
                                    shoulder_left_rear_angle[0 + 4] = 256;  shoulder_right_rear_angle[0 + 4] = -256;  shoulder_left_front_angle[0 + 4] = -256;  shoulder_right_front_angle[0 + 4] = 256;
                                    leg_left_rear_angle[0 + 4] = 0;       leg_right_rear_angle[0 + 4] = 0;      leg_left_front_angle[0 + 4] = 0;      leg_right_front_angle[0 + 4] = 0;
                                    time_duration[0 + 4] = 0.1 / FASTER;       time_space[0 + 4] = .0;
                                    shoulder_left_rear_angle[1 + 4] = 0;  shoulder_right_rear_angle[1 + 4] = 0;  shoulder_left_front_angle[1 + 4] = 0;  shoulder_right_front_angle[1 + 4] = 0;
                                    leg_left_rear_angle[1 + 4] = 0;       leg_right_rear_angle[1 + 4] = 0;      leg_left_front_angle[1 + 4] = 0;      leg_right_front_angle[1 + 4] = 0;
                                    time_duration[1 + 4] = 0.1 / FASTER;       time_space[1 + 4] = .0;

                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;
                                    break;

       case LOOK_AROUND_LEFT:       motion_all_num = 2;
                                    shoulder_left_rear_angle[0] = 700;  shoulder_right_rear_angle[0] = 700;  shoulder_left_front_angle[0] = 700;  shoulder_right_front_angle[0] = 700;
                                    leg_left_rear_angle[0] = 0;       leg_right_rear_angle[0] = 0;      leg_left_front_angle[0] = 0;      leg_right_front_angle[0] = 0;
                                    time_duration[0] = 2.0 / FASTER;       time_space[0] = 2.0;
                                    shoulder_left_rear_angle[1] = 0;  shoulder_right_rear_angle[1] = 0;  shoulder_left_front_angle[1] = 0;  shoulder_right_front_angle[1] = 0;
                                    leg_left_rear_angle[1] = 0;       leg_right_rear_angle[1] = 0;      leg_left_front_angle[1] = 0;      leg_right_front_angle[1] = 0;
                                    time_duration[1] = 2.0 / FASTER;       time_space[1] = .0;

                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;
                                    break;
       case LOOK_AROUND_RIGHT:       motion_all_num = 2;
                                    shoulder_left_rear_angle[0] = -700;  shoulder_right_rear_angle[0] = -700;  shoulder_left_front_angle[0] = -700;  shoulder_right_front_angle[0] = -700;
                                    leg_left_rear_angle[0] = 0;       leg_right_rear_angle[0] = 0;      leg_left_front_angle[0] = 0;      leg_right_front_angle[0] = 0;
                                    time_duration[0] = 2.0 / FASTER;       time_space[0] = 2.0;
                                    shoulder_left_rear_angle[1] = 0;  shoulder_right_rear_angle[1] = 0;  shoulder_left_front_angle[1] = 0;  shoulder_right_front_angle[1] = 0;
                                    leg_left_rear_angle[1] = 0;       leg_right_rear_angle[1] = 0;      leg_left_front_angle[1] = 0;      leg_right_front_angle[1] = 0;
                                    time_duration[1] = 2.0 / FASTER;       time_space[1] = .0;

                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;
                                    break;

       case RLEFT:                  motion_all_num = 13;
                                    shoulder_left_rear_angle[0] = 500;  shoulder_right_rear_angle[0] = 500;  shoulder_left_front_angle[0] = 500;  shoulder_right_front_angle[0] = 500;
                                    leg_left_rear_angle[0] = 0;         leg_right_rear_angle[0] = 0;         leg_left_front_angle[0] = 0;         leg_right_front_angle[0] = 0;
                                    time_duration[0] = 0.1 / FASTER;       time_space[0] = .0;

                                    shoulder_left_rear_angle[1] = 500;  shoulder_right_rear_angle[1] = 500;  shoulder_left_front_angle[1] = 500;  shoulder_right_front_angle[1] = 500;
                                    leg_left_rear_angle[1] = 0;         leg_right_rear_angle[1] = -256;      leg_left_front_angle[1] = 0;         leg_right_front_angle[1] = 0;
                                    time_duration[1] = 0.1 / FASTER;       time_space[1] = .0;
                                    shoulder_left_rear_angle[2] = 500;  shoulder_right_rear_angle[2] = 0;    shoulder_left_front_angle[2] = 500;  shoulder_right_front_angle[2] = 500;
                                    leg_left_rear_angle[2] = 0;         leg_right_rear_angle[2] = -512;      leg_left_front_angle[2] = 0;         leg_right_front_angle[2] = 0;
                                    time_duration[2] = 0.2 / FASTER;       time_space[2] = .0;
                                    shoulder_left_rear_angle[3] = 500;  shoulder_right_rear_angle[3] = 0;    shoulder_left_front_angle[3] = 500;  shoulder_right_front_angle[3] = 500;
                                    leg_left_rear_angle[3] = 0;         leg_right_rear_angle[3] = 0;         leg_left_front_angle[3] = 0;         leg_right_front_angle[3] = 0;
                                    time_duration[3] = 0.1 / FASTER;       time_space[3] = .0;

                                    shoulder_left_rear_angle[4] = 500;  shoulder_right_rear_angle[4] = 0;    shoulder_left_front_angle[4] = 500;  shoulder_right_front_angle[4] = 500;
                                    leg_left_rear_angle[4] = 0;         leg_right_rear_angle[4] = 0;         leg_left_front_angle[4] = 0;         leg_right_front_angle[4] = 256;
                                    time_duration[4] = 0.1 / FASTER;       time_space[4] = .0;
                                    shoulder_left_rear_angle[5] = 500;  shoulder_right_rear_angle[5] = 0;    shoulder_left_front_angle[5] = 500;  shoulder_right_front_angle[5] = 0;
                                    leg_left_rear_angle[5] = 0;         leg_right_rear_angle[5] = 0;         leg_left_front_angle[5] = 0;         leg_right_front_angle[5] = 512;
                                    time_duration[5] = 0.2 / FASTER;       time_space[5] = .0;
                                    shoulder_left_rear_angle[6] = 500;  shoulder_right_rear_angle[6] = 0;    shoulder_left_front_angle[6] = 500;  shoulder_right_front_angle[6] = 0;
                                    leg_left_rear_angle[6] = 0;         leg_right_rear_angle[6] = 0;         leg_left_front_angle[6] = 0;         leg_right_front_angle[6] = 0;
                                    time_duration[6] = 0.1 / FASTER;       time_space[6] = .0;

                                    shoulder_left_rear_angle[7] = 500;    shoulder_right_rear_angle[7] = 0;    shoulder_left_front_angle[7] = 500;    shoulder_right_front_angle[7] = 0;
                                    leg_left_rear_angle[7] = 0;       leg_right_rear_angle[7] = 0;         leg_left_front_angle[7] = -256;         leg_right_front_angle[7] = 0;
                                    time_duration[7] = 0.1 / FASTER;       time_space[7] = .0;
                                    shoulder_left_rear_angle[8] = 500;  shoulder_right_rear_angle[8] = 0;    shoulder_left_front_angle[8] = 0;    shoulder_right_front_angle[8] = 0;
                                    leg_left_rear_angle[8] = 0;       leg_right_rear_angle[8] = 0;         leg_left_front_angle[8] = -512;         leg_right_front_angle[8] = 0;
                                    time_duration[8] = 0.2 / FASTER;       time_space[8] = .0;
                                    shoulder_left_rear_angle[9] = 500;  shoulder_right_rear_angle[9] = 0;    shoulder_left_front_angle[9] = 0;    shoulder_right_front_angle[9] = 0;
                                    leg_left_rear_angle[9] = 0;         leg_right_rear_angle[9] = 0;         leg_left_front_angle[9] = 0;         leg_right_front_angle[9] = 0;
                                    time_duration[9] = 0.1 / FASTER;       time_space[9] = .0;

                                    shoulder_left_rear_angle[10] = 500;    shoulder_right_rear_angle[10] = 0;    shoulder_left_front_angle[10] = 0;    shoulder_right_front_angle[10] = 0;
                                    leg_left_rear_angle[10] = 256;       leg_right_rear_angle[10] = 0;         leg_left_front_angle[10] = 0;         leg_right_front_angle[10] = 0;
                                    time_duration[10] = 0.1 / FASTER;       time_space[10] = .0;
                                    shoulder_left_rear_angle[11] = 0;  shoulder_right_rear_angle[11] = 0;    shoulder_left_front_angle[11] = 0;    shoulder_right_front_angle[11] = 0;
                                    leg_left_rear_angle[11] = 512;       leg_right_rear_angle[11] = 0;         leg_left_front_angle[11] = 0;         leg_right_front_angle[11] = 0;
                                    time_duration[11] = 0.2 / FASTER;       time_space[11] = .0;
                                    shoulder_left_rear_angle[12] = 0;  shoulder_right_rear_angle[12] = 0;    shoulder_left_front_angle[12] = 0;    shoulder_right_front_angle[12] = 0;
                                    leg_left_rear_angle[12] = 0;         leg_right_rear_angle[12] = 0;         leg_left_front_angle[12] = 0;         leg_right_front_angle[12] = 0;
                                    time_duration[12] = 0.1 / FASTER;       time_space[12] = .0;

                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;

                                    break;
       case RRIGHT:                 motion_all_num = 13;
                                    shoulder_left_rear_angle[0] = -500;    shoulder_right_rear_angle[0] = -500;    shoulder_left_front_angle[0] = -500;    shoulder_right_front_angle[0] = -500;
                                    leg_left_rear_angle[0] = 0;         leg_right_rear_angle[0] = 0;         leg_left_front_angle[0] = 0;         leg_right_front_angle[0] = 0;
                                    time_duration[0] = 0.1 / FASTER;       time_space[0] = .0;

                                    shoulder_left_rear_angle[1] = -500;    shoulder_right_rear_angle[1] = -500;    shoulder_left_front_angle[1] = -500;    shoulder_right_front_angle[1] = -500;
                                    leg_left_rear_angle[1] = 256;       leg_right_rear_angle[1] = 0;         leg_left_front_angle[1] = 0;         leg_right_front_angle[1] = 0;
                                    time_duration[1] = 0.1 / FASTER;       time_space[1] = .0;
                                    shoulder_left_rear_angle[2] = 0;  shoulder_right_rear_angle[2] = -500;    shoulder_left_front_angle[2] = -500;    shoulder_right_front_angle[2] = -500;
                                    leg_left_rear_angle[2] = 512;       leg_right_rear_angle[2] = 0;         leg_left_front_angle[2] = 0;         leg_right_front_angle[2] = 0;
                                    time_duration[2] = 0.2 / FASTER;       time_space[2] = .0;
                                    shoulder_left_rear_angle[3] = 0;  shoulder_right_rear_angle[3] = -500;    shoulder_left_front_angle[3] = -500;    shoulder_right_front_angle[3] = -500;
                                    leg_left_rear_angle[3] = 0;         leg_right_rear_angle[3] = 0;         leg_left_front_angle[3] = 0;         leg_right_front_angle[3] = 0;
                                    time_duration[3] = 0.1 / FASTER;       time_space[3] = .0;

                                    shoulder_left_rear_angle[4] = 0;    shoulder_right_rear_angle[4] = -500;    shoulder_left_front_angle[4] = -500;    shoulder_right_front_angle[4] = -500;
                                    leg_left_rear_angle[4] = 0;       leg_right_rear_angle[4] = 0;         leg_left_front_angle[4] = 0;         leg_right_front_angle[4] = 0;
                                    time_duration[4] = 0.1 / FASTER;       time_space[4] = .0;
                                    shoulder_left_rear_angle[5] = 0;  shoulder_right_rear_angle[5] = -500;    shoulder_left_front_angle[5] = 0;    shoulder_right_front_angle[5] = -500;
                                    leg_left_rear_angle[5] = 0;       leg_right_rear_angle[5] = 0;         leg_left_front_angle[5] = -256;         leg_right_front_angle[5] = 0;
                                    time_duration[5] = 0.2 / FASTER;       time_space[5] = .0;
                                    shoulder_left_rear_angle[6] = 0;  shoulder_right_rear_angle[6] = -500;    shoulder_left_front_angle[6] = 0;    shoulder_right_front_angle[6] = -500;
                                    leg_left_rear_angle[6] = 0;         leg_right_rear_angle[6] = 0;         leg_left_front_angle[6] = -512;         leg_right_front_angle[6] = 0;
                                    time_duration[6] = 0.1 / FASTER;       time_space[6] = .0;

                                    shoulder_left_rear_angle[7] = 0;    shoulder_right_rear_angle[7] = -500;    shoulder_left_front_angle[7] = 0;    shoulder_right_front_angle[7] = -500;
                                    leg_left_rear_angle[7] = 0;       leg_right_rear_angle[7] = 0;         leg_left_front_angle[7] = 0;         leg_right_front_angle[7] = 0;
                                    time_duration[7] = 0.1 / FASTER;       time_space[7] = .0;
                                    shoulder_left_rear_angle[8] = 0;  shoulder_right_rear_angle[8] = -500;    shoulder_left_front_angle[8] = 0;    shoulder_right_front_angle[8] = 0;
                                    leg_left_rear_angle[8] = 0;       leg_right_rear_angle[8] = 0;         leg_left_front_angle[8] = 0;         leg_right_front_angle[8] = 256;
                                    time_duration[8] = 0.2 / FASTER;       time_space[8] = .0;
                                    shoulder_left_rear_angle[9] = 0;  shoulder_right_rear_angle[9] = -500;    shoulder_left_front_angle[9] = 0;    shoulder_right_front_angle[9] = 0;
                                    leg_left_rear_angle[9] = 0;         leg_right_rear_angle[9] = 0;         leg_left_front_angle[9] = 0;         leg_right_front_angle[9] = 512;
                                    time_duration[9] = 0.1 / FASTER;       time_space[9] = .0;

                                    shoulder_left_rear_angle[10] = 0;    shoulder_right_rear_angle[10] = -500;    shoulder_left_front_angle[10] = 0;    shoulder_right_front_angle[10] = 0;
                                    leg_left_rear_angle[10] = 0;       leg_right_rear_angle[10] = -256;         leg_left_front_angle[10] = 0;         leg_right_front_angle[10] = 0;
                                    time_duration[10] = 0.1 / FASTER;       time_space[10] = .0;
                                    shoulder_left_rear_angle[11] = 0;  shoulder_right_rear_angle[11] = 0;    shoulder_left_front_angle[11] = 0;    shoulder_right_front_angle[11] = 0;
                                    leg_left_rear_angle[11] = 0;       leg_right_rear_angle[11] = -512;         leg_left_front_angle[11] = 0;         leg_right_front_angle[11] = 0;
                                    time_duration[11] = 0.2 / FASTER;       time_space[11] = .0;
                                    shoulder_left_rear_angle[12] = 0;  shoulder_right_rear_angle[12] = 0;    shoulder_left_front_angle[12] = 0;    shoulder_right_front_angle[12] = 0;
                                    leg_left_rear_angle[12] = 0;         leg_right_rear_angle[12] = 0;         leg_left_front_angle[12] = 0;         leg_right_front_angle[12] = 0;
                                    time_duration[12] = 0.1 / FASTER;       time_space[12] = .0;

                                    mobile_mode = NONE;
                                    pres_mobile_mode = mobile_mode;

                                    break;

      default:              motion_all_num = 1;
                            shoulder_left_rear_angle[0] = 0; shoulder_right_rear_angle[0] = 0; shoulder_left_front_angle[0] = 0; shoulder_right_front_angle[0] = 0;
                            leg_left_rear_angle[0] = 0; leg_right_rear_angle[0] = 0; leg_left_front_angle[0] = 0; leg_right_front_angle[0] = 0;
                            time_duration[0] = 1.0; time_space[0] = 1.0;
                            break;
      }
    }

    int showMode()
    {
      return (int)mobile_mode;
    }

    int* setJointAngle(int motion_num)
    {
      if (leg_left_rear_angle[motion_num] < 0)
      {
        leg_left_rear_angle[motion_num] = 0;
      }
      else if (leg_left_rear_angle[motion_num] > 512)
      {
        leg_left_rear_angle[motion_num] = 512;
      }

      if (leg_right_rear_angle[motion_num] < -512)
      {
        leg_right_rear_angle[motion_num] = -512;
      }
      else if (leg_right_rear_angle[motion_num] > 0)
      {
        leg_right_rear_angle[motion_num] = 0;
      }

      if (leg_left_front_angle[motion_num] < -512)
      {
        leg_left_front_angle[motion_num] = -512;
      }
      else if (leg_left_front_angle[motion_num] > 0)
      {
        leg_left_front_angle[motion_num] = 0;
      }

      if (leg_right_front_angle[motion_num] < 0)
      {
        leg_right_front_angle[motion_num] = 0;
      }
      else if (leg_right_front_angle[motion_num] > 512)
      {
        leg_right_front_angle[motion_num] = 512;
      }

      if (shoulder_left_rear_angle[motion_num] < -1024)
      {
        shoulder_left_rear_angle[motion_num] = -1024;
      }
      else if (shoulder_left_rear_angle[motion_num] > 1024)
      {
        shoulder_left_rear_angle[motion_num] = 1024;
      }

      if (shoulder_right_rear_angle[motion_num] < -1024)
      {
        shoulder_right_rear_angle[motion_num] = -1024;
      }
      else if (shoulder_right_rear_angle[motion_num] > 1024)
      {
        shoulder_right_rear_angle[motion_num] = 1024;
      }

      if (shoulder_left_front_angle[motion_num] < -1024)
      {
        shoulder_left_front_angle[motion_num] = -1024;
      }
      else if (shoulder_left_front_angle[motion_num] > 768)
      {
        shoulder_left_front_angle[motion_num] = 768;
      }

      if (shoulder_right_front_angle[motion_num] < -768)
      {
        shoulder_right_front_angle[motion_num] = -768;
      }
      else if (shoulder_right_front_angle[motion_num] > 1024)
      {
        shoulder_right_front_angle[motion_num] = 1024;
      }

      joint_angle[0] = leg_left_rear_angle[motion_num] + 1024;
      joint_angle[1] = leg_right_rear_angle[motion_num] + 3072;
      joint_angle[2] = leg_left_front_angle[motion_num] + 3072;
      joint_angle[3] = leg_right_front_angle[motion_num] + 1024;
      joint_angle[4] = shoulder_left_rear_angle[motion_num] + 2048;
      joint_angle[5] = shoulder_right_rear_angle[motion_num] + 2048;
      joint_angle[6] = shoulder_left_front_angle[motion_num] + 2048;
      joint_angle[7] = shoulder_right_front_angle[motion_num] + 2048;

      return joint_angle;
    }
};

#endif // TURTLEBOT3_REALTURTLEBOT_MOTION_H_
