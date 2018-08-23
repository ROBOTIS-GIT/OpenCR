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

/* Authors: Hye-Jong KIM */

#ifndef MOTION_H_
#define MOTION_H_

#include "OMLink.h"

//////////////////Motion num////////////////
#define MAX_MOTION_NUM 32
////////////////////////////////////////////

//////////////////////flug///////////////////
bool motion    = false;
bool repeat    = false;
////////////////////////////////////////////

///////////////////storage//////////////////
float motion_storage[MAX_MOTION_NUM][5] = {0.0, };
const float initial_motion_set[MAX_MOTION_NUM][5] = { // time, joint1, joint2, joint3, grip
////////////////////////////////////STEP1///////////////////////////////////////////
                                                { 2.0,  1.04, -1.99, -2.87,  0.0},    //move
                                                { 1.0,  1.04, -1.74, -2.39,  0.0},    //down
                                                { 0.5,  0.00,  0.00,  0.00,  1.0},    //pick
                                                { 1.0,  1.04, -1.99, -2.87,  0.0},    //up
                                                { 2.0,  0.20, -2.04, -3.10,  0.0},    //move
                                                { 1.0,  0.20, -1.55, -2.13,  0.0},    //down
                                                { 2.0,  0.00,  0.00,  0.00, -1.0},    //place
                                                { 1.0,  0.20, -2.04, -3.10,  0.0},    //up
/////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////STEP2///////////////////////////////////////////
                                                { 2.0,  1.04, -1.99, -2.87,  0.0},    //move
                                                { 1.0,  1.04, -1.74, -2.39,  0.0},    //down
                                                { 0.5,  0.00,  0.00,  0.00,  1.0},    //pick
                                                { 1.0,  1.04, -1.99, -2.87,  0.0},    //up
                                                { 2.0, -0.20, -2.04, -3.10,  0.0},    //move
                                                { 1.0, -0.20, -1.55, -2.13,  0.0},    //down
                                                { 2.0,  0.00,  0.00,  0.00, -1.0},    //place
                                                { 1.0, -0.20, -2.04, -3.10,  0.0},    //up
/////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////STEP3///////////////////////////////////////////
                                                { 2.0,  1.04, -1.99, -2.87,  0.0},    //move
                                                { 1.0,  1.03, -1.69, -2.31,  0.0},    //down
                                                { 0.5,  0.00,  0.00,  0.00,  1.0},    //pick
                                                { 1.0,  1.04, -1.99, -2.87,  0.0},    //up
                                                { 2.0,  0.15, -1.71, -2.79,  0.0},    //move
                                                { 1.0,  0.14, -1.34, -2.18,  0.0},    //down
                                                { 2.0,  0.00,  0.00,  0.00, -1.0},    //place
                                                { 1.0,  0.15, -1.71, -2.79,  0.0},    //up
/////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////STEP4///////////////////////////////////////////
                                                { 2.0,  1.04, -1.99, -2.87,  0.0},    //move
                                                { 1.0,  1.03, -1.62, -2.22,  0.0},    //down
                                                { 0.5,  0.00,  0.00,  0.00,  1.0},    //pick
                                                { 1.0,  1.04, -1.99, -2.87,  0.0},    //up
                                                { 2.0, -0.15, -1.71, -2.80,  0.0},    //move
                                                { 1.0, -0.15, -1.34, -2.19,  0.0},    //down
                                                { 2.0,  0.00,  0.00,  0.00, -1.0},    //place
                                                { 1.0, -0.15, -1.71, -2.80,  0.0}     //up
/////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////STEP5///////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
                                                };
////////////////////////////////////////////

//////////////////motion cnt////////////////
uint8_t motion_cnt = 0;
uint8_t filled_motion_num = 0;
////////////////////////////////////////////

void setMotion()
  {
    if (motion)
    {
      if (omlink.moving())///////////////////////////
      {
        return;
      }
      else
      {
        //repeat check
        if (motion_cnt >= filled_motion_num)
        {
          if (repeat)
          {
            motion_cnt = 0;
          }
          else
          {
            motion_cnt = 0;
            motion     = false;     
          }
        }

        //motion make
        static std::vector <float> target_angle;

        if (motion_storage[motion_cnt][4] == 1.0)
        {
          digitalWrite(RELAY_PIN, HIGH);      //suction on
          omlink.jointMove(target_angle, motion_storage[motion_cnt][0]);         
          motion_cnt++;
        }
        else if (motion_storage[motion_cnt][4] == -1.0)
        {
          digitalWrite(RELAY_PIN, LOW);      //suction off
          omlink.jointMove(target_angle, motion_storage[motion_cnt][0]);   
          motion_cnt++;
        }
        else
        {
          target_angle.clear();
          for (int8_t i = 1; i < 4; i++)
          {
            target_angle.push_back(motion_storage[motion_cnt][i]);
          }
          omlink.jointMove(target_angle, motion_storage[motion_cnt][0]);   
          motion_cnt++;
        }
      }
    }
    else
    {
      motion_cnt = 0;
    }
  }

void motionStart()
{
  for (uint8_t i = 0; i < MAX_MOTION_NUM; i++)
  {
    for (uint8_t j = 0; j < 5; j++)
    {
      motion_storage[i][j] = initial_motion_set[i][j];
    }
  }
  filled_motion_num = MAX_MOTION_NUM;  
  motion_cnt = 0;          
  motion = true;
  repeat = true;
}

void motionStop()
{
  for (uint8_t i = 0; i < MAX_MOTION_NUM; i++)
  {
    for (uint8_t j = 0; j < 5; j++)
    {
      motion_storage[i][j] = 0.0;
    }
  }
  motion_cnt = 0;
  motion     = false;
  repeat     = false;
}


void AddMotionStorage()
{

}

#endif
