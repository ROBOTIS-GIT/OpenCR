/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef DEMO2_H_
#define DEMO2_H_

#include <stewart_libs.h>

/*****************************************************************************
** Touch panel stuff for detecting ball position
*****************************************************************************/
#define PIN_LR      8
#define PIN_LL      9
#define PIN_UR      10
#define PIN_UL      11
#define PIN_COM     A0

bool touch = 0;
double touch_position[2];

void touchPanelInit()
{
  pinMode(PIN_LR, OUTPUT);
  pinMode(PIN_LL, OUTPUT);  
  pinMode(PIN_UR, OUTPUT);
  pinMode(PIN_UL, OUTPUT);

  digitalWrite(PIN_LR, LOW);
  digitalWrite(PIN_LL, LOW);
  digitalWrite(PIN_UR, LOW);
  digitalWrite(PIN_UL, LOW);
}

int touchAvailable(void)
{
  int pin_data;
  
  digitalWrite(PIN_UL, LOW);
  digitalWrite(PIN_UR, LOW);
  digitalWrite(PIN_LL, LOW);
  digitalWrite(PIN_LR, LOW);
  
  pinMode(PIN_COM, INPUT_PULLUP);
  delay(2);  
  pin_data = digitalRead(PIN_COM);
  pinMode(PIN_COM, INPUT_ANALOG);
  
  return pin_data; 
}

int touchX(void)
{
  digitalWrite(PIN_UL, LOW);
  digitalWrite(PIN_UR, HIGH);
  digitalWrite(PIN_LL, HIGH);
  digitalWrite(PIN_LR, LOW);

  delay(2);
  return analogRead(PIN_COM); 
}

int touchY(void)
{
  digitalWrite(PIN_UL, LOW);
  digitalWrite(PIN_UR, LOW);
  digitalWrite(PIN_LL, HIGH);
  digitalWrite(PIN_LR, HIGH);

  delay(2);
  return analogRead(PIN_COM); 
}

/*****************************************************************************
** PID stuff for ball position
*****************************************************************************/
int16_t ball_x_pos, ball_y_pos;
int16_t measured_x_pos, measured_y_pos;
double corrected_x_pos, corrected_y_pos;

int16_t set_x_pos = 0, set_y_pos = 0;
int16_t x_pos_array [79], y_pos_array [79];
double x_avg [16], y_avg [16];

double FPGA_x_angle = 0, FPGA_y_angle = 0;
double testxangle = 0, testyangle = 0;

double p_gain = 0.28, i_gain = 0.01, d_gain = 1.5;//550

double x_offset = -10, y_offset = 0;
double error_x = 0, error_y = 0;

double i_x = 0, i_y = 0;

double d_x = 0, d_y = 0;

double d_x_avg = 0, d_y_avg = 0;
double d_x_pos = 0, d_y_pos = 0;
double d_x_neg = 0, d_y_neg = 0;

static int count_for_round = 0;

int i, j;

void ball_PID(Stewart *stewart)
{
  // Raw Ball Position
  touch_position[0] = double(touchX()-520) / 520.0 * 310.0;
  touch_position[1] = double(touchY()-520) / 520.0 * 230.0;

  if (touchAvailable()==1) {
    touch_position[0] = 0;
    touch_position[1] = 0;
  }

  corrected_x_pos = touch_position[0];
  corrected_y_pos = touch_position[1];

  Serial.print("T : ");
  Serial.print(touchAvailable());
    
  // Serial.print("\t X : ");
  // Serial.print(touch_position[0]);
  // Serial.print("\tY : ");
  // Serial.print(touch_position[1]);
  // Serial.println(); 


  if (stewart->getOffsetPositionNum() == 0) 
  {
    set_x_pos = 0;
    set_y_pos = 0;
    count_for_round = 0;
  }
  else if (stewart->getOffsetPositionNum() == 1) 
  {
    set_x_pos = -20;
    set_y_pos = 15;
    count_for_round = 0;
  }
  else if (stewart->getOffsetPositionNum() == 2) 
  {
    set_x_pos = -20;
    set_y_pos = -15;
    count_for_round = 0;
  }
  else if (stewart->getOffsetPositionNum() == 3) 
  {
    set_x_pos = 20;
    set_y_pos = -15;
    count_for_round = 0;
  }
  else if (stewart->getOffsetPositionNum() == 4) 
  {
    set_x_pos = 20;
    set_y_pos = 15;
    count_for_round = 0;
  }
  else if (stewart->getOffsetPositionNum() == 5) 
  {
    set_x_pos = 75*cos((double)count_for_round/10.0);
    set_y_pos = 75*sin((double)count_for_round/10.0);
    count_for_round++;
    if (count_for_round == 6280) count_for_round = 0;
  }
  else if (stewart->getOffsetPositionNum() == 6) 
  {
    set_x_pos = 75*cos((double)count_for_round/10.0);
    set_y_pos =-75*sin((double)count_for_round/10.0);
    count_for_round++;
    if (count_for_round == 6280) count_for_round = 0;
  }

  // P value
  error_x = (set_x_pos - corrected_x_pos) ;
  error_y = (set_y_pos - corrected_y_pos) ;

  // D value
  //fill ball history array
  for (i = 0; i < 79; i ++)
  {
    x_pos_array [i + 1] = x_pos_array [i];
    y_pos_array [i + 1] = y_pos_array [i];
  }
  x_pos_array [0] = corrected_x_pos;
  y_pos_array [0] = corrected_y_pos;

  for (i = 0; i < 16; i ++)
  {
    x_avg[i] = x_pos_array [i * 5]; //5
    y_avg[i] = y_pos_array [i * 5]; //5
  }

  d_x_pos = 322.0 * x_avg[0] + 217.0 * x_avg[1] + 110.0 * x_avg[2] + 35.0 * x_avg[3] + 25.0 * x_avg[13] + 98.0 * x_avg[14] + 203.0 * x_avg[15]; //
  d_y_pos = 322.0 * y_avg[0] + 217.0 * y_avg[1] + 110.0 * y_avg[2] + 35.0 * y_avg[3] + 25.0 * y_avg[13] + 98.0 * y_avg[14] + 203.0 * y_avg[15];

  d_x_neg = -42.0 * x_avg[4] - 87.0 * x_avg[5] - 134.0 * x_avg[6] - 149.0 * x_avg[7] - 166.0 * x_avg[8] - 151.0 * x_avg[9] - 138.0 * x_avg[10] - 93.0 * x_avg[11] - 50.0 * x_avg[12];
  d_y_neg = -42.0 * y_avg[4] - 87.0 * y_avg[5] - 134.0 * y_avg[6] - 149.0 * y_avg[7] - 166.0 * y_avg[8] - 151.0 * y_avg[9] - 138.0 * y_avg[10] - 93.0 * y_avg[11] - 50.0 * y_avg[12];

  d_x = (d_x_pos + d_x_neg) * d_gain / 28.56;
  d_y = (d_y_pos + d_y_neg) * d_gain / 28.56;

  d_x_avg = .5 * d_x + .5 * d_x_avg;
  d_y_avg = .5 * d_y + .5 * d_y_avg;

  // I value
  if (error_x < 200 && error_x > -200) i_x += i_gain * error_x / 10.0;
  else i_x = 0;
  if (error_y < 200 && error_y > -200) i_y += i_gain * error_y / 10.0;
  else i_y = 0;

  if (i_gain == 0)
  {
    i_x = 0;
    i_y = 0;
  }

  if (i_x > 400.0) i_x = 400.0;
  else if (i_x < -400.0) i_x = -400.0;

  if (i_y > 400.0) i_y = 400.0;
  else if (i_y < -400.0) i_y = -400.0;


  FPGA_x_angle = error_x * p_gain + i_x - d_x_avg + x_offset;
  FPGA_y_angle = error_y * p_gain + i_y - d_y_avg + y_offset;
  testxangle = error_x * p_gain;
  testyangle = error_y * p_gain;

  Serial.print("\t OffsetNum : ");
  Serial.print(stewart->getOffsetPositionNum());
  Serial.print("\t set_X : ");
  Serial.print(set_x_pos,2);
  Serial.print("\t set_Y : ");
  Serial.print(set_y_pos,2);
  Serial.print("\t curr_x : ");
  Serial.print(corrected_x_pos,2);
  Serial.print("\t curr_Y : ");
  Serial.print(corrected_y_pos,2);
  Serial.println(); 
  Serial.print("\t Ent_X : ");
  Serial.print(FPGA_x_angle,2);
  Serial.print("\t Ent_Y : ");
  Serial.print(FPGA_y_angle,2);
  Serial.print("\t Prop_X : ");
  Serial.print(testxangle,2);
  Serial.print("\t Prop_Y : ");
  Serial.print(testyangle,2);
  Serial.print("\t Dete_X : ");
  Serial.print(d_x / d_gain,2);
  Serial.print("\t Dete_Y : ");
  Serial.print(d_y / d_gain,2);
  Serial.println(); 
}

/*****************************************************************************
** For calculating Median and Average
*****************************************************************************/
std::vector<double> sample_angle_x_for_median;
std::vector<double> sample_angle_y_for_median;
std::vector<double> sample_angle_1_for_average;
std::vector<double> sample_angle_2_for_average;
std::vector<double> sample_angle_3_for_average;
std::vector<double> sample_angle_4_for_average;
std::vector<double> sample_angle_5_for_average;
std::vector<double> sample_angle_6_for_average;

double calcMedian(std::vector<double> *sample_angle_vector, double new_sample) 
{
  std::vector<double> _sample_angle_vector;
  double result;

  // Add new sample
  if (sample_angle_vector->size() < 10)
  {
    sample_angle_vector->push_back(new_sample);
  }
  else 
  {
    for (int8_t i=0; i<9; i++)
    {
      sample_angle_vector->at(i) = sample_angle_vector->at(i+1);
    }
    sample_angle_vector->at(9) = new_sample;
  }

  
  _sample_angle_vector = *sample_angle_vector;

  // Sort values
  std::sort(_sample_angle_vector.begin(),_sample_angle_vector.end());

  // Compute Median 
  if(_sample_angle_vector.size()%2==1) //Number of elements are odd
  {
    result = _sample_angle_vector[_sample_angle_vector.size()/2];
  }
  else // Number of elements are even
  {
    int index = _sample_angle_vector.size()/2;
    result = (_sample_angle_vector[index-1] + _sample_angle_vector[index])/2;
  }

  return result;
}

double calcAverage(std::vector<double> *sample_angle_vector, double new_sample) 
{
  std::vector<double> _sample_angle_vector;
  double result;

  // Add new sample
  if (sample_angle_vector->size() < 10) 
  {
    sample_angle_vector->push_back(new_sample);
  }
  else 
  {
    for (int8_t i=0; i<9; i++)
    {
      sample_angle_vector->at(i) = sample_angle_vector->at(i+1);
    }
    sample_angle_vector->at(9) = new_sample;
  }


  _sample_angle_vector = *sample_angle_vector;

  // Compute Average
  double sum;
  for (int8_t i=0; i<_sample_angle_vector.size(); i++)
  {
    sum += _sample_angle_vector.at(i);
  }

  result = sum / _sample_angle_vector.size();

  return result;
}

/*****************************************************************************
** Start or Stop Demo
*****************************************************************************/
bool start_demo_flag;

void startDemo()
{
  // Start the demo
  start_demo_flag = true;
}

void stopDemo(Stewart *stewart)
{
  // Stop the demo
  start_demo_flag = false;

  // Move to the default pose.
  stewart->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, 0.0), 2.0);
}

/*****************************************************************************
** Initialize Demo
*****************************************************************************/
void initDemo()
{
  touchPanelInit();
}

/*****************************************************************************
** Run Demo
*****************************************************************************/
void runDemo(Stewart *stewart)
{
  if(digitalRead(BDPIN_PUSH_SW_1))
  {
    startDemo();
  }
  if(digitalRead(BDPIN_PUSH_SW_2))
  {
    stopDemo(stewart);    
  }

  if (stewart->getMovingState()) 
  {
    return;
  }
  else
  {
    if (start_demo_flag)
    {
      ball_PID(stewart);

      // Compute X, Y Position
      double x_input;
      double y_input;

      x_input = calcMedian(&sample_angle_x_for_median, double(FPGA_x_angle)) /10*DEG2RAD;
      y_input = calcMedian(&sample_angle_y_for_median, double(FPGA_y_angle)) /10*DEG2RAD;

      if (x_input >  10*DEG2RAD) x_input =  10*DEG2RAD;
      if (x_input < -10*DEG2RAD) x_input = -10*DEG2RAD;
      if (y_input >  10*DEG2RAD) y_input =  10*DEG2RAD;
      if (y_input < -10*DEG2RAD) y_input = -10*DEG2RAD;


      Eigen::Matrix4d robot_balancing;
      robot_balancing = RM_MATH::getRotation4d(x_input, y_input, 0);
      // robot_balancing = RM_MATH::getRotation4d((-touch_position[0]/10)*DEG2RAD, (-touch_position[1]/10)*DEG2RAD, 0);

      Pose goal_pose;
      goal_pose.position = RM_MATH::makeVector3(0.0, 0.0, 0.0);
      goal_pose.orientation = RM_MATH::makeMatrix3(robot_balancing(0,0), robot_balancing(0,1), robot_balancing(0,2),
                                                  robot_balancing(1,0), robot_balancing(1,1), robot_balancing(1,2),
                                                  robot_balancing(2,0), robot_balancing(2,1), robot_balancing(2,2));

      // Compute Joint Angular Position
      std::vector<double> goal_joint_values;

      stewart->inverseKinematics("tool", goal_pose, &goal_joint_values);

      std::vector<double> angle_input;

      double curr_motor_angle[6];

      //add deadbamd that the motors do nothing
      curr_motor_angle[0] = stewart->getJointValue("joint1").value;
      curr_motor_angle[1] = stewart->getJointValue("joint2").value;
      curr_motor_angle[2] = stewart->getJointValue("joint3").value;
      curr_motor_angle[3] = stewart->getJointValue("joint4").value;
      curr_motor_angle[4] = stewart->getJointValue("joint5").value;
      curr_motor_angle[5] = stewart->getJointValue("joint6").value;

      double set_speed[6];

      angle_input.push_back(calcAverage(&sample_angle_1_for_average, goal_joint_values.at(0)));
      angle_input.push_back(calcAverage(&sample_angle_2_for_average, goal_joint_values.at(1)));
      angle_input.push_back(calcAverage(&sample_angle_3_for_average, goal_joint_values.at(2)));
      angle_input.push_back(calcAverage(&sample_angle_4_for_average, goal_joint_values.at(3)));
      angle_input.push_back(calcAverage(&sample_angle_5_for_average, goal_joint_values.at(4)));
      angle_input.push_back(calcAverage(&sample_angle_6_for_average, goal_joint_values.at(5)));

      for (int8_t i=0; i<6; i++)
      {
        angle_input.push_back(goal_joint_values.at(i));
      }

      double deadband = 2*DEG2RAD;
      for (int8_t i=0; i<6; i++)
      {
        if (curr_motor_angle[i] - angle_input.at(i) < deadband && curr_motor_angle[i] - angle_input.at(i) > -deadband) 
          angle_input.at(i) = curr_motor_angle[i];
      }

      stewart->jointTrajectoryMove(angle_input, 0.05);
      // stewart->jointTrajectoryMove(angle_input, 0.1);
    }
  }
}

#endif // DEMO2_H_

