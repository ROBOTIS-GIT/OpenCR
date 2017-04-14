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

#include "turtlebot3_segway.h"

/*******************************************************************************
* Declaration for Hardware Timer (Interrupt control)
*******************************************************************************/
HardwareTimer Timer(TIMER_CH1);

/*******************************************************************************
* Declaration for IMU
*******************************************************************************/
cIMU IMU;

/*******************************************************************************
* Declaration for Filter
*******************************************************************************/
float filterFrequency = 1.0;    // Hz
FilterOnePole lowpassFilter(LOWPASS, filterFrequency);

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;

float angle[3] = {0.0,0.0,0.0}; //roll, pitch, yaw

// initial angle offset
float angle_offset = 0.0;

// keyboard input
char keyboard;

// Set PID gain
float p_gain = 4000.0;
float i_gain = 2.0;
float d_gain = 78.0;

float control_output = 0.0;

void setup()
{
  // Setting for Dynamixel motors
  motor_driver.init();

  // Initialization IMU
  imuInit();
}

void loop()
{
  if (IMU.update() > 0)
    getAngle(angle);

#ifdef DEBUG
  if (Serial.available())
  {
    keyboard = Serial.read();
    if (keyboard == 'u')
    {
      p_gain = p_gain + 1.0;
    }
    else if (keyboard == 'j')
    {
      p_gain = p_gain - 1.0;
    }
    else if (keyboard == 'o')
    {
      d_gain = d_gain + 0.1;
    }
    else if (keyboard == 'l')
    {
      d_gain = d_gain - 0.1;
    }
    else if (keyboard == 'i')
    {
      i_gain = i_gain + 0.1;
    }
    else if (keyboard == 'k')
    {
      i_gain = i_gain - 0.1;
    }
    else if (keyboard == 'a')
    {
      angle_offset = angle_offset + 0.01;
    }
    else if (keyboard == 's')
    {
      angle_offset = angle_offset - 0.01;
    }
  }

  Serial.print(" P : ");
  Serial.print(p_gain);
  Serial.print(" I : ");
  Serial.print(i_gain);
  Serial.print(" D : ");
  Serial.print(d_gain);
  Serial.print(" offset : ");
  Serial.print(angle_offset);
  Serial.print(" output : ");
  Serial.print(control_output);
  Serial.print(" angle : ");
  Serial.println(angle[0]);
#endif
}

/*******************************************************************************
* Get angle from IMU
*******************************************************************************/
void getAngle(float angle[3])
{
  float roll, pitch, yaw;

  roll  = IMU.rpy[0];
  pitch = IMU.rpy[1];
  yaw   = IMU.rpy[2];

  angle[0] = lowpassFilter.input(pitch) + angle_offset;
  angle[1] = pitch + angle_offset;
}

void startDynamixelControlInterrupt()
{
  Timer.pause();
  Timer.setPeriod(CONTOL_PERIOD);           // in microseconds
  Timer.attachInterrupt(controlSegway);
  Timer.refresh();
  Timer.resume();
}

/*******************************************************************************
* Initialization of IMU
*******************************************************************************/
void imuInit()
{
  IMU.begin();

  IMU.SEN.acc_cali_start();
  while( IMU.SEN.acc_cali_get_done() == false )
  {
    IMU.update();
  }

  // Start Dynamixel Control Interrupt
  startDynamixelControlInterrupt();
}

/*******************************************************************************
* Control segway PWM
*******************************************************************************/
void controlSegway(void)
{
  bool dxl_comm_result = false;
  static float control_input = 0.0;

  static float cur_error = 0.0, pre_error = 0.0, integral = 0.0, derivative = 0.0;
  static float diff_time = 0.007;

  static int16_t cnt = 0;           // timer counter

  cur_error  = control_input - angle[0];
  integral   = integral + (cur_error * diff_time);
  derivative = (cur_error - pre_error) / diff_time;

  if (cnt > 500)
  {
    integral = 0.0;
    cnt = 0;
  }
  else
  {
    cnt++;
  }

  control_output = p_gain * cur_error +
                   i_gain * integral  +
                   d_gain * derivative;

  if (control_output >= PWM_LIMIT)
  {
    control_output = PWM_LIMIT;
  }
  else if (control_output <= (-1) * PWM_LIMIT)
  {
    control_output = (-1) * PWM_LIMIT;
  }
  pre_error = cur_error;

  dxl_comm_result = motor_driver.controlMotor((int64_t)control_output, (int64_t)control_output);
  if (dxl_comm_result == false)
    return;
}
