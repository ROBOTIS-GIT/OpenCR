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

#include "../../include/turtlebot3/turtlebot3_sensor.h"

Turtlebot3Sensor::Turtlebot3Sensor()
{
}

Turtlebot3Sensor::~Turtlebot3Sensor()
{}

bool Turtlebot3Sensor::init(void)
{
  bool ret = false;

  initBumper();
  initIR();
  initSonar();
  initLED();

  uint8_t get_error_code = imu_.begin();

  if (get_error_code == 0x00)
    ret = true;

  return ret;
}

void Turtlebot3Sensor::initIMU(void)
{
  imu_.begin();
}

void Turtlebot3Sensor::updateIMU(void)
{
  imu_.update();
}

void Turtlebot3Sensor::calibrationGyro()
{
  uint32_t pre_time;
  uint32_t t_time;

  const uint8_t led_ros_connect = 3;

  imu_.SEN.gyro_cali_start();
  
  t_time   = millis();
  pre_time = millis();

  while(!imu_.SEN.gyro_cali_get_done())
  {
    imu_.update();

    if (millis()-pre_time > 5000)
    {
      break;
    }
    if (millis()-t_time > 100)
    {
      t_time = millis();
      setLedToggle(led_ros_connect);
    }
  }
}

float* Turtlebot3Sensor::getImuAngularVelocity(void)
{
  static float angular_vel[3];

  angular_vel[0] = imu_.SEN.gyroADC[0] * GYRO_FACTOR;
  angular_vel[1] = imu_.SEN.gyroADC[1] * GYRO_FACTOR;
  angular_vel[2] = imu_.SEN.gyroADC[2] * GYRO_FACTOR;

  return angular_vel;
}

float* Turtlebot3Sensor::getImuLinearAcc(void)
{
  static float linear_acc[3];

  linear_acc[0] = imu_.SEN.accADC[0] * ACCEL_FACTOR;
  linear_acc[1] = imu_.SEN.accADC[1] * ACCEL_FACTOR;
  linear_acc[2] = imu_.SEN.accADC[2] * ACCEL_FACTOR;

  return linear_acc;
}

float* Turtlebot3Sensor::getImuMagnetic(void)
{
  static float magnetic[3];

  magnetic[0] = imu_.SEN.magADC[0] * MAG_FACTOR;
  magnetic[1] = imu_.SEN.magADC[1] * MAG_FACTOR;
  magnetic[2] = imu_.SEN.magADC[2] * MAG_FACTOR;

  return magnetic;
}

float* Turtlebot3Sensor::getOrientation(void)
{
  static float orientation[4];

  orientation[0] = imu_.quat[0];
  orientation[1] = imu_.quat[1];
  orientation[2] = imu_.quat[2];
  orientation[3] = imu_.quat[3];

  return orientation;
}

float Turtlebot3Sensor::checkVoltage(void)
{
  float vol_value;
  
  vol_value = getPowerInVoltage();

  return vol_value;
}

uint8_t Turtlebot3Sensor::checkPushButton(void)
{
  return getPushButton();
}

void Turtlebot3Sensor::onMelody()
{
  static uint8_t pre_note_number = 7, current_note_number;
  static uint32_t pre_time;
  static uint32_t note_duration, pause_time_ms_between_notes;

  if(is_melody_play_complete_ == false){
    if(pre_note_number != current_note_number){
      // to calculate the note duration, take one second
      // divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      note_duration = 1000/melody_duration_[current_note_number];    
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      pause_time_ms_between_notes = note_duration * 1.30;

      tone(BDPIN_BUZZER, melody_note_[current_note_number], note_duration);
      
      pre_note_number = current_note_number;
      pre_time = millis();
    }else{
      if(millis()-pre_time >= pause_time_ms_between_notes){
        pre_time = millis();
        noTone(BDPIN_BUZZER);
        current_note_number++;

        if(current_note_number == 8){
          is_melody_play_complete_ = true;
          current_note_number = 0;
        }
      }
    }
  }
}

void Turtlebot3Sensor::makeMelody(uint8_t index)
{
  const uint16_t NOTE_C4 = 262;
  const uint16_t NOTE_D4 = 294;
  const uint16_t NOTE_E4 = 330;
  const uint16_t NOTE_F4 = 349;
  const uint16_t NOTE_G4 = 392;
  const uint16_t NOTE_A4 = 440;
  const uint16_t NOTE_B4 = 494;
  const uint16_t NOTE_C5 = 523;
  //const uint16_t NOTE_C6 = 1047;

  const uint8_t OFF         = 0;
  const uint8_t ON          = 1;
  const uint8_t LOW_BATTERY = 2;
  const uint8_t ERROR       = 3;
  const uint8_t BUTTON1     = 4;
  const uint8_t BUTTON2     = 5;

  switch (index)
  {
    case ON:
      melody_note_[0] = NOTE_C4;   melody_duration_[0] = 4;
      melody_note_[1] = NOTE_D4;   melody_duration_[1] = 4;
      melody_note_[2] = NOTE_E4;   melody_duration_[2] = 4;
      melody_note_[3] = NOTE_F4;   melody_duration_[3] = 4;
      melody_note_[4] = NOTE_G4;   melody_duration_[4] = 4;
      melody_note_[5] = NOTE_A4;   melody_duration_[5] = 4;
      melody_note_[6] = NOTE_B4;   melody_duration_[6] = 4;
      melody_note_[7] = NOTE_C5;   melody_duration_[7] = 4;
     break;

    case OFF:
      melody_note_[0] = NOTE_C5;   melody_duration_[0] = 4;
      melody_note_[1] = NOTE_B4;   melody_duration_[1] = 4;
      melody_note_[2] = NOTE_A4;   melody_duration_[2] = 4;
      melody_note_[3] = NOTE_G4;   melody_duration_[3] = 4;
      melody_note_[4] = NOTE_F4;   melody_duration_[4] = 4;
      melody_note_[5] = NOTE_E4;   melody_duration_[5] = 4;
      melody_note_[6] = NOTE_D4;   melody_duration_[6] = 4;
      melody_note_[7] = NOTE_C4;   melody_duration_[7] = 4;  
     break;

    case LOW_BATTERY:
      melody_note_[0] = 1000;      melody_duration_[0] = 1;
      melody_note_[1] = 1000;      melody_duration_[1] = 1;
      melody_note_[2] = 1000;      melody_duration_[2] = 1;
      melody_note_[3] = 1000;      melody_duration_[3] = 1;
      melody_note_[4] = 0;         melody_duration_[4] = 8;
      melody_note_[5] = 0;         melody_duration_[5] = 8;
      melody_note_[6] = 0;         melody_duration_[6] = 8;
      melody_note_[7] = 0;         melody_duration_[7] = 8;
     break;

    case ERROR:
      melody_note_[0] = 1000;      melody_duration_[0] = 3;
      melody_note_[1] = 500;       melody_duration_[1] = 3;
      melody_note_[2] = 1000;      melody_duration_[2] = 3;
      melody_note_[3] = 500;       melody_duration_[3] = 3;
      melody_note_[4] = 1000;      melody_duration_[4] = 3;
      melody_note_[5] = 500;       melody_duration_[5] = 3;
      melody_note_[6] = 1000;      melody_duration_[6] = 3;
      melody_note_[7] = 500;       melody_duration_[7] = 3;
     break;

    case BUTTON1:
    case BUTTON2:
    default:
      return;
  }

  is_melody_play_complete_ = false;
}

void Turtlebot3Sensor::initBumper(void)
{
  ollo_.begin(3, TOUCH_SENSOR);
  ollo_.begin(4, TOUCH_SENSOR);
}

uint8_t Turtlebot3Sensor::checkPushBumper(void)
{
  uint8_t push_state = 0;

  if      (ollo_.read(3, TOUCH_SENSOR) == HIGH) push_state = 2;
  else if (ollo_.read(4, TOUCH_SENSOR) == HIGH) push_state = 1;
  else    push_state = 0;
  
  return push_state;
}

bool Turtlebot3Sensor::getBumper1State()
{
  return ollo_.read(3, TOUCH_SENSOR); 
}

bool Turtlebot3Sensor::getBumper2State()
{
  return ollo_.read(4, TOUCH_SENSOR);
}

void Turtlebot3Sensor::initIR(void)
{
  ollo_.begin(2, IR_SENSOR);
}

float Turtlebot3Sensor::getIRsensorData(void)
{
  float ir_data = ollo_.read(2, IR_SENSOR);
  
  return ir_data;
}

void Turtlebot3Sensor::initSonar(void)
{
  sonar_pin_.trig = BDPIN_GPIO_1;
  sonar_pin_.echo = BDPIN_GPIO_2;

  pinMode(sonar_pin_.trig, OUTPUT);
  pinMode(sonar_pin_.echo, INPUT);
}

void Turtlebot3Sensor::updateSonar(uint32_t t)
{
  static uint32_t t_time = 0;
  static bool make_pulse = TRUE;
  static bool get_duration = FALSE;

  float distance = 0.0, duration = 0.0;

  if (make_pulse == TRUE)
  {
    digitalWrite(sonar_pin_.trig, HIGH);

    if (t - t_time >= 10)
    {
      digitalWrite(sonar_pin_.trig, LOW);

      get_duration = TRUE;
      make_pulse = FALSE;

      t_time = t;
    }
  }

  if (get_duration == TRUE)
  {
    duration = pulseIn(sonar_pin_.echo, HIGH);
    distance = ((float)(340 * duration) / 10000) / 2;

    make_pulse = TRUE;
    get_duration = FALSE;
  }

  sonar_data_ = distance;
}

float Turtlebot3Sensor::getSonarData(void)
{
  float distance = 0.0;

  distance = sonar_data_;

  return distance;
}

float Turtlebot3Sensor::getIlluminationData(void)
{
  uint16_t light;

  light = analogRead(A1);

  return light;
}

void Turtlebot3Sensor::initLED(void)
{
  led_pin_array_.front_left  = BDPIN_GPIO_4;
  led_pin_array_.front_right = BDPIN_GPIO_6;
  led_pin_array_.back_left   = BDPIN_GPIO_8;
  led_pin_array_.back_right  = BDPIN_GPIO_10;
 
  pinMode(led_pin_array_.front_left, OUTPUT);
  pinMode(led_pin_array_.front_right, OUTPUT);
  pinMode(led_pin_array_.back_left, OUTPUT);
  pinMode(led_pin_array_.back_right, OUTPUT);
}

void Turtlebot3Sensor::setLedPattern(double linear_vel, double angular_vel)
{
  if (linear_vel > 0.0 && angular_vel == 0.0)     // front
  {
    digitalWrite(led_pin_array_.front_left, HIGH);
    digitalWrite(led_pin_array_.front_right, HIGH);
    digitalWrite(led_pin_array_.back_left, LOW);
    digitalWrite(led_pin_array_.back_right, LOW);
  }
  else if (linear_vel >= 0.0 && angular_vel > 0.0)  // front left
  {
    digitalWrite(led_pin_array_.front_left, HIGH);
    digitalWrite(led_pin_array_.front_right, LOW);
    digitalWrite(led_pin_array_.back_left, LOW);
    digitalWrite(led_pin_array_.back_right, LOW);
  }
  else if (linear_vel >= 0.0 && angular_vel < 0.0)  // front right
  {
    digitalWrite(led_pin_array_.front_left, LOW);
    digitalWrite(led_pin_array_.front_right, HIGH);
    digitalWrite(led_pin_array_.back_left, LOW);
    digitalWrite(led_pin_array_.back_right, LOW);
  }
  else if (linear_vel < 0.0 && angular_vel == 0.0) // back
  {
    digitalWrite(led_pin_array_.front_left, LOW);
    digitalWrite(led_pin_array_.front_right, LOW);
    digitalWrite(led_pin_array_.back_left, HIGH);
    digitalWrite(led_pin_array_.back_right, HIGH);
  }
  else if (linear_vel <= 0.0 && angular_vel > 0.0)  // back right
  {
    digitalWrite(led_pin_array_.front_left, LOW);
    digitalWrite(led_pin_array_.front_right, LOW);
    digitalWrite(led_pin_array_.back_left, LOW);
    digitalWrite(led_pin_array_.back_right, HIGH);
  }
  else if (linear_vel <= 0.0 && angular_vel < 0.0)  // back left
  {
    digitalWrite(led_pin_array_.front_left, LOW);
    digitalWrite(led_pin_array_.front_right, LOW);
    digitalWrite(led_pin_array_.back_left, HIGH);
    digitalWrite(led_pin_array_.back_right, LOW);
  }
  else 
  {
    digitalWrite(led_pin_array_.front_left, LOW);
    digitalWrite(led_pin_array_.front_right, LOW);
    digitalWrite(led_pin_array_.back_left, LOW);
    digitalWrite(led_pin_array_.back_right, LOW);
  }
}





