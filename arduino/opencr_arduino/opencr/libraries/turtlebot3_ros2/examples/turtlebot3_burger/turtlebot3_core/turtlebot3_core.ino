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

#include "turtlebot3_core_config.h"

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  DEBUG_SERIAL.begin(57600);

  pinMode(LED_WORKING_CHECK, OUTPUT);

  // Setting for Dynamixel motors
  motor_driver.init();
  // Setting for IMU
  sensors.init();
  // Init diagnosis
  diagnosis.init();
  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  controllers.init(MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);

  updateGyroCali(true);

  dxl_slave.setPortProtocolVersion(PROTOCOL_VERSION_DXL_SLAVE);
  dxl_slave.setID(ID_DXL_SLAVE);
  dxl_slave.setFirmwareVersion(1);

  dxl_slave.setControlTable(ADDR_MODEL_NUMBER, (uint16_t)dxl_slave.getModelNumber());
  dxl_slave.setControlTable(ADDR_MODEL_INFORM, (uint32_t)MODEL_INFO);
  dxl_slave.setControlTable(ADDR_FIRMWARE_VER, (uint8_t)dxl_slave.getFirmwareVersion());
  dxl_slave.setControlTable(ADDR_ID, (uint8_t)dxl_slave.getID());
  dxl_slave.setControlTable(ADDR_BAUDRATE, (uint8_t)3);
  
  dxl_slave.setAddrRemoteWriteProtected(0, 255, true);
  dxl_slave.setAddrRemoteWriteProtected(ADDR_SOUND, 1, false);
  dxl_slave.setAddrRemoteWriteProtected(ADDR_USER_LED_1, 4, false);
  dxl_slave.setAddrRemoteWriteProtected(ADDR_MOTOR_TORQUE, ADDR_PROFILE_ACC_R+4-ADDR_MOTOR_TORQUE, false);

  dxl_slave.setReadCallbackFunc(dxl_slave_read_callback_func);
  dxl_slave.setWriteCallbackFunc(dxl_slave_write_callback_func);

  motor_driver.setTorque(true);
  dxl_slave.setControlTable(ADDR_MOTOR_TORQUE, (uint8_t)motor_driver.getTorque());
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  static uint32_t pre_time;

  updateVariable(true);

  if ((millis()-pre_time) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    pre_time = millis();
    updateGoalVelocity();
    if(sensors.checkVoltage() >= 7.0){
      motor_driver.controlMotor(WHEEL_SEPARATION, goal_velocity);
    }
  }

  // Receive data from RC100 
  controllers.getRCdata(goal_velocity_from_rc100);

  if(sensors.checkVoltage() >= 7.0){
    // Check push button pressed for simple test drive
    driveTest(diagnosis.getButtonPress(3000));
  }

  // Update the IMU unit
  sensors.updateIMU();

  // Update sonar data
  //TODO: sensors.updateSonar(t);

  // Start Gyro Calibration after ROS connection
  updateGyroCali(true);

  // Show LED status
  diagnosis.showLedStatus(true);

  // Update Voltage
  diagnosis.updateVoltageCheck(true);

  updateGPIOs(&dxl_slave, 500);
  updateBatteryStatus(&dxl_slave, 1000);
  updateAnalogSensors(&dxl_slave, 200);
  updateIMU(&dxl_slave, 10);

  if(sensors.checkVoltage() >= 7.0){  
    updateMotorStatus(&dxl_slave, 10);
  }

  dxl_slave.processPacket();
}


/*******************************************************************************
* Turtlebot3 test drive using push buttons
*******************************************************************************/
void driveTest(uint8_t buttons)
{
  static bool move[2] = {false, false};
  static int32_t saved_tick[2] = {0, 0};
  static double diff_encoder = 0.0;

  int32_t current_tick[2] = {0, 0};

  motor_driver.readPresentPosition(current_tick[LEFT], current_tick[RIGHT]);

  if (buttons & (1<<0))  
  {
    move[LINEAR] = true;
    saved_tick[RIGHT] = current_tick[RIGHT];

    diff_encoder = TEST_DISTANCE / (0.207 / 4096); // (Circumference of Wheel) / (The number of tick per revolution)
  }
  else if (buttons & (1<<1))
  {
    move[ANGULAR] = true;
    saved_tick[RIGHT] = current_tick[RIGHT];

    diff_encoder = (TEST_RADIAN * TURNING_RADIUS) / (0.207 / 4096);
  }

  if (move[LINEAR])
  {    
    if (abs(saved_tick[RIGHT] - current_tick[RIGHT]) <= diff_encoder)
    {
      goal_velocity_from_button[LINEAR]  = 0.05;
    }
    else
    {
      goal_velocity_from_button[LINEAR]  = 0.0;
      move[LINEAR] = false;
    }
  }
  else if (move[ANGULAR])
  {   
    if (abs(saved_tick[RIGHT] - current_tick[RIGHT]) <= diff_encoder)
    {
      goal_velocity_from_button[ANGULAR]= -0.7;
    }
    else
    {
      goal_velocity_from_button[ANGULAR]  = 0.0;
      move[ANGULAR] = false;
    }
  }
}

/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;
  
  if (isConnected){
    if (variable_flag == false){      
      sensors.initIMU();
      variable_flag = true;
    }
  }else{
    variable_flag = false;
  }
}


/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;

  if (isConnected){
    if (isEnded == false){
      sensors.calibrationGyro();
      isEnded = true;
    }
  }else{
    isEnded = false;
  }
}

/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{
  goal_velocity[LINEAR]  = goal_velocity_from_button[LINEAR]  + goal_velocity_from_cmd[LINEAR]  + goal_velocity_from_rc100[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_button[ANGULAR] + goal_velocity_from_cmd[ANGULAR] + goal_velocity_from_rc100[ANGULAR];

  sensors.setLedPattern(goal_velocity[LINEAR], goal_velocity[ANGULAR]);
}


float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void updateGPIOs(DYNAMIXEL::Slave *slave, uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    slave->setControlTable(ADDR_USER_LED_1, (uint8_t)digitalRead(BDPIN_GPIO_4));
    slave->setControlTable(ADDR_USER_LED_2, (uint8_t)digitalRead(BDPIN_GPIO_6));
    slave->setControlTable(ADDR_USER_LED_3, (uint8_t)digitalRead(BDPIN_GPIO_8));
    slave->setControlTable(ADDR_USER_LED_4, (uint8_t)digitalRead(BDPIN_GPIO_10));
    slave->setControlTable(ADDR_BUTTON_1, (uint8_t)digitalRead(BDPIN_PUSH_SW_1));
    slave->setControlTable(ADDR_BUTTON_2, (uint8_t)digitalRead(BDPIN_PUSH_SW_2));
    slave->setControlTable(ADDR_BUMPER_1, (uint8_t)sensors.getBumper1State());
    slave->setControlTable(ADDR_BUMPER_2, (uint8_t)sensors.getBumper2State());
  }  
}

void updateBatteryStatus(DYNAMIXEL::Slave *slave, uint32_t interval_ms)
{
  static uint32_t pre_time = 0;
  float bat_voltage, bat_percent;
  uint32_t bat_voltage_uint, bat_percent_uint = 0;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    bat_voltage = sensors.checkVoltage();
    bat_voltage_uint = (uint32_t)(bat_voltage*100);
    slave->setControlTable(ADDR_BATTERY_VOLTAGE, (uint32_t)bat_voltage_uint);
    if(bat_voltage >= 3.5*3){
      bat_percent = map_float(bat_voltage, 3.5*3, 4.1*3, 0.0, 100.0);
      bat_percent_uint = (uint32_t)(bat_percent*100);
    }
    slave->setControlTable(ADDR_BATTERY_PERCENT, (uint32_t)bat_percent_uint);
  }
}

void updateAnalogSensors(DYNAMIXEL::Slave *slave, uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    slave->setControlTable(ADDR_ILLUMINATION, (uint16_t)sensors.getIlluminationData());
    slave->setControlTable(ADDR_IR, (uint32_t)sensors.getIRsensorData());
    slave->setControlTable(ADDR_SORNA, (float)sensors.getSonarData());
  }  
}

void updateIMU(DYNAMIXEL::Slave *slave, uint32_t interval_ms)
{
  static uint32_t pre_time = 0;
  float* p_imu_data;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    p_imu_data = sensors.getImuAngularVelocity();
    slave->setControlTable(ADDR_ANGULAR_VELOCITY_X, (float)p_imu_data[0]);
    slave->setControlTable(ADDR_ANGULAR_VELOCITY_Y, (float)p_imu_data[1]);
    slave->setControlTable(ADDR_ANGULAR_VELOCITY_Z, (float)p_imu_data[2]);

    p_imu_data = sensors.getImuLinearAcc();
    slave->setControlTable(ADDR_LINEAR_ACC_X, (float)p_imu_data[0]);
    slave->setControlTable(ADDR_LINEAR_ACC_Y, (float)p_imu_data[1]);
    slave->setControlTable(ADDR_LINEAR_ACC_Z, (float)p_imu_data[2]);

    p_imu_data = sensors.getImuMagnetic();
    slave->setControlTable(ADDR_MAGNETIC_X, (float)p_imu_data[0]);
    slave->setControlTable(ADDR_MAGNETIC_Y, (float)p_imu_data[1]);
    slave->setControlTable(ADDR_MAGNETIC_Z, (float)p_imu_data[2]);

    p_imu_data = sensors.getOrientation();
    slave->setControlTable(ADDR_ORIENTATION_W, (float)p_imu_data[0]);
    slave->setControlTable(ADDR_ORIENTATION_X, (float)p_imu_data[1]);
    slave->setControlTable(ADDR_ORIENTATION_Y, (float)p_imu_data[2]);
    slave->setControlTable(ADDR_ORIENTATION_Z, (float)p_imu_data[3]);
  }  
}

void updateMotorStatus(DYNAMIXEL::Slave *slave, uint32_t interval_ms)
{
  static uint32_t pre_time = 0;
  int32_t present_position[WHEEL_NUM], present_velocity[WHEEL_NUM];
  int16_t present_current[WHEEL_NUM];

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    if (motor_driver.readPresentPosition(present_position[LEFT], present_position[RIGHT]) == true){
      slave->setControlTable(ADDR_PRESENT_POSITION_L, (int32_t)present_position[LEFT]);
      slave->setControlTable(ADDR_PRESENT_POSITION_R, (int32_t)present_position[RIGHT]);
    }

    if (motor_driver.readPresentVelocity(present_velocity[LEFT], present_velocity[RIGHT]) == true){
      slave->setControlTable(ADDR_PRESENT_VELOCITY_L, (int32_t)present_velocity[LEFT]);
      slave->setControlTable(ADDR_PRESENT_VELOCITY_R, (int32_t)present_velocity[RIGHT]);
    }
    
    if (motor_driver.readPresentCurrent(present_current[LEFT], present_current[RIGHT]) == true){
      slave->setControlTable(ADDR_PRESENT_CURRENT_L, (int32_t)present_current[LEFT]);
      slave->setControlTable(ADDR_PRESENT_CURRENT_R, (int32_t)present_current[RIGHT]);
    }
  }
}



bool isAddrInRange(uint16_t addr, uint16_t length,
  uint16_t range_addr, uint16_t range_length)
{
  return (addr >= range_addr && addr+length <= range_addr+range_length) ? true:false;
}


void dxl_slave_read_callback_func(DYNAMIXEL::Slave *slave, uint16_t addr, uint16_t length)
{
  if(isAddrInRange(ADDR_MILLIS, 4, addr, length) == true){
    slave->setControlTable(ADDR_MILLIS, (uint32_t)millis());
  }

  if(isAddrInRange(ADDR_MICROS, 4, addr, length) == true){
    slave->setControlTable(ADDR_MICROS, (uint32_t)micros());
  }

  if(isAddrInRange(ADDR_MOTOR_TORQUE, 1, addr, length) == true){
    slave->setControlTable(ADDR_MOTOR_TORQUE, (uint8_t)motor_driver.getTorque());
  }
}

void dxl_slave_write_callback_func(DYNAMIXEL::Slave *slave, uint16_t addr, uint16_t length)
{
  if(isAddrInRange(ADDR_SOUND, 1, addr, length) == true){
    uint8_t data;
    if(slave->getControlTable(ADDR_SOUND, 1, (uint8_t*)&data) == DXL_ERR_NONE){
      sensors.makeSound(data);
    }
  }

  if(isAddrInRange(ADDR_MOTOR_TORQUE, 1, addr, length) == true){
    uint8_t data;
    if(slave->getControlTable(ADDR_MOTOR_TORQUE, 1, (uint8_t*)&data) == DXL_ERR_NONE){
      motor_driver.setTorque(data);
    }
  }

  if(isAddrInRange(ADDR_CMD_VEL_LINEAR_X, 4, addr, length) == true){
    int32_t data;
    float f_data;
    if(slave->getControlTable(ADDR_CMD_VEL_LINEAR_X, 4, (uint8_t*)&data) == DXL_ERR_NONE){
      f_data = data;
      f_data *= 0.01f;
      goal_velocity_from_cmd[LINEAR] = constrain(f_data, MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    }
  }

  if(isAddrInRange(ADDR_CMD_VEL_ANGULAR_Z, 4, addr, length) == true){
    int32_t data;
    float f_data;
    if(slave->getControlTable(ADDR_CMD_VEL_ANGULAR_Z, 4, (uint8_t*)&data) == DXL_ERR_NONE){
      f_data = data;
      f_data *= 0.01f;
      goal_velocity_from_cmd[ANGULAR] = constrain(f_data, MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    }
  }

  if(isAddrInRange(ADDR_PROFILE_ACC_L, 4, addr, length) == true
    || isAddrInRange(ADDR_PROFILE_ACC_R, 4, addr, length) == true)
  {
    uint32_t l_data, r_data;
    if(slave->getControlTable(ADDR_PROFILE_ACC_L, 4, (uint8_t*)&l_data) == DXL_ERR_NONE){
      if(slave->getControlTable(ADDR_PROFILE_ACC_R, 4, (uint8_t*)&r_data) == DXL_ERR_NONE){
        if(sensors.checkVoltage() >= 7.0){
          motor_driver.writeProfileAcceleration(l_data, r_data);
        }
      }
    }
  }
}


