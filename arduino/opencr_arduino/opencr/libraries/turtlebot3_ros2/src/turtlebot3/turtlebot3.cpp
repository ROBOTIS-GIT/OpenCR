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

#include "../../include/turtlebot3/turtlebot3.h"

/*******************************************************************************
* Definition of dependency data according to TB3 model.
*******************************************************************************/
typedef struct TB3ModelInfo{
  const char* model_str;
  uint32_t model_info;
  float wheel_radius;
  float wheel_separation;
  float turning_radius;
  float robot_radius;
} TB3ModelInfo;

static const TB3ModelInfo burger_info = {
  "Burger",
  1,
  0.033,
  0.160,
  0.080,
  0.105
};

static const TB3ModelInfo waffle_info = {
  "Waffle",
  2,
  0.033,
  0.287,
  0.1435,
  0.220
};


/*******************************************************************************
* Declaration for motors
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;

static const TB3ModelInfo* p_tb3_model_info;
static float max_linear_velocity, min_linear_velocity;
static float max_angular_velocity, min_angular_velocity;

static float goal_velocity[VelocityType::TYPE_NUM_MAX] = {0.0, 0.0};
static float goal_velocity_from_cmd[MortorLocation::MOTOR_NUM_MAX] = {0.0, 0.0};
static float goal_velocity_from_rc100[MortorLocation::MOTOR_NUM_MAX] = {0.0, 0.0};
static float goal_velocity_from_button[MortorLocation::MOTOR_NUM_MAX] = {0.0, 0.0};

static void update_goal_velocity_from_3values(void);
static void test_motors_with_buttons(uint8_t buttons);
static bool get_connection_state_with_motors();
static void set_connection_state_with_motors(bool is_connected);

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
Turtlebot3Sensor sensors;

/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
Turtlebot3Diagnosis diagnosis;

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
Turtlebot3Controller controllers;

/*******************************************************************************
* Declaration for DYNAMIXEL Slave Function
*******************************************************************************/
#define SERIAL_DXL_SLAVE Serial
const uint8_t ID_DXL_SLAVE = 200;
const uint16_t MODEL_NUM_DXL_SLAVE = 0x5000;
const float PROTOCOL_VERSION_DXL_SLAVE = 2.0;
const uint32_t HEARTBEAT_TIMEOUT_MS = 500;

static bool is_dxl_addr_in_range(uint16_t addr, uint16_t length, uint16_t range_addr, uint16_t range_length);
static void dxl_slave_write_callback_func(DYNAMIXEL::Slave *slave, uint16_t addr, uint16_t length);

static bool get_connection_state_with_ros2_node();
static void set_connection_state_with_ros2_node(bool is_connected);
static void update_connection_state_with_ros2_node(DYNAMIXEL::Slave *slave);

static void update_imu_to_table(DYNAMIXEL::Slave *slave, uint32_t interval_ms);
static void update_times_to_table(DYNAMIXEL::Slave *slave, uint32_t interval_ms);
static void update_gpios_to_table(DYNAMIXEL::Slave *slave, uint32_t interval_ms);
static void update_motor_status_to_table(DYNAMIXEL::Slave *slave, uint32_t interval_ms);
static void update_battery_status_to_table(DYNAMIXEL::Slave *slave, uint32_t interval_ms);
static void update_analog_sensors_to_table(DYNAMIXEL::Slave *slave, uint32_t interval_ms);

DYNAMIXEL::USBSerialPortHandler port_dxl_slave(SERIAL_DXL_SLAVE);
DYNAMIXEL::Slave dxl_slave(port_dxl_slave, MODEL_NUM_DXL_SLAVE, PROTOCOL_VERSION_DXL_SLAVE);

enum ControlTableItemAddr{
  ADDR_MODEL_NUMBER    = 0,
  ADDR_MODEL_INFORM    = 2,
  ADDR_FIRMWARE_VER    = 6,
  ADDR_ID              = 7,
  ADDR_BAUDRATE        = 8,
  
  ADDR_MILLIS          = 10,
  ADDR_MICROS          = 14,

  ADDR_DEVICE_STATUS   = 18,
  ADDR_HEARTBEAT       = 19,

  ADDR_USER_LED_1      = 20,
  ADDR_USER_LED_2      = 21,
  ADDR_USER_LED_3      = 22,
  ADDR_USER_LED_4      = 23,

  ADDR_BUTTON_1        = 26,
  ADDR_BUTTON_2        = 27,
  ADDR_BUMPER_1        = 28,
  ADDR_BUMPER_2        = 29,
  ADDR_ILLUMINATION    = 30,
  ADDR_IR              = 34,
  ADDR_SORNA           = 38,
  ADDR_BATTERY_VOLTAGE = 42,
  ADDR_BATTERY_PERCENT = 46,
  ADDR_SOUND           = 50,

  ADDR_IMU_RECALIBRATION  = 59,
  ADDR_ANGULAR_VELOCITY_X = 60,
  ADDR_ANGULAR_VELOCITY_Y = 64,
  ADDR_ANGULAR_VELOCITY_Z = 68,
  ADDR_LINEAR_ACC_X       = 72,
  ADDR_LINEAR_ACC_Y       = 76,
  ADDR_LINEAR_ACC_Z       = 80,
  ADDR_MAGNETIC_X         = 84,
  ADDR_MAGNETIC_Y         = 88,
  ADDR_MAGNETIC_Z         = 92,
  ADDR_ORIENTATION_W      = 96,
  ADDR_ORIENTATION_X      = 100,
  ADDR_ORIENTATION_Y      = 104,
  ADDR_ORIENTATION_Z      = 108,
  
  ADDR_PRESENT_CURRENT_L  = 120,
  ADDR_PRESENT_CURRENT_R  = 124,
  ADDR_PRESENT_VELOCITY_L = 128,
  ADDR_PRESENT_VELOCITY_R = 132,
  ADDR_PRESENT_POSITION_L = 136,
  ADDR_PRESENT_POSITION_R = 140,
  
  ADDR_MOTOR_TORQUE       = 149,
  ADDR_CMD_VEL_LINEAR_X   = 150,
  ADDR_CMD_VEL_LINEAR_Y   = 154,
  ADDR_CMD_VEL_LINEAR_Z   = 158,
  ADDR_CMD_VEL_ANGULAR_X  = 162,
  ADDR_CMD_VEL_ANGULAR_Y  = 166,
  ADDR_CMD_VEL_ANGULAR_Z  = 170,
  ADDR_PROFILE_ACC_L      = 174,
  ADDR_PROFILE_ACC_R      = 178
};


/*******************************************************************************
* Definition for TurtleBot3Core 'begin()' function
*******************************************************************************/
void TurtleBot3Core::begin(const char* model_name)
{
  uint16_t model_motor_rpm;

  if(strcmp(model_name, "Burger") == 0 || strcmp(model_name, "burger") == 0){
    p_tb3_model_info = &burger_info;
    model_motor_rpm = 61;
  }else if(strcmp(model_name, "Waffle") == 0 || strcmp(model_name, "waffle") == 0){
    p_tb3_model_info = &waffle_info;
    model_motor_rpm = 77;
  }else{
    p_tb3_model_info = &burger_info;
    model_motor_rpm = 61;
  }

  max_linear_velocity = p_tb3_model_info->wheel_radius*2*PI*model_motor_rpm/60;
  min_linear_velocity = -max_linear_velocity;
  max_angular_velocity = max_linear_velocity/p_tb3_model_info->turning_radius;
  min_angular_velocity = -max_angular_velocity;

  pinMode(LED_WORKING_CHECK, OUTPUT);

  // Setting for Dynamixel motors
  motor_driver.init();
  // Setting for IMU
  sensors.init();
  // Init diagnosis
  diagnosis.init();
  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  controllers.init(max_linear_velocity, max_angular_velocity);

  // Init DXL Slave function
  dxl_slave.setPortProtocolVersion(PROTOCOL_VERSION_DXL_SLAVE);
  dxl_slave.setID(ID_DXL_SLAVE);
  dxl_slave.setFirmwareVersion(FIRMWARE_VER);

  dxl_slave.setControlTable(ADDR_MODEL_NUMBER, (uint16_t)dxl_slave.getModelNumber());
  dxl_slave.setControlTable(ADDR_MODEL_INFORM, (uint32_t)p_tb3_model_info->model_info);
  dxl_slave.setControlTable(ADDR_FIRMWARE_VER, (uint8_t)dxl_slave.getFirmwareVersion());
  dxl_slave.setControlTable(ADDR_ID, (uint8_t)dxl_slave.getID());
  dxl_slave.setControlTable(ADDR_BAUDRATE, (uint8_t)3);
  
  dxl_slave.setAddrRemoteWriteProtected(0, 255, true);
  dxl_slave.setAddrRemoteWriteProtected(ADDR_HEARTBEAT, 1, false);
  dxl_slave.setAddrRemoteWriteProtected(ADDR_USER_LED_1, 4, false);
  dxl_slave.setAddrRemoteWriteProtected(ADDR_SOUND, 1, false);
  dxl_slave.setAddrRemoteWriteProtected(ADDR_IMU_RECALIBRATION, 1, false);
  dxl_slave.setAddrRemoteWriteProtected(ADDR_MOTOR_TORQUE, ADDR_PROFILE_ACC_R+4-ADDR_MOTOR_TORQUE, false);

  dxl_slave.setWriteCallbackFunc(dxl_slave_write_callback_func);

  if(motor_driver.is_connected() == true){
    motor_driver.set_torque(true);
    dxl_slave.setControlTable(ADDR_DEVICE_STATUS, (int8_t)STATUS_RUNNING);
    set_connection_state_with_motors(true);
  }else{
    dxl_slave.setControlTable(ADDR_DEVICE_STATUS, (int8_t)STATUS_NOT_CONNECTED_MOTORS);
    set_connection_state_with_motors(false);
  }
 
  sensors.initIMU();
  sensors.calibrationGyro();
  sensors.makeMelody(1); //To indicate that the initialization is complete.
}

/*******************************************************************************
* Definition for TurtleBot3Core 'run()' function
*******************************************************************************/
void TurtleBot3Core::run()
{
  static uint32_t pre_time_to_control_motor;

  // Check connection state with ROS2 node
  update_connection_state_with_ros2_node(&dxl_slave);

  /* For diagnosis */
  // Show LED status
  diagnosis.showLedStatus(get_connection_state_with_ros2_node());
  // Update Voltage
  diagnosis.updateVoltageCheck(true);
  // Check push button pressed for simple test drive
  test_motors_with_buttons(diagnosis.getButtonPress(3000));

  /* For sensing and run buzzer */
  // Update the IMU unit
  sensors.updateIMU();
  // Update sonar data
  // TODO: sensors.updateSonar(t);
  // Run buzzer if there is still melody to play.
  sensors.onMelody();

  /* For getting command from rc100 */
  // Receive data from RC100 
  controllers.getRCdata(goal_velocity_from_rc100);

  /* For processing DYNAMIXEL slave function */
  // Update control table of OpenCR to communicate with ROS2 node
  update_imu_to_table(&dxl_slave, INTERVAL_MS_TO_UPDATE_CONTROL_TABLE);
  update_times_to_table(&dxl_slave, INTERVAL_MS_TO_UPDATE_CONTROL_TABLE);
  update_gpios_to_table(&dxl_slave, INTERVAL_MS_TO_UPDATE_CONTROL_TABLE);
  update_motor_status_to_table(&dxl_slave, INTERVAL_MS_TO_UPDATE_CONTROL_TABLE);
  update_battery_status_to_table(&dxl_slave, INTERVAL_MS_TO_UPDATE_CONTROL_TABLE);
  update_analog_sensors_to_table(&dxl_slave, INTERVAL_MS_TO_UPDATE_CONTROL_TABLE);
  // Packet processing with ROS2 Node.
  dxl_slave.processPacket();

  /* For controlling DYNAMIXEL motors (Wheels) */  
  if (millis()-pre_time_to_control_motor >= INTERVAL_MS_TO_CONTROL_MOTOR)
  {
    pre_time_to_control_motor = millis();
    if(get_connection_state_with_ros2_node() == false){
      memset(goal_velocity_from_cmd, 0, sizeof(goal_velocity_from_cmd));
    }
    update_goal_velocity_from_3values();
    if(get_connection_state_with_motors() == true){
      motor_driver.control_motors(p_tb3_model_info->wheel_separation, goal_velocity[VelocityType::LINEAR], goal_velocity[VelocityType::ANGULAR]);
    }
  }  
}


/*******************************************************************************
* Function definition for updating velocity values 
* to be used for control of DYNAMIXEL(motors).
*******************************************************************************/
void update_goal_velocity_from_3values(void)
{
  goal_velocity[VelocityType::LINEAR]  = goal_velocity_from_button[VelocityType::LINEAR]  + goal_velocity_from_cmd[VelocityType::LINEAR]  + goal_velocity_from_rc100[VelocityType::LINEAR];
  goal_velocity[VelocityType::ANGULAR] = goal_velocity_from_button[VelocityType::ANGULAR] + goal_velocity_from_cmd[VelocityType::ANGULAR] + goal_velocity_from_rc100[VelocityType::ANGULAR];

  sensors.setLedPattern(goal_velocity[VelocityType::LINEAR], goal_velocity[VelocityType::ANGULAR]);
}


/*******************************************************************************
* Function definition for updating control tables in TB3.
*******************************************************************************/
float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void update_times_to_table(DYNAMIXEL::Slave *slave, uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    slave->setControlTable(ADDR_MILLIS, (uint32_t)millis());
    slave->setControlTable(ADDR_MICROS, (uint32_t)micros());
  } 
}

void update_gpios_to_table(DYNAMIXEL::Slave *slave, uint32_t interval_ms)
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

void update_battery_status_to_table(DYNAMIXEL::Slave *slave, uint32_t interval_ms)
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

void update_analog_sensors_to_table(DYNAMIXEL::Slave *slave, uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    slave->setControlTable(ADDR_ILLUMINATION, (uint16_t)sensors.getIlluminationData());
    slave->setControlTable(ADDR_IR, (uint32_t)sensors.getIRsensorData());
    slave->setControlTable(ADDR_SORNA, (float)sensors.getSonarData());
  }  
}

void update_imu_to_table(DYNAMIXEL::Slave *slave, uint32_t interval_ms)
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

void update_motor_status_to_table(DYNAMIXEL::Slave *slave, uint32_t interval_ms)
{
  static uint32_t pre_time;
  int32_t present_position[MortorLocation::MOTOR_NUM_MAX], present_velocity[MortorLocation::MOTOR_NUM_MAX];
  int16_t present_current[MortorLocation::MOTOR_NUM_MAX];

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();
    if(get_connection_state_with_motors() == true){
      if (motor_driver.read_present_position(present_position[MortorLocation::LEFT], present_position[MortorLocation::RIGHT]) == true){
        slave->setControlTable(ADDR_PRESENT_POSITION_L, (int32_t)present_position[MortorLocation::LEFT]);
        slave->setControlTable(ADDR_PRESENT_POSITION_R, (int32_t)present_position[MortorLocation::RIGHT]);
      }

      if (motor_driver.read_present_velocity(present_velocity[MortorLocation::LEFT], present_velocity[MortorLocation::RIGHT]) == true){
        slave->setControlTable(ADDR_PRESENT_VELOCITY_L, (int32_t)present_velocity[MortorLocation::LEFT]);
        slave->setControlTable(ADDR_PRESENT_VELOCITY_R, (int32_t)present_velocity[MortorLocation::RIGHT]);
      }
      
      if (motor_driver.read_present_current(present_current[MortorLocation::LEFT], present_current[MortorLocation::RIGHT]) == true){
        slave->setControlTable(ADDR_PRESENT_CURRENT_L, (int32_t)present_current[MortorLocation::LEFT]);
        slave->setControlTable(ADDR_PRESENT_CURRENT_R, (int32_t)present_current[MortorLocation::RIGHT]);
      }

      slave->setControlTable(ADDR_MOTOR_TORQUE, (uint8_t)motor_driver.get_torque());
    }
  }
}


/*******************************************************************************
* Callback function definition to be used in communication with the ROS2 node.
*******************************************************************************/
static bool is_dxl_addr_in_range(uint16_t addr, uint16_t length,
  uint16_t range_addr, uint16_t range_length)
{
  return (addr >= range_addr && addr+length <= range_addr+range_length) ? true:false;
}

static void dxl_slave_write_callback_func(DYNAMIXEL::Slave *slave, uint16_t addr, uint16_t length)
{
  if(is_dxl_addr_in_range(ADDR_SOUND, 1, addr, length) == true){
    uint8_t data;
    if(slave->getControlTable(ADDR_SOUND, 1, (uint8_t*)&data) == DXL_ERR_NONE){
      sensors.makeMelody(data);
    }
  }

  if(is_dxl_addr_in_range(ADDR_MOTOR_TORQUE, 1, addr, length) == true){
    uint8_t data;
    if(slave->getControlTable(ADDR_MOTOR_TORQUE, 1, (uint8_t*)&data) == DXL_ERR_NONE){
      if(get_connection_state_with_motors() == true){
        motor_driver.set_torque(data);
      }
    }
  }

  if(is_dxl_addr_in_range(ADDR_CMD_VEL_LINEAR_X, 4, addr, length) == true){
    int32_t data;
    float f_data;
    if(slave->getControlTable(ADDR_CMD_VEL_LINEAR_X, 4, (uint8_t*)&data) == DXL_ERR_NONE){
      f_data = data;
      f_data *= 0.01f;
      goal_velocity_from_cmd[VelocityType::LINEAR] = constrain(f_data, min_linear_velocity, max_linear_velocity);
    }
  }

  if(is_dxl_addr_in_range(ADDR_CMD_VEL_ANGULAR_Z, 4, addr, length) == true){
    int32_t data;
    float f_data;
    if(slave->getControlTable(ADDR_CMD_VEL_ANGULAR_Z, 4, (uint8_t*)&data) == DXL_ERR_NONE){
      f_data = data;
      f_data *= 0.01f;
      goal_velocity_from_cmd[VelocityType::ANGULAR] = constrain(f_data, min_angular_velocity, max_angular_velocity);
    }
  }

  if(is_dxl_addr_in_range(ADDR_PROFILE_ACC_L, 4, addr, length) == true
    || is_dxl_addr_in_range(ADDR_PROFILE_ACC_R, 4, addr, length) == true)
  {
    uint32_t l_data, r_data;
    if(slave->getControlTable(ADDR_PROFILE_ACC_L, 4, (uint8_t*)&l_data) == DXL_ERR_NONE){
      if(slave->getControlTable(ADDR_PROFILE_ACC_R, 4, (uint8_t*)&r_data) == DXL_ERR_NONE){
        if(get_connection_state_with_motors() == true){
          motor_driver.write_profile_acceleration(l_data, r_data);
        }
      }
    }
  }

  if(is_dxl_addr_in_range(ADDR_IMU_RECALIBRATION, 1, addr, length) == true){
    bool data;
    if(slave->getControlTable(ADDR_IMU_RECALIBRATION, 1, (uint8_t*)&data) == DXL_ERR_NONE){
      if(data == true){
        sensors.calibrationGyro();
        slave->setControlTable(ADDR_IMU_RECALIBRATION, (uint8_t)0);
      }
    }
  }  
}


/*******************************************************************************
* Function definition to check the connection status with the ROS2 node.
*******************************************************************************/
static bool connection_state_with_ros2_node = false;

static bool get_connection_state_with_ros2_node()
{
  return connection_state_with_ros2_node;
}

static void set_connection_state_with_ros2_node(bool is_connected)
{
  connection_state_with_ros2_node = is_connected;
}

void update_connection_state_with_ros2_node(DYNAMIXEL::Slave *slave)
{
  static uint32_t pre_time;
  static uint8_t pre_data, data;
  static bool pre_state;

  //To wait for IMU Calibration
  if(pre_state != get_connection_state_with_ros2_node()){
    pre_state = get_connection_state_with_ros2_node();
    pre_time = millis();
    return;
  }

  if(slave->getControlTable(ADDR_HEARTBEAT, 1, (uint8_t*)&data) == DXL_ERR_NONE){
    if(pre_data != data){
      pre_time = millis();
      pre_data = data;
      set_connection_state_with_ros2_node(true);
    }else{
      if(millis()-pre_time >= HEARTBEAT_TIMEOUT_MS){
        pre_time = millis();
        set_connection_state_with_ros2_node(false);
      }
    }
  }
}


/*******************************************************************************
* Function definition to check the connection with the motor.
*******************************************************************************/
static bool is_connected_motors = false;

static bool get_connection_state_with_motors()
{
  return is_connected_motors;
}

static void set_connection_state_with_motors(bool is_connected)
{
  is_connected_motors = is_connected;
}


/*******************************************************************************
* Function definition to test motors using the built-in buttons of OpenCR.
*******************************************************************************/
const float TICK2RAD = 0.001533981; // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
const float TEST_DISTANCE = 0.300; // meter
const float TEST_RADIAN = 3.14; // 180 degree

void test_motors_with_buttons(uint8_t buttons)
{
  static bool move[2] = {false, false};
  static int32_t saved_tick[2] = {0, 0};
  static double diff_encoder = 0.0;

  int32_t current_tick[2] = {0, 0};

  if(get_connection_state_with_motors() == true){
    motor_driver.read_present_position(current_tick[MortorLocation::LEFT], current_tick[MortorLocation::RIGHT]);
  }

  if (buttons & (1<<0))  
  {
    move[VelocityType::LINEAR] = true;
    saved_tick[MortorLocation::RIGHT] = current_tick[MortorLocation::RIGHT];

    diff_encoder = TEST_DISTANCE / (0.207 / 4096); // (Circumference of Wheel) / (The number of tick per revolution)
  }
  else if (buttons & (1<<1))
  {
    move[VelocityType::ANGULAR] = true;
    saved_tick[MortorLocation::RIGHT] = current_tick[MortorLocation::RIGHT];

    diff_encoder = (TEST_RADIAN * p_tb3_model_info->turning_radius) / (0.207 / 4096);
  }

  if (move[VelocityType::LINEAR])
  {    
    if (abs(saved_tick[MortorLocation::RIGHT] - current_tick[MortorLocation::RIGHT]) <= diff_encoder)
    {
      goal_velocity_from_button[VelocityType::LINEAR]  = 0.05;
    }
    else
    {
      goal_velocity_from_button[VelocityType::LINEAR]  = 0.0;
      move[VelocityType::LINEAR] = false;
    }
  }
  else if (move[VelocityType::ANGULAR])
  {   
    if (abs(saved_tick[MortorLocation::RIGHT] - current_tick[MortorLocation::RIGHT]) <= diff_encoder)
    {
      goal_velocity_from_button[VelocityType::ANGULAR]= -0.7;
    }
    else
    {
      goal_velocity_from_button[VelocityType::ANGULAR]  = 0.0;
      move[VelocityType::ANGULAR] = false;
    }
  }
}