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

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho, Gilbert, Kei Ki */

#include "turtlebot3_core_config.h"

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  DEBUG_SERIAL.begin(57600);
  while(!RTPS_SERIAL)
  {
    DEBUG_PRINT(".");
  }

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

  ros2::init(&RTPS_SERIAL);

  sprintf(imu_frame_id, "imu_link");
  sprintf(joint_state_header_frame_id, "base_link");
  sprintf(sensor_state_header_frame_id, "sensor_state");
  
  setup_end = true;
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  static TurtleBot3 turtlebot3_node;

  uint32_t t = millis();
  updateVariable(true);
  //updateTFPrefix(true);

  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    updateGoalVelocity();
    motor_driver.controlMotor(WHEEL_SEPARATION, goal_velocity);
    tTime[0] = t;
  }

#ifdef DEBUG
  if ((t-tTime[1]) >= (1000 / DEBUG_LOG_FREQUENCY))
  {
    //sendDebuglog();
    tTime[1] = t;
  }
#endif

  // Send log message after ROS connection
  //sendLogMsg();

  // Receive data from RC100 
  //controllers.getRCdata(goal_velocity_from_rc100);

  // Check push button pressed for simple test drive
  driveTest(diagnosis.getButtonPress(3000));

  // Update the IMU unit
  sensors.updateIMU();

  // TODO
  // Update sonar data
  // sensors.updateSonar(t);

  // Start Gyro Calibration after ROS connection
  updateGyroCali(true);

  // Show LED status
  diagnosis.showLedStatus(true);

  // Update Voltage
  //battery_state = diagnosis.updateVoltageCheck(setup_end);

  // Call all the callbacks waiting to be called at that point in time
  ros2::spin(&turtlebot3_node);
}


/*******************************************************************************
* Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
     // nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");  

        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/base_link");
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      //nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      //nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      //nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      //nh.loginfo(log_msg); 

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}




/*******************************************************************************
* Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};
  
  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index]   = 0.0;
      last_tick[index]        = 0.0;
      joint_states_pos[index] = 0.0;
      joint_states_vel[index] = 0.0;
    }  

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;
  last_diff_tick[LEFT]     = current_tick - last_tick[LEFT];
  last_tick[LEFT]          = current_tick;
  joint_states_pos[LEFT]  += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;
  last_diff_tick[RIGHT]    = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]         = current_tick;
  joint_states_pos[RIGHT] += TICK2RAD * (double)last_diff_tick[RIGHT];
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;  
  orientation = sensors.getOrientation();
  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  joint_states_vel[LEFT]  = wheel_l / step_time;
  joint_states_vel[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
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

  motor_driver.readEncoder(current_tick[LEFT], current_tick[RIGHT]);

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
  
  if (isConnected)
  {
    if (variable_flag == false)
    {      
      sensors.initIMU();
      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}


/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  if (isConnected)//(nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      //nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      //nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*******************************************************************************
* Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];  

  String name             = NAME;
  String firmware_version = FIRMWARE_VER;
  String bringup_log      = "This core(v" + firmware_version + ") is compatible with TB3 " + name;
   
  const char* init_log_data = bringup_log.c_str();

  if (true)//(nh.connected())
  {
    if (log_flag == false)
    {      
      sprintf(log_msg, "--------------------------");
      //nh.loginfo(log_msg);

      sprintf(log_msg, "Connected to OpenCR board!");
      //nh.loginfo(log_msg);

      sprintf(log_msg, init_log_data);
      //nh.loginfo(log_msg);

      sprintf(log_msg, "--------------------------");
      //nh.loginfo(log_msg);

      log_flag = true;
    }
  }
  else
  {
    log_flag = false;
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

/*******************************************************************************
* Send Debug data
*******************************************************************************/
void sendDebuglog(void)
{
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("EXTERNAL SENSORS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.print("Bumper : "); DEBUG_SERIAL.println(sensors.checkPushBumper());
  DEBUG_SERIAL.print("Cliff : "); DEBUG_SERIAL.println(sensors.getIRsensorData());
  DEBUG_SERIAL.print("Sonar : "); DEBUG_SERIAL.println(sensors.getSonarData());
  DEBUG_SERIAL.print("Illumination : "); DEBUG_SERIAL.println(sensors.getIlluminationData());

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("OpenCR SENSORS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.print("Battery : "); DEBUG_SERIAL.println(sensors.checkVoltage());
  DEBUG_SERIAL.println("Button : " + String(sensors.checkPushButton()));

  float* quat = sensors.getOrientation();

  DEBUG_SERIAL.println("IMU : ");
  DEBUG_SERIAL.print("    w : "); DEBUG_SERIAL.println(quat[0]);
  DEBUG_SERIAL.print("    x : "); DEBUG_SERIAL.println(quat[1]);
  DEBUG_SERIAL.print("    y : "); DEBUG_SERIAL.println(quat[2]);
  DEBUG_SERIAL.print("    z : "); DEBUG_SERIAL.println(quat[3]);
  
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("DYNAMIXELS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Torque : " + String(motor_driver.getTorque()));

  int32_t encoder[WHEEL_NUM] = {0, 0};
  motor_driver.readEncoder(encoder[LEFT], encoder[RIGHT]);
  
  DEBUG_SERIAL.println("Encoder(left) : " + String(encoder[LEFT]));
  DEBUG_SERIAL.println("Encoder(right) : " + String(encoder[RIGHT]));

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("TurtleBot3");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Odometry : ");   
  DEBUG_SERIAL.print("         x : "); DEBUG_SERIAL.println(odom_pose[0]);
  DEBUG_SERIAL.print("         y : "); DEBUG_SERIAL.println(odom_pose[1]);
  DEBUG_SERIAL.print("     theta : "); DEBUG_SERIAL.println(odom_pose[2]);
}



//IMU data: angular velocity, linear acceleration, orientation
void publishImu(sensor_msgs::Imu* msg, void* arg)
{
  (void)(arg);
  sensor_msgs::Imu imu_msg = sensors.getIMU();
  memcpy(msg, &imu_msg, sizeof(sensor_msgs::Imu));
  
  msg->header.stamp    = ros2::now();
  strcpy(msg->header.frame_id, imu_frame_id);
}


//Odometry : 
void publishOdometry(nav_msgs::Odometry* msg, void* arg)
{
  (void)(arg);
  unsigned long time_now       = millis();
  unsigned long step_time      = time_now - prev_update_time;
  calcOdometry((double)(step_time * 0.001));

  msg->header.stamp            = ros2::now();
  strcpy(msg->header.frame_id, odom_header_frame_id);
  strcpy(msg->child_frame_id, odom_child_frame_id);
  msg->pose.pose.position.x    = odom_pose[0];
  msg->pose.pose.position.y    = odom_pose[1];
  msg->pose.pose.position.z    = 0;
  //msg->pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);
  msg->pose.pose.orientation.x = 0;
  msg->pose.pose.orientation.y = 0;
  msg->pose.pose.orientation.z = 0;
  msg->pose.pose.orientation.w = 0;
  msg->twist.twist.linear.x    = odom_vel[0];
  msg->twist.twist.angular.z   = odom_vel[2];
}


//Joint State : 
void publishJointState(sensor_msgs::JointState* msg, void* arg)
{
  (void)(arg);
  static double effort_val[WHEEL_NUM] = {0, 0};

  msg->header.stamp    = ros2::now();
  strcpy(msg->header.frame_id, joint_state_header_frame_id);
  strcpy(msg->name[0], "wheel_left_joint");
  strcpy(msg->name[1], "wheel_right_joint");
  msg->name_size       = WHEEL_NUM;
  msg->position_size   = WHEEL_NUM;
  msg->velocity_size   = WHEEL_NUM;
  msg->effort_size     = WHEEL_NUM;
  memcpy(msg->position, joint_states_pos, sizeof(joint_states_pos));
  memcpy(msg->velocity, joint_states_vel, sizeof(joint_states_vel));
  memcpy(msg->effort, effort_val, sizeof(effort_val));
}


//sensor_state: bumpers, cliffs, buttons, encoders, battery
void publishSensorState(turtlebot3_msgs::SensorState* msg, void* arg)
{
  (void)(arg);
  msg->header.stamp = ros2::now();
  strcpy(msg->header.frame_id, sensor_state_header_frame_id);
  msg->bumper       = sensors.checkPushBumper();
  msg->cliff        = sensors.getIRsensorData();
  msg->sonar        = 0.0 ;//TODO : sensors.getSonarData();
  msg->illumination = sensors.getIlluminationData();
  msg->led          = 0;
  msg->button       = sensors.checkPushButton();
  msg->torque       = motor_driver.getTorque();
  if (motor_driver.readEncoder(msg->left_encoder, msg->right_encoder))
  {
    updateMotorInfo(msg->left_encoder, msg->right_encoder);
  }
  msg->battery      = sensors.checkVoltage();
}


//version info
void publishVersionInfo(turtlebot3_msgs::VersionInfo* msg, void* arg)
{
  (void)(arg);
  strcpy(msg->hardware, (char*)HARDWARE_VER);
  strcpy(msg->software, (char*)SOFTWARE_VER);
  strcpy(msg->firmware, (char*)FIRMWARE_VER);  
}



void subscribeCmdVel(geometry_msgs::Twist* msg, void* arg)
{
  (void)(arg);
  goal_velocity_from_cmd[LINEAR]  = constrain(msg->linear.x,  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(msg->angular.z, MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
}


void subscribeSound(turtlebot3_msgs::Sound* msg, void* arg)
{
  (void)(arg);
  sensors.makeSound(msg->value);
}


void subscribeMotorPower(std_msgs::Bool* msg, void* arg)
{ 
  (void)(arg);
  motor_driver.setTorque(msg->data);
}


void subscribeReset(std_msgs::Empty* msg, void* arg)
{
  (void)(msg);
  (void)(arg);

  char log_msg[50];

  sprintf(log_msg, "Start Calibration of Gyro");
  DEBUG_SERIAL.println(log_msg);
  //nh.loginfo(log_msg);

  sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  DEBUG_SERIAL.println(log_msg);
  //nh.loginfo(log_msg);

  sprintf(log_msg, "Reset Odometry");
  DEBUG_SERIAL.println(log_msg);
  //nh.loginfo(log_msg);  
}


void subscribeTimeSync(builtin_interfaces::Time* msg, void* arg)
{
  (void)(arg);
  
  ros2::syncTimeFromRemote(msg);
}
