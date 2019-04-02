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

#ifndef TURTLEBOT3_CORE_CONFIG_H_
#define TURTLEBOT3_CORE_CONFIG_H_

#include <ros2arduino.h>

#include <TurtleBot3_ROS2.h>
#include "turtlebot3_burger.h"

#include <math.h>

#define HARDWARE_VER "1.0.0"
#define SOFTWARE_VER "1.0.0"
#define FIRMWARE_VER "1.0.1"

#define SENSOR_STATE_PUBLISH_FREQUENCY         30    //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define ODOMETRY_PUBLISH_FREQUENCY             30  //hz
#define JOINT_STATE_PUBLISH_FREQUENCY          30   //hz
#define BATTERY_STATE_PUBLISH_FREQUENCY        30   //hz
#define MAGNETIC_FIELD_PUBLISH_FREQUENCY       30   //hz

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define DEBUG_LOG_FREQUENCY                    10   //hz
 
#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14      // 180 degree

//#define DEBUG                            
#define DEBUG_SERIAL                     SerialBT2
#define RTPS_SERIAL                      Serial
#ifdef DEBUG
  #define DEBUG_PRINT(x)                 DEBUG_SERIAL.print(x)
#else
  #define DEBUG_PRINT(x)                  
#endif



/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;

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
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_button[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_rc100[WHEEL_NUM] = {0.0, 0.0};

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double  joint_states_pos[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  joint_states_vel[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
bool setup_end        = false;
uint8_t battery_state = 0;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];
char sensor_state_header_frame_id[30];


void updateTFPrefix(bool isConnected);
void updateVariable(bool isConnected);
void updateGyroCali(bool isConnected);
void updateGoalVelocity(void);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
bool calcOdometry(double diff_time);
void driveTest(uint8_t buttons);
void sendLogMsg(void);
void sendDebuglog(void);


void publishOdometry(nav_msgs::Odometry* msg, void* arg);
void publishImu(sensor_msgs::Imu* msg, void* arg);
void publishJointState(sensor_msgs::JointState* msg, void* arg);
void publishSensorState(turtlebot3_msgs::SensorState* msg, void* arg);
void publishVersionInfo(turtlebot3_msgs::VersionInfo* msg, void* arg);

//void publishCmdVelRC100(geometry_msgs::Twist* msg, void* arg);
//void publishBatteryState(sensor_msgs::BatteryState* msg, void* arg);
//void publishMagneticField(sensor_msgs::MagneticField* msg, void* arg);

void subscribeCmdVel(geometry_msgs::Twist* msg, void* arg);
void subscribeSound(turtlebot3_msgs::Sound* msg, void* arg);
void subscribeMotorPower(std_msgs::Bool* msg, void* arg);
void subscribeReset(std_msgs::Empty* msg, void* arg);
void subscribeTimeSync(builtin_interfaces::Time* msg, void* arg);


/*******************************************************************************
* TurtleBot3 Node Class
*******************************************************************************/
class TurtleBot3 : public ros2::Node
{
public:
  TurtleBot3()
  : Node()
  {
    /*******************************************************************************
    * Publisher
    *******************************************************************************/
    // // Odometry of Turtlebot3
    // odom_pub_          = this->createPublisher<nav_msgs::Odometry>("odom");
    // this->createWallFreq(ODOMETRY_PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishOdometry, NULL, odom_pub_);
    // DEBUG_PRINT("\r\n [Publisher Create]   /odom           : "); DEBUG_PRINT((odom_pub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    // Sensor State
    sensor_state_pub_  = this->createPublisher<turtlebot3_msgs::SensorState>("sensor_state");
    this->createWallFreq(SENSOR_STATE_PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishSensorState, NULL, sensor_state_pub_);  
    DEBUG_PRINT("\r\n [Publisher Create]   /sensor_state   : "); DEBUG_PRINT((sensor_state_pub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    // IMU of Turtlebot3
    imu_pub_           = this->createPublisher<sensor_msgs::Imu>("imu");
    this->createWallFreq(IMU_PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishImu, NULL, imu_pub_);
    DEBUG_PRINT("\r\n [Publisher Create]   /imu            : "); DEBUG_PRINT((imu_pub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    // Joint(Dynamixel) state of Turtlebot3
    // joint_states_pub_  = this->createPublisher<sensor_msgs::JointState>("joint_states");
    // this->createWallFreq(JOINT_STATE_PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishJointState, NULL, joint_states_pub_);
    // DEBUG_PRINT("\r\n [Publisher Create]   /joint_states   : "); DEBUG_PRINT((joint_states_pub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    // // (Not necessary) Battey state of Turtlebot3 
    // battery_state_pub_ = this->createPublisher<sensor_msgs::BatteryState>("battery_state");
    // this->createWallFreq(BATTERY_STATE_PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishBatteryState, NULL, battery_state_pub_);
    // DEBUG_PRINT("\r\n [Publisher Create]   /battery_state  : "); DEBUG_PRINT((battery_state_pub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    // // (Not necessary) Magnetic field
    // mag_pub_           = this->createPublisher<sensor_msgs::MagneticField>("magnetic_field");
    // this->createWallFreq(MAGNETIC_FIELD_PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishMagneticField, NULL, mag_pub_);
    // DEBUG_PRINT("\r\n [Publisher Create]   /magnetic_field : "); DEBUG_PRINT((mag_pub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    // Version information of Turtlebot3
    version_info_pub_  = this->createPublisher<turtlebot3_msgs::VersionInfo>("version_info");
    this->createWallFreq(VERSION_INFORMATION_PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishVersionInfo, NULL, version_info_pub_);
    DEBUG_PRINT("\r\n [Publisher Create]   /version_info   : "); DEBUG_PRINT((version_info_pub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    // // (Not necessary) Command velocity of Turtlebot3 using RC100 remote controller
    // cmd_vel_rc100_pub_ = this->createPublisher<geometry_msgs::Twist>("cmd_vel_rc100");
    // this->createWallFreq(CMD_VEL_PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishCmdVelRC100, NULL, cmd_vel_rc100_pub_);
    // DEBUG_PRINT("\r\n [Publisher Create]   /cmd_vel_rc100  : "); DEBUG_PRINT((cmd_vel_rc100_pub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);
   

    /*******************************************************************************
    * Subscriber
    *******************************************************************************/
    cmd_vel_sub_       = this->createSubscriber<geometry_msgs::Twist>("cmd_vel", (ros2::CallbackFunc)subscribeCmdVel, NULL);
    DEBUG_PRINT("\r\n [Subscriber Create]  /cmd_vel        : "); DEBUG_PRINT((cmd_vel_sub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    motor_power_sub_   = this->createSubscriber<std_msgs::Bool>("motor_power", (ros2::CallbackFunc)subscribeMotorPower, NULL);
    DEBUG_PRINT("\r\n [Subscriber Create]  /motor_power    : "); DEBUG_PRINT((motor_power_sub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    reset_sub_         = this->createSubscriber<std_msgs::Empty>("reset", (ros2::CallbackFunc)subscribeReset, NULL);
    DEBUG_PRINT("\r\n [Subscriber Create]  /reset          : "); DEBUG_PRINT((reset_sub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    sound_sub_         = this->createSubscriber<turtlebot3_msgs::Sound>("sound", (ros2::CallbackFunc)subscribeSound, NULL);
    DEBUG_PRINT("\r\n [Subscriber Create]  /sound          : "); DEBUG_PRINT((sound_sub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    time_sync_sub_     = this->createSubscriber<builtin_interfaces::Time>("time_sync", (ros2::CallbackFunc)subscribeTimeSync, NULL);
    DEBUG_PRINT("\r\n [Subscriber Create]  /time_sync      : "); DEBUG_PRINT((time_sync_sub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);
  }


private:

  /* Publisher Pointer */
  //ros2::Publisher<nav_msgs::Odometry>*            odom_pub_;
  ros2::Publisher<turtlebot3_msgs::SensorState>*  sensor_state_pub_;
  ros2::Publisher<sensor_msgs::Imu>*              imu_pub_;
  // ros2::Publisher<sensor_msgs::JointState>*       joint_states_pub_;
  ros2::Publisher<turtlebot3_msgs::VersionInfo>*  version_info_pub_;
  
  //ros2::Publisher<sensor_msgs::BatteryState>*     battery_state_pub_;
  //ros2::Publisher<geometry_msgs::Twist>*          cmd_vel_rc100_pub_;
  //ros2::Publisher<sensor_msgs::MagneticField>*    mag_pub_;

  /* Subscriber Pointer */
  ros2::Subscriber<geometry_msgs::Twist>*         cmd_vel_sub_;
  ros2::Subscriber<turtlebot3_msgs::Sound>*       sound_sub_;
  ros2::Subscriber<std_msgs::Bool>*               motor_power_sub_;
  ros2::Subscriber<std_msgs::Empty>*              reset_sub_;
  ros2::Subscriber<builtin_interfaces::Time>*     time_sync_sub_;
};



#endif // TURTLEBOT3_CORE_CONFIG_H_

