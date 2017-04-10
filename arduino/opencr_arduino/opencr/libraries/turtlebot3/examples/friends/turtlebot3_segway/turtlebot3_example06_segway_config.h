#ifndef TURTLEBOT3_EXAMPLE06_SEGWAY_CONFIG_H
#define TURTLEBOT3_EXAMPLE06_SEGWAY_CONFIG_H

#include <DynamixelSDK.h>
#include <IMU.h>
#include <RC100.h>
#include <Filters.h>

// Control table address (XM430-W350-T)
#define ADDR_XM_OPERATING_MODE           11
#define ADDR_XM_VELOCITY_LIMIT           44
#define ADDR_XM_TORQUE_ENABLE            64
#define ADDR_XM_GOAL_VELOCITY           104
#define ADDR_XM_GOAL_POSITION           116
#define ADDR_XM_GOAL_PWM                100
#define ADDR_XM_REALTIME_TICK           120
#define ADDR_XM_PRESENT_VELOCITY        128
#define ADDR_XM_PRESENT_POSITION        132
#define ADDR_XM_VELOCITY_P_GAIN         78

// Data Byte Length
#define LEN_XM_OPERATING_MODE           1
#define LEN_XM_VELOCITY_LIMIT           4
#define LEN_XM_TORQUE_ENABLE            1
#define LEN_XM_GOAL_VELOCITY            4
#define LEN_XM_GOAL_POSITION            4
#define LEN_XM_GOAL_PWM                 2
#define LEN_XM_REALTIME_TICK            2
#define LEN_XM_PRESENT_VELOCITY         4
#define LEN_XM_PRESENT_POSITION         4
#define LEN_XM_VELOCITY_P_GAIN          2

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_LEFT_ID                     1                   // ID: 1
#define DXL_RIGHT_ID                    2                   // ID: 2
#define BAUDRATE                        1000000
#define DEVICENAME                      ""                  // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
#define PWM_LIMIT                       885

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define CONTOL_PERIOD                   7000                // in microseconds
#define DEBUG_SERIAL                    Serial2

// #define DEBUG
// #define GRAPH
#define REMOTE

int dxl_comm_result = COMM_TX_FAIL;             // Communication result
bool dxl_addparam_result = false;               // addParam result
bool dxl_getdata_result = false;                // GetParam resulttorque ON

uint8_t dxl_error = 0;                          // Dynamixel error

float control_input = 0.0;
float control_output = 0.0;
float angle[3] = {0.0,0.0,0.0};                 //roll, pitch, yaw
float cur_error = 0.0, pre_error = 0.0, integral = 0.0, derivative = 0.0;
float diff_time = 0.007;

// PID gain
float p_gain = 4000.0;//4000.0;//1250.0;
float i_gain = 2.0;//2.0;//4.5;
float d_gain = 78.0;//78.0;//46.0;

// Low-Pass Filters
float filterFrequency = 0.5;  // Hz
FilterOnePole lowpassFilter(LOWPASS, filterFrequency);

float angle_offset = 0.0;  // initial angle offset
int16_t cnt = 0;           // timer counter
int RcvData = 0;           // receive data
char keyboard;             // keyboard input
int graph_max = 1500;
int graph_min = -1500;

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

// Initialize GroupSyncWrite instance
dynamixel::GroupSyncWrite *groupSyncWrite;
dynamixel::GroupSyncRead *groupSyncRead;

HardwareTimer Timer(TIMER_CH1);
cIMU IMU;
RC100 Controller;

void timerInit();
void imuInit();
void setTorque();
void getAngle(float angle[3]);
void setDynamixelPWM(int64_t wheel_value);
void handler_control();

#endif // TURTLEBOT3_EXAMPLE06_SEGWAY_CONFIG_H
