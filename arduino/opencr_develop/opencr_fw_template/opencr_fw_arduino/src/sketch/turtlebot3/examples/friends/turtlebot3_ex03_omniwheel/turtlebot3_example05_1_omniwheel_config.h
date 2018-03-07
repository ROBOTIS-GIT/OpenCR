#ifndef TURTLEBOT3_EXAMPLE05_1_OMNIWHEELWHEEL_CONFIG_H
#define TURTLEBOT3_EXAMPLE05_1_OMNIWHEELWHEEL_CONFIG_H

// Control table address (XM430-W350-T)
#define ADDR_XM_OPERATING_MODE           11
#define ADDR_XM_VELOCITY_LIMIT           44
#define ADDR_XM_TORQUE_ENABLE            64
#define ADDR_XM_GOAL_VELOCITY           104
#define ADDR_XM_GOAL_POSITION           116
#define ADDR_XM_REALTIME_TICK           120
#define ADDR_XM_PRESENT_VELOCITY        128
#define ADDR_XM_PRESENT_POSITION        132

// Data Byte Length
#define LEN_XM_OPERATING_MODE           1
#define LEN_XM_VELOCITY_LIMIT           4
#define LEN_XM_TORQUE_ENABLE            1
#define LEN_XM_GOAL_VELOCITY            4
#define LEN_XM_GOAL_POSITION            4
#define LEN_XM_REALTIME_TICK            2
#define LEN_XM_PRESENT_VELOCITY         4
#define LEN_XM_PRESENT_POSITION         4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define OMNIWHEEL_NUM                   3
#define BAUDRATE                        1000000
#define DEVICENAME                      ""                  // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define CONTROL_PEROID                  8000

#define VELOCITY_LINEAR_X               0.05
#define VELOCITY_LINEAR_Y               0.05
#define VELOCITY_ANGULAR_Z              0.05

#define LIMIT_XM_MAX_VELOCITY           480

#define DISTANCE_CENTER_TO_WHEEL        0.165
#define WHEEL_SEPERATION_ANGLE          60                  //degree
#define WHEEL_RADIUS                    0.03
#define VELOCITY_CONSTANT_VAULE         0.024               //goal_velocity RPM = 0.229, rad/sec = 0.024

#define DEGREE2RADIAN                   (M_PI / 180.0)
#define RADIAN2DEGREE                   (180.0 / M_PI)

#endif // TURTLEBOT3_EXAMPLE05_1_OMNIWHEELWHEEL_CONFIG_H

// Ref : http://www.revistas.unal.edu.co/index.php/ingeinv/article/view/47763/52384
