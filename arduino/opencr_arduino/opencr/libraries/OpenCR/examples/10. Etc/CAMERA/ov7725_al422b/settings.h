#ifndef __SETTINGS_H
#define __SETTINGS_H

#define BOARD_LED_PIN           BDPIN_LED_STATUS    //Status LED
#define LED_RATE                500000              // in microseconds; should toggle every 0.5sec

#define _USE_FULLSCREEN         true                // true : use 320x240, false : use 160x120

#if _USE_FULLSCREEN == true
  #define MIN_OBJECT_PIXEL		5	//Set the object size to detect
#else
  #define MIN_OBJECT_PIXEL		3	//Set the object size to detect
#endif

#define MAX_OBJECT				  200	//maximum number of object to detect
#define SELECTED_COLOR      0   //0:Red, 1:Blue

//Dynamixel Definition
#define PROTOCOL_VERSION        2.0
#define BAUDRATE                1000000
#define DXL_PORT                "1"
#define LEFT_WHEEL              1
#define RIGHT_WHEEL             2
#define PAN_ID                  3
#define TILT_ID                 4

#define ADD_TORQUE_ENABLE       64
#define ADD_GOAL_POSITION       116
#define ADD_PRESENT_POSITION    132
#define ADD_PROF_ACCEL          112
#define ADD_PROF_VEL            108

#define TORQUE_ON               1
#define TORQUE_OFF              0
#define PAN_MIN_POSITION        1024    //+90
#define PAN_MAX_POSITION        3072    //-90
#define TILT_MIN_POSITION       1024    //+90
#define TILT_MAX_POSITION       3072    //-90
#define PROF_ACCEL              25
#define PROF_VEL                200
#define MOVING_THRESHOLD        5       //11.38 = 1degree
#define TRACK_THRESHOLD         50
#define MOVING_STEP             15

#define ESC_ASCII_VALUE         0x1b

extern  uint16_t image_buf[];
extern  uint16_t masked_image_buf[];

#endif