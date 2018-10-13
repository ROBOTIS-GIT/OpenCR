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

/* Authors: Taehun Lim (Darby) */

#include "../../include/dynamixel_workbench_toolbox/dynamixel_item.h"
static uint8_t the_number_of_item = 0;


static ModelInfo model_info = {0.0, 0.0, 0, 0, 0, 0.0, 0.0};

//=========================================================
// Servo register definitions
//=========================================================

//---------------------------------------------------------
// AX servos - (num == AX_12A || num == AX_12W || num == AX_18A)
//---------------------------------------------------------
static const ControlTableItem items_AX[] {
    {0  , "Model_Number"                  , 2},
    {2  , "Firmware_Version"              , 1},
    {3  , "ID"                            , 1},
    {4  , "Baud_Rate"                     , 1},
    {5  , "Return_Delay_Time"             , 1},
    {6  , "CW_Angle_Limit"                , 2},
    {8  , "CCW_Angle_Limit"               , 2},
    {11 , "Temperature_Limit"             , 1},
    {12 , "Min_Voltage_Limit"             , 1},
    {13 , "Max_Voltage_Limit"             , 1},
    {14 , "Max_Torque"                    , 2},
    {16 , "Status_Return_Level"           , 1},
    {17 , "Alarm_LED"                     , 1},
    {18 , "Shutdown"                      , 1},

    {24 , "Torque_Enable"                 , 1},
    {25 , "LED"                           , 1},
    {26 , "CW_Compliance_Margin"          , 1},
    {27 , "CCW_Compliance_Margin"         , 1},
    {28 , "CW_Compliance_Slope"           , 1},
    {29 , "CCW_Compliance_Slope"          , 1},
    {30 , "Goal_Position"                 , 2},
    {32 , "Moving_Speed"                  , 2},
    {34 , "Torque_Limit"                  , 2},
    {36 , "Present_Position"              , 2},
    {38 , "Present_Speed"                 , 2},
    {40 , "Present_Load"                  , 2},
    {42 , "Present_Voltage"               , 1},
    {43 , "Present_Temperature"           , 1},
    {44 , "Registered"                    , 1},
    {46 , "Moving"                        , 1},
    {47 , "Lock"                          , 1},
    {48 , "Punch"                         , 2} };
#define COUNT_AX_ITEMS (sizeof(items_AX)/sizeof(items_AX[0]))

static void setAXInfo()
{
  model_info.velocity_to_value_ratio         = 86.03; // AX series don't support exact speed in wheel mode.
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 512;
  model_info.value_of_max_radian_position    = 1024;

  model_info.min_radian                      = -2.61799;
  model_info.max_radian                      =  2.61799;
}

//---------------------------------------------------------
// RX servos - (num == RX_10 || num == RX_24F || num == RX_28 || num == RX_64)
//---------------------------------------------------------
static const ControlTableItem items_RX[] {
    {0  , "Model_Number"                  , 2},
    {2  , "Firmware_Version"              , 1},
    {3  , "ID"                            , 1},
    {4  , "Baud_Rate"                     , 1},
    {5  , "Return_Delay_Time"             , 1},
    {6  , "CW_Angle_Limit"                , 2},
    {8  , "CCW_Angle_Limit"               , 2},
    {11 , "Temperature_Limit"             , 1},
    {12 , "Min_Voltage_Limit"             , 1},
    {13 , "Max_Voltage_Limit"             , 1},
    {14 , "Max_Torque"                    , 2},
    {16 , "Status_Return_Level"           , 1},
    {17 , "Alarm_LED"                     , 1},
    {18 , "Shutdown"                      , 1},

    {24 , "Torque_Enable"                 , 1},
    {25 , "LED"                           , 1},
    {26 , "CW_Compliance_Margin"          , 1},
    {27 , "CCW_Compliance_Margin"         , 1},
    {28 , "CW_Compliance_Slope"           , 1},
    {29 , "CCW_Compliance_Slope"          , 1},
    {30 , "Goal_Position"                 , 2},
    {32 , "Moving_Speed"                  , 2},
    {34 , "Torque_Limit"                  , 2},
    {36 , "Present_Position"              , 2},
    {38 , "Present_Speed"                 , 2},
    {40 , "Present_Load"                  , 2},
    {42 , "Present_Voltage"               , 1},
    {43 , "Present_Temperature"           , 1},
    {44 , "Registered"                    , 1},
    {46 , "Moving"                        , 1},
    {47 , "Lock"                          , 1},
    {48 , "Punch"                         , 2} };

#define COUNT_RX_ITEMS (sizeof(items_RX)/sizeof(items_RX[0]))

static void setRXInfo(void)
{
  model_info.velocity_to_value_ratio         = 86.03; // RX series don't support exact speed in wheel mode.
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 512;
  model_info.value_of_max_radian_position    = 1024;

  model_info.min_radian                      = -2.61799;
  model_info.max_radian                      =  2.61799;
}

//---------------------------------------------------------
// EX servos - (num == EX_106)
//---------------------------------------------------------
static const ControlTableItem items_EX[] {
    {0  , "Model_Number"                  , 2},
    {2  , "Firmware_Version"              , 1},
    {3  , "ID"                            , 1},
    {4  , "Baud_Rate"                     , 1},
    {5  , "Return_Delay_Time"             , 1},
    {6  , "CW_Angle_Limit"                , 2},
    {8  , "CCW_Angle_Limit"               , 2},
    {10 , "Drive_Mode"                    , 1},
    {11 , "Temperature_Limit"             , 1},
    {12 , "Min_Voltage_Limit"             , 1},
    {13 , "Max_Voltage_Limit"             , 1},
    {14 , "Max_Torque"                    , 2},
    {16 , "Status_Return_Level"           , 1},
    {17 , "Alarm_LED"                     , 1},
    {18 , "Shutdown"                      , 1},

    {24 , "Torque_Enable"                 , 1},
    {25 , "LED"                           , 1},
    {26 , "CW_Compliance_Margin"          , 1},
    {27 , "CCW_Compliance_Margin"         , 1},
    {28 , "CW_Compliance_Slope"           , 1},
    {29 , "CCW_Compliance_Slope"          , 1},
    {30 , "Goal_Position"                 , 2},
    {34 , "Moving_Speed"                  , 2},
    {35 , "Torque_Limit"                  , 2},
    {36 , "Present_Position"              , 2},
    {38 , "Present_Speed"                 , 2},
    {40 , "Present_Load"                  , 2},
    {42 , "Present_Voltage"               , 1},
    {43 , "Present_Temperature"           , 1},
    {44 , "Registered"                    , 1},
    {46 , "Moving"                        , 1},
    {47 , "Lock"                          , 1},
    {48 , "Punch"                         , 2},
    {56 , "Sensored_Current"              , 2} };

#define COUNT_EX_ITEMS (sizeof(items_EX)/sizeof(items_EX[0]))

static void setEXInfo()
{
  model_info.velocity_to_value_ratio         = 86.03; // EX series don't support exact speed in wheel mode.
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 2048; 
  model_info.value_of_max_radian_position    = 4096;

  model_info.min_radian                      = -2.18969008;
  model_info.max_radian                      =  2.18969008;
}

//---------------------------------------------------------
// MX Protocol 1 servos - (num == MX_12W || num == MX_28)
//---------------------------------------------------------
static const ControlTableItem items_MX[] {
    {0  , "Model_Number"                  , 2},
    {2  , "Firmware_Version"              , 1},
    {3  , "ID"                            , 1},
    {4  , "Baud_Rate"                     , 1},
    {5  , "Return_Delay_Time"             , 1},
    {6  , "CW_Angle_Limit"                , 2},
    {8  , "CCW_Angle_Limit"               , 2},
    {11 , "Temperature_Limit"             , 1},
    {12 , "Min_Voltage_Limit"             , 1},
    {13 , "Max_Voltage_Limit"             , 1},
    {14 , "Max_Torque"                    , 2},
    {16 , "Status_Return_Level"           , 1},
    {17 , "Alarm_LED"                     , 1},
    {18 , "Shutdown"                      , 1},
    {20 , "Multi_Turn_Offset"             , 2},
    {22 , "Resolution_Divider"            , 1},

    {24 , "Torque_Enable"                 , 1},
    {25 , "LED"                           , 1},
    {26 , "D_gain"                        , 1},
    {27 , "I_gain"                        , 1},
    {28 , "P_gain"                        , 1},
    {30 , "Goal_Position"                 , 2},
    {32 , "Moving_Speed"                  , 2},
    {34 , "Torque_Limit"                  , 2},
    {36 , "Present_Position"              , 2},
    {38 , "Present_Speed"                 , 2},
    {40 , "Present_Load"                  , 2},
    {42 , "Present_Voltage"               , 1},
    {43 , "Present_Temperature"           , 1},
    {44 , "Registered"                    , 1},
    {46 , "Moving"                        , 1},
    {47 , "Lock"                          , 1},
    {48 , "Punch"                         , 2},
    {73 , "Goal_Acceleration"             , 1} };

#define COUNT_MX_ITEMS (sizeof(items_MX)/sizeof(items_MX[0]))

static void setMXInfo()
{
  model_info.velocity_to_value_ratio         = 86.81;
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 2048;  
  model_info.value_of_max_radian_position    = 4096;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

//---------------------------------------------------------
// MX Protocol 2 servos - (num == MX_28_2)
//---------------------------------------------------------
static const ControlTableItem items_MX2[] {
      {0  , "Model_Number"          , 2},
      {6  , "Firmware_Version"      , 1},
      {7  , "ID"                    , 1},
      {8  , "Baud_Rate"             , 1},
      {9  , "Return_Delay_Time"     , 1},
      {10 , "Drive_Mode"            , 1},
      {11 , "Operating_Mode"        , 1},
      {12 , "Secondary_ID"          , 1},
      {13 , "Protocol_Version"      , 1},
      {20 , "Homing_Offset"         , 4},
      {24 , "Moving_Threshold"      , 4},
      {31 , "Temperature_Limit"     , 1},
      {32 , "Max_Voltage_Limit"     , 2},
      {34 , "Min_Voltage_Limit"     , 2},
      {36 , "PWM_Limit"             , 2},
      {40 , "Acceleration_Limit"    , 4},
      {44 , "Velocity_Limit"        , 4},
      {48 , "Max_Position_Limit"    , 4},
      {52 , "Min_Position_Limit"    , 4},
      {63 , "Shutdown"              , 1},

      {64 , "Torque_Enable"         , 1},
      {65 , "LED"                   , 1},
      {68 , "Status_Return_Level"   , 1},
      {69 , "Registered_Instruction", 1},
      {70 , "Hardware_Error_Status" , 1},
      {76 , "Velocity_I_Gain"       , 2},
      {78 , "Velocity_P_Gain"       , 2},
      {80 , "Position_D_Gain"       , 2},
      {82 , "Position_I_Gain"       , 2},
      {84 , "Position_P_Gain"       , 2},
      {88 , "Feedforward_2nd_Gain"  , 2},
      {90 , "Feedforward_1st_Gain"  , 2},
      {98 , "Bus_Watchdog"          , 1},
      {100, "Goal_PWM"              , 2},
      {104, "Goal_Velocity"         , 4},
      {108, "Profile_Acceleration"  , 4},
      {112, "Profile_Velocity"      , 4},
      {116, "Goal_Position"         , 4},
      {120, "Realtime_Tick"         , 2},
      {122, "Moving"                , 1},
      {123, "Moving_Status"         , 1},
      {124, "Present_PWM"           , 2},
      {126, "Present_Load"          , 2},
      {128, "Present_Velocity"      , 4},
      {132, "Present_Position"      , 4},
      {136, "Velocity_Trajectory"   , 4},
      {140, "Position_Trajectory"   , 4},
      {144, "Present_Input_Voltage" , 2},
      {146, "Present_Temperature"   , 1} };

#define COUNT_MX2_ITEMS (sizeof(items_MX2)/sizeof(items_MX2[0]))


static void setMX2Info(void)
{
  model_info.velocity_to_value_ratio         = 41.70;
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 2048;  
  model_info.value_of_max_radian_position    = 4096;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

//---------------------------------------------------------
// EXT MX Protocol 1 servos - (num == MX_64 || num == MX_106)
//---------------------------------------------------------
static const ControlTableItem items_EXTMX[] {
    {0  , "Model_Number"                  , 2},
    {2  , "Firmware_Version"              , 1},
    {3  , "ID"                            , 1},
    {4  , "Baud_Rate"                     , 1},
    {5  , "Return_Delay_Time"             , 1},
    {6  , "CW_Angle_Limit"                , 2},
    {8  , "CCW_Angle_Limit"               , 2},
    {11 , "Temperature_Limit"             , 1},
    {12 , "Min_Voltage_Limit"             , 1},
    {13 , "Max_Voltage_Limit"             , 1},
    {14 , "Max_Torque"                    , 2},
    {16 , "Status_Return_Level"           , 1},
    {17 , "Alarm_LED"                     , 1},
    {18 , "Shutdown"                      , 1},
    {20 , "Multi_Turn_Offset"             , 2},
    {22 , "Resolution_Divider"            , 1},

    {24 , "Torque_Enable"                 , 1},
    {25 , "LED"                           , 1},
    {26 , "D_gain"                        , 1},
    {27 , "I_gain"                        , 1},
    {28 , "P_gain"                        , 1},
    {30 , "Goal_Position"                 , 2},
    {32 , "Moving_Speed"                  , 2},
    {34 , "Torque_Limit"                  , 2},
    {36 , "Present_Position"              , 2},
    {38 , "Present_Speed"                 , 2},
    {40 , "Present_Load"                  , 2},
    {42 , "Present_Voltage"               , 1},
    {43 , "Present_Temperature"           , 1},
    {44 , "Registered"                    , 1},
    {46 , "Moving"                        , 1},
    {47 , "Lock"                          , 1},
    {48 , "Punch"                         , 2},
    {68 , "Current"                       , 2},
    {70 , "Torque_Control_Mode_Enable"    , 1},
    {71 , "Goal_Torque"                   , 2},
    {73 , "Goal_Acceleration"             , 1} };

#define COUNT_EXTMX_ITEMS (sizeof(items_EXTMX)/sizeof(items_EXTMX[0]))

static void setExtMXInfo()
{
  model_info.velocity_to_value_ratio         = 86.81;
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 2048;  
  model_info.value_of_max_radian_position    = 4096;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

//---------------------------------------------------------
// EXT MX Protocol 2 Servos - (num == MX_64_2 || num == MX_106_2)
//---------------------------------------------------------
static const ControlTableItem items_EXTMX2[] {
    {0  , "Model_Number"          , 2},
    {6  , "Firmware_Version"      , 1},
    {7  , "ID"                    , 1},
    {8  , "Baud_Rate"             , 1},
    {9  , "Return_Delay_Time"     , 1},
    {10 , "Drive_Mode"            , 1},
    {11 , "Operating_Mode"        , 1},
    {12 , "Secondary_ID"          , 1},
    {13 , "Protocol_Version"      , 1},
    {20 , "Homing_Offset"         , 4},
    {24 , "Moving_Threshold"      , 4},
    {31 , "Temperature_Limit"     , 1},
    {32 , "Max_Voltage_Limit"     , 2},
    {34 , "Min_Voltage_Limit"     , 2},
    {36 , "PWM_Limit"             , 2},
    {40 , "Current_Limit"         , 2},
    {40 , "Acceleration_Limit"    , 4},
    {44 , "Velocity_Limit"        , 4},
    {48 , "Max_Position_Limit"    , 4},
    {52 , "Min_Position_Limit"    , 4},
    {63 , "Shutdown"              , 1},

    {64 , "Torque_Enable"         , 1},
    {65 , "LED"                   , 1},
    {68 , "Status_Return_Level"   , 1},
    {69 , "Registered_Instruction", 1},
    {70 , "Hardware_Error_Status" , 1},
    {76 , "Velocity_I_Gain"       , 2},
    {78 , "Velocity_P_Gain"       , 2},
    {80 , "Position_D_Gain"       , 2},
    {82 , "Position_I_Gain"       , 2},
    {84 , "Position_P_Gain"       , 2},
    {88 , "Feedforward_2nd_Gain"  , 2},
    {90 , "Feedforward_1st_Gain"  , 2},
    {98 , "Bus_Watchdog"          , 1},
    {100, "Goal_PWM"              , 2},
    {102, "Goal_Current"          , 2},
    {104, "Goal_Velocity"         , 4},
    {108, "Profile_Acceleration"  , 4},
    {112, "Profile_Velocity"      , 4},
    {116, "Goal_Position"         , 4},
    {120, "Realtime_Tick"         , 2},
    {122, "Moving"                , 1},
    {123, "Moving_Status"         , 1},
    {124, "Present_PWM"           , 2},
    {126, "Present_Current"       , 2},
    {128, "Present_Velocity"      , 4},
    {132, "Present_Position"      , 4},
    {136, "Velocity_Trajectory"   , 4},
    {140, "Position_Trajectory"   , 4},
    {144, "Present_Input Voltage" , 2},
    {146, "Present_Temperature"   , 1}};

#define COUNT_EXTMX2_ITEMS (sizeof(items_EXTMX2)/sizeof(items_EXTMX2[0]))

static void setExtMX2Info(void)
{
  model_info.velocity_to_value_ratio         = 41.70;
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 2048;  
  model_info.value_of_max_radian_position    = 4096;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

//---------------------------------------------------------
// XL320 - (num == XL_320)
//---------------------------------------------------------
static const ControlTableItem items_XL320[] {
    {0  , "Model_Number"                  , 2},
    {2  , "Firmware_Version"              , 1},
    {3  , "ID"                            , 1},
    {4  , "Baud_Rate"                     , 1},
    {5  , "Return_Delay_Time"             , 1},
    {6  , "CW_Angle_Limit"                , 2},
    {8  , "CCW_Angle_Limit"               , 2},
    {11 , "Control_Mode"                  , 1},
    {12 , "Temperature_Limit"             , 1},
    {13 , "Min_Voltage_Limit"             , 1},
    {14 , "Max_Voltage_Limit"             , 1},
    {15 , "Max_Torque"                    , 2},
    {17 , "Status_Return_Level"           , 1},
    {18 , "Shutdown"                      , 1},

    {24 , "Torque_Enable"                 , 1},
    {25 , "LED"                           , 1},
    {27 , "D_gain"                        , 1},
    {28 , "I_gain"                        , 1},
    {29 , "P_gain"                        , 1},
    {30 , "Goal_Position"                 , 2},
    {32 , "Moving_Speed"                  , 2},
    {34 , "Torque_Limit"                  , 2},
    {37 , "Present_Position"              , 2},
    {39 , "Present_Speed"                 , 2},
    {41 , "Present_Load"                  , 2},
    {45 , "Present_Voltage"               , 1},
    {46 , "Present_Temperature"           , 1},
    {47 , "Registered"                    , 1},
    {49 , "Moving"                        , 1},
    {50 , "Hardware_Error_Status"         , 1},
    {51 , "Punch"                         , 2} };

#define COUNT_XL320_ITEMS (sizeof(items_XL320)/sizeof(items_XL320[0]))

static void setXL320Info()
{
  model_info.velocity_to_value_ratio         = 86.03; // XL320 don't support exact speed in wheel mode.
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 512;  
  model_info.value_of_max_radian_position    = 1024;

  model_info.min_radian                      = -2.61799;
  model_info.max_radian                      =  2.61799;
}

//---------------------------------------------------------
// XL - (num == XL430_W250)
//---------------------------------------------------------
static const ControlTableItem items_XL[] {
    {0  , "Model_Number"          , 2},
    {6  , "Firmware_Version"      , 1},
    {7  , "ID"                    , 1},
    {8  , "Baud_Rate"             , 1},
    {9  , "Return_Delay_Time"     , 1},
    {10 , "Drive_Mode"            , 1},
    {11 , "Operating_Mode"        , 1},
    {12 , "Secondary_ID"          , 1},
    {13 , "Protocol_Version"      , 1},
    {20 , "Homing_Offset"         , 4},
    {24 , "Moving_Threshold"      , 4},
    {31 , "Temperature_Limit"     , 1},
    {32 , "Max_Voltage_Limit"     , 2},
    {34 , "Min_Voltage_Limit"     , 2},
    {36 , "PWM_Limit"             , 2},
    {40 , "Acceleration_Limit"    , 4},
    {44 , "Velocity_Limit"        , 4},
    {48 , "Max_Position_Limit"    , 4},
    {52 , "Min_Position_Limit"    , 4},
    {63 , "Shutdown"              , 1},

    {64 , "Torque_Enable"         , 1},
    {65 , "LED"                   , 1},
    {68 , "Status_Return_Level"   , 1},
    {69 , "Registered_Instruction", 1},
    {70 , "Hardware_Error_Status" , 1},
    {76 , "Velocity_I_Gain"       , 2},
    {78 , "Velocity_P_Gain"       , 2},
    {80 , "Position_D_Gain"       , 2},
    {82 , "Position_I_Gain"       , 2},
    {84 , "Position_P_Gain"       , 2},
    {88 , "Feedforward_2nd_Gain"  , 2},
    {90 , "Feedforward_1st_Gain"  , 2},
    {98 , "Bus_Watchdog"          , 1},
    {100, "Goal_PWM"              , 2},
    {104, "Goal_Velocity"         , 4},
    {108, "Profile_Acceleration"  , 4},
    {112, "Profile_Velocity"      , 4},
    {116, "Goal_Position"         , 4},
    {120, "Realtime_Tick"         , 2},
    {122, "Moving"                , 1},
    {123, "Moving_Status"         , 1},
    {124, "Present_PWM"           , 2},
    {126, "Present_Load"          , 2},
    {128, "Present_Velocity"      , 4},
    {132, "Present_Position"      , 4},
    {136, "Velocity_Trajectory"   , 4},
    {140, "Position_Trajectory"   , 4},
    {144, "Present_Input_Voltage" , 2},
    {146, "Present_Temperature"   , 1} };

#define COUNT_XL_ITEMS (sizeof(items_XL)/sizeof(items_XL[0]))

static void setXLInfo()
{
  model_info.velocity_to_value_ratio         = 41.70;

  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 2048;  
  model_info.value_of_max_radian_position    = 4096;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

//---------------------------------------------------------
// XM - (num == XM430_W210 || num == XM430_W350)
//---------------------------------------------------------
static const ControlTableItem items_XM[] {
    {0  , "Model_Number"          , 2},
    {6  , "Firmware_Version"      , 1},
    {7  , "ID"                    , 1},
    {8  , "Baud_Rate"             , 1},
    {9  , "Return_Delay_Time"     , 1},
    {10 , "Drive_Mode"            , 1},
    {11 , "Operating_Mode"        , 1},
    {12 , "Secondary_ID"          , 1},
    {13 , "Protocol_Version"      , 1},
    {20 , "Homing_Offset"         , 4},
    {24 , "Moving_Threshold"      , 4},
    {31 , "Temperature_Limit"     , 1},
    {32 , "Max_Voltage_Limit"     , 2},
    {34 , "Min_Voltage_Limit"     , 2},
    {36 , "PWM_Limit"             , 2},
    {40 , "Current_Limit"         , 2},
    {40 , "Acceleration_Limit"    , 4},
    {44 , "Velocity_Limit"        , 4},
    {48 , "Max_Position_Limit"    , 4},
    {52 , "Min_Position_Limit"    , 4},
    {63 , "Shutdown"              , 1},

    {64 , "Torque_Enable"         , 1},
    {65 , "LED"                   , 1},
    {68 , "Status_Return_Level"   , 1},
    {69 , "Registered_Instruction", 1},
    {70 , "Hardware_Error_Status" , 1},
    {76 , "Velocity_I_Gain"       , 2},
    {78 , "Velocity_P_Gain"       , 2},
    {80 , "Position_D_Gain"       , 2},
    {82 , "Position_I_Gain"       , 2},
    {84 , "Position_P_Gain"       , 2},
    {88 , "Feedforward_2nd_Gain"  , 2},
    {90 , "Feedforward_1st_Gain"  , 2},
    {98 , "Bus_Watchdog"          , 1},
    {100, "Goal_PWM"              , 2},
    {102, "Goal_Current"          , 2},
    {104, "Goal_Velocity"         , 4},
    {108, "Profile_Acceleration"  , 4},
    {112, "Profile_Velocity"      , 4},
    {116, "Goal_Position"         , 4},
    {120, "Realtime_Tick"         , 2},
    {122, "Moving"                , 1},
    {123, "Moving_Status"         , 1},
    {124, "Present_PWM"           , 2},
    {126, "Present_Current"       , 2},
    {128, "Present_Velocity"      , 4},
    {132, "Present_Position"      , 4},
    {136, "Velocity_Trajectory"   , 4},
    {140, "Position_Trajectory"   , 4},
    {144, "Present_Input_Voltage" , 2},
    {146, "Present_Temperature"   , 1} };

#define COUNT_XM_ITEMS (sizeof(items_XM)/sizeof(items_XM[0]))

static void setXMInfo()
{
  model_info.velocity_to_value_ratio       = 41.70;
  model_info.torque_to_current_value_ratio = 149.795386991;

  model_info.value_of_min_radian_position  = 0;
  model_info.value_of_0_radian_position    = 2048;
  model_info.value_of_max_radian_position  = 4096;

  model_info.min_radian = -3.14159265;
  model_info.max_radian =  3.14159265;
}

//---------------------------------------------------------
// EXTXM - (num == XM540_W150 || num == XM540_W270)
//---------------------------------------------------------
static const ControlTableItem items_EXTXM[] {
    {0  , "Model_Number"          , 2},
    {6  , "Firmware_Version"      , 1},
    {7  , "ID"                    , 1},
    {8  , "Baud_Rate"             , 1},
    {9  , "Return_Delay_Time"     , 1},
    {10 , "Drive_Mode"            , 1},
    {11 , "Operating_Mode"        , 1},
    {12 , "Secondary_ID"          , 1},
    {13 , "Protocol_Version"      , 1},
    {20 , "Homing_Offset"         , 4},
    {24 , "Moving_Threshold"      , 4},
    {31 , "Temperature_Limit"     , 1},
    {32 , "Max Voltage_Limit"     , 2},
    {34 , "Min Voltage_Limit"     , 2},
    {36 , "PWM_Limit"             , 2},
    {40 , "Current_Limit"         , 2},
    {40 , "Acceleration_Limit"    , 4},
    {44 , "Velocity_Limit"        , 4},
    {48 , "Max_Position_Limit"    , 4},
    {52 , "Min_Position_Limit"    , 4},
    {56 , "External_Port_Mode_1"  , 1},
    {57 , "External_Port_Mode_2"  , 1},
    {58 , "External_Port_Mode_3"  , 1},
    {63 , "Shutdown"              , 1},

    {64 , "Torque_Enable"         , 1},
    {65 , "LED"                   , 1},
    {68 , "Status_Return_Level"   , 1},
    {69 , "Registered_Instruction", 1},
    {70 , "Hardware_Error_Status" , 1},
    {76 , "Velocity_I_Gain"       , 2},
    {78 , "Velocity_P_Gain"       , 2},
    {80 , "Position_D_Gain"       , 2},
    {82 , "Position_I_Gain"       , 2},
    {84 , "Position_P_Gain"       , 2},
    {88 , "Feedforward_2nd_Gain"  , 2},
    {90 , "Feedforward_1st_Gain"  , 2},
    {98 , "Bus_Watchdog"          , 1},
    {100, "Goal_PWM"              , 2},
    {102, "Goal_Current"          , 2},
    {104, "Goal_Velocity"         , 4},
    {108, "Profile_Acceleration"  , 4},
    {112, "Profile_Velocity"      , 4},
    {116, "Goal_Position"         , 4},
    {120, "Realtime_Tick"         , 2},
    {122, "Moving"                , 1},
    {123, "Moving_Status"         , 1},
    {124, "Present_PWM"           , 2},
    {126, "Present_Current"       , 2},
    {128, "Present_Velocity"      , 4},
    {132, "Present_Position"      , 4},
    {136, "Velocity_Trajectory"   , 4},
    {140, "Position_Trajectory"   , 4},
    {144, "Present_Input_Voltage" , 2},
    {146, "Present_Temperature"   , 1} };

#define COUNT_EXTXM_ITEMS (sizeof(items_EXTXM)/sizeof(items_EXTXM[0]))

static void setExtXMInfo(void)
{
  model_info.velocity_to_value_ratio       = 41.70;

  model_info.value_of_min_radian_position  = 0;
  model_info.value_of_0_radian_position    = 2048;
  model_info.value_of_max_radian_position  = 4096;

  model_info.min_radian = -3.14159265;
  model_info.max_radian =  3.14159265;
}

//---------------------------------------------------------
// XH - (num == XH430_V210 || num == XH430_V350 || num == XH430_W210 || num == XH430_W350)
//---------------------------------------------------------
static const ControlTableItem items_XH[] {
    {0  , "Model_Number"          , 2},
    {6  , "Firmware_Version"      , 1},
    {7  , "ID"                    , 1},
    {8  , "Baud_Rate"             , 1},
    {9  , "Return_Delay_Time"     , 1},
    {10 , "Drive_Mode"            , 1},
    {11 , "Operating_Mode"        , 1},
    {12 , "Secondary_ID"          , 1},
    {13 , "Protocol_Version"      , 1},
    {20 , "Homing_Offset"         , 4},
    {24 , "Moving_Threshold"      , 4},
    {31 , "Temperature_Limit"     , 1},
    {32 , "Max_Voltage_Limit"     , 2},
    {34 , "Min_Voltage_Limit"     , 2},
    {36 , "PWM_Limit"             , 2},
    {40 , "Current_Limit"         , 2},
    {40 , "Acceleration_Limit"    , 4},
    {44 , "Velocity_Limit"        , 4},
    {48 , "Max_Position_Limit"    , 4},
    {52 , "Min_Position_Limit"    , 4},
    {63 , "Shutdown"              , 1},

    {64 , "Torque_Enable"         , 1},
    {65 , "LED"                   , 1},
    {68 , "Status_Return_Level"   , 1},
    {69 , "Registered_Instruction", 1},
    {70 , "Hardware_Error_Status" , 1},
    {76 , "Velocity_I_Gain"       , 2},
    {78 , "Velocity_P_Gain"       , 2},
    {80 , "Position_D_Gain"       , 2},
    {82 , "Position_I_Gain"       , 2},
    {84 , "Position_P_Gain"       , 2},
    {88 , "Feedforward_2nd_Gain"  , 2},
    {90 , "Feedforward_1st_Gain"  , 2},
    {98 , "Bus_Watchdog"          , 1},
    {100, "Goal_PWM"              , 2},
    {102, "Goal_Current"          , 2},
    {104, "Goal_Velocity"         , 4},
    {108, "Profile_Acceleration"  , 4},
    {112, "Profile_Velocity"      , 4},
    {116, "Goal_Position"         , 4},
    {120, "Realtime_Tick"         , 2},
    {122, "Moving"                , 1},
    {123, "Moving_Status"         , 1},
    {124, "Present_PWM"           , 2},
    {126, "Present_Current"       , 2},
    {128, "Present_Velocity"      , 4},
    {132, "Present_Position"      , 4},
    {136, "Velocity_Trajectory"   , 4},
    {140, "Position_Trajectory"   , 4},
    {144, "Present_Input_Voltage" , 2},
    {146, "Present_Temperature"   , 1} };

#define COUNT_XH_ITEMS (sizeof(items_XH)/sizeof(items_XH[0]))


static void setXHInfo()
{
  model_info.velocity_to_value_ratio       = 41.71;

  model_info.value_of_min_radian_position  = 0;
  model_info.value_of_0_radian_position    = 2048;
  model_info.value_of_max_radian_position  = 4096;

  model_info.min_radian = -3.14159265;
  model_info.max_radian = 3.14159265;
}

//---------------------------------------------------------
// PRO - (num == PRO_L42_10_S300_R)
//---------------------------------------------------------
static const ControlTableItem items_PRO[] {
    {0  , "Model_Number"          , 2},
    {6  , "Firmware_Version"      , 1},
    {7  , "ID"                    , 1},
    {8  , "Baud_Rate"             , 1},
    {9  , "Return_Delay_Time"     , 1},
    {11 , "Operating_Mode"        , 1},
    {17 , "Moving_Threshold"      , 4},
    {21 , "Temperature_Limit"     , 1},
    {22 , "Max_Voltage_Limit"     , 2},
    {24 , "Min_Voltage_Limit"     , 2},
    {26 , "Acceleration_Limit"    , 4},
    {30 , "Torque_Limit"          , 2},
    {32 , "Velocity_Limit"        , 4},
    {36 , "Max_Position_Limit"    , 4},
    {40 , "Min_Position_Limit"    , 4},
    {44 , "External_Port_Mode_1"  , 1},
    {45 , "External_Port_Mode_2"  , 1},
    {46 , "External_Port_Mode_3"  , 1},
    {47 , "External_Port_Mode_4"  , 1},
    {48 , "Shutdown"              , 1},

    {562, "Torque_Enable"         , 1},
    {563, "LED_RED"               , 1},
    {564, "LED_GREEN"             , 1},
    {565, "LED_BLUE"              , 1},
    {586, "Velocity_I_Gain"       , 2},
    {588, "Velocity_P_Gain"       , 2},
    {594, "Position_P_Gain"       , 2},
    {596, "Goal_Position"         , 4},
    {600, "Goal_Velocity"         , 4},
    {604, "Goal_Torque"           , 2},
    {606, "Goal_Acceleration"     , 4},
    {610, "Moving"                , 1},
    {611, "Present_Position"      , 4},
    {615, "Present_Velocity"      , 4},
    {621, "Present_Current"       , 2},
    {623, "Present_Input_Voltage" , 2},
    {625, "Present_Temperature"   , 1},
    {626, "External_Port_Mode_1"  , 2},
    {628, "External_Port_Mode_2"  , 2},
    {630, "External_Port_Mode_3"  , 2},
    {632, "External_Port_Mode_4"  , 2},
    {890, "Registered_Instruction", 1},
    {891, "Status_Return_Level"   , 1},
    {892, "Hardware_Error_Status" , 1} };

#define COUNT_PRO_ITEMS (sizeof(items_PRO)/sizeof(items_PRO[0]))

//---------------------------------------------------------
// EXT PRO - All Other Pros...
//---------------------------------------------------------
static const ControlTableItem items_EXTPRO[] {
    {0  , "Model_Number"          , 2},
    {6  , "Firmware_Version"      , 1},
    {7  , "ID"                    , 1},
    {8  , "Baud_Rate"             , 1},
    {9  , "Return_Delay_Time"     , 1},
    {11 , "Operating_Mode"        , 1},
    {13 , "Homing_Offset"         , 4},
    {17 , "Moving_Threshold"      , 4},
    {21 , "Temperature_Limit"     , 1},
    {22 , "Max_Voltage_Limit"     , 2},
    {24 , "Min_Voltage_Limit"     , 2},
    {26 , "Acceleration_Limit"    , 4},
    {30 , "Torque_Limit"          , 2},
    {32 , "Velocity_Limit"        , 4},
    {36 , "Max_Position_Limit"    , 4},
    {40 , "Min_Position_Limit"    , 4},
    {44 , "External_Port_Mode_1"  , 1},
    {45 , "External_Port_Mode_2"  , 1},
    {46 , "External_Port_Mode_3"  , 1},
    {47 , "External_Port_Mode_4"  , 1},
    {48 , "Shutdown"              , 1},

    {562, "Torque_Enable"         , 1},
    {563, "LED_RED"               , 1},
    {564, "LED_GREEN"             , 1},
    {565, "LED_BLUE"              , 1},
    {586, "Velocity_I_Gain"       , 2},
    {588, "Velocity_P_Gain"       , 2},
    {594, "Position_P_Gain"       , 2},
    {596, "Goal_Position"         , 4},
    {600, "Goal_Velocity"         , 4},
    {604, "Goal_Torque"           , 2},
    {606, "Goal_Acceleration"     , 4},
    {610, "Moving"                , 1},
    {611, "Present_Position"      , 4},
    {615, "Present_Velocity"      , 4},
    {621, "Present_Current"       , 2},
    {623, "Present_Input_Voltage" , 2},
    {625, "Present_Temperature"   , 1},
    {626, "External_Port_Mode_1"  , 2},
    {628, "External_Port_Mode_2"  , 2},
    {630, "External_Port_Mode_3"  , 2},
    {632, "External_Port_Mode_4"  , 2},
    {890, "Registered_Instruction", 1},
    {891, "Status_Return_Level"   , 1},
    {892, "Hardware_Error_Status" , 1} };

#define COUNT_EXTPRO_ITEMS (sizeof(items_EXTPRO)/sizeof(items_EXTPRO[0]))

static void setPROInfo(uint16_t model_number)
{
  model_info.velocity_to_value_ratio         = 4792.8;
  
  if (model_number == PRO_L42_10_S300_R)
  {
    model_info.value_of_min_radian_position    = 0;
    model_info.value_of_0_radian_position      = 2048;
    model_info.value_of_max_radian_position    = 4096;
  }
  else if (model_number == PRO_L54_30_S400_R)
  {
    model_info.value_of_min_radian_position    = -144197;
    model_info.value_of_0_radian_position      = 0;
    model_info.value_of_max_radian_position    = 144197;   
  }
  else if (model_number == PRO_L54_30_S500_R || model_number == PRO_L54_50_S500_R)
  {
    model_info.value_of_min_radian_position    = -180692;
    model_info.value_of_0_radian_position      = 0;
    model_info.value_of_max_radian_position    = 180692;   
  }
  else if (model_number == PRO_L54_50_S290_R)
  {
    model_info.value_of_min_radian_position    = -103846;
    model_info.value_of_0_radian_position      = 0;
    model_info.value_of_max_radian_position    = 103846;   
  }
  else if (model_number == PRO_M42_10_S260_R)
  {
    model_info.value_of_min_radian_position    = -131593;
    model_info.value_of_0_radian_position      = 0;
    model_info.value_of_max_radian_position    = 131593;   
  }
  else if (model_number == PRO_M54_40_S250_R || model_number == PRO_M54_60_S250_R)
  {
    model_info.value_of_min_radian_position    = -125708;
    model_info.value_of_0_radian_position      = 0;
    model_info.value_of_max_radian_position    = 125708;   
  }
  else if (model_number == PRO_H42_20_S300_R)
  {
    model_info.value_of_min_radian_position    = -151875;
    model_info.value_of_0_radian_position      = 0;
    model_info.value_of_max_radian_position    = 151875;   
  }
  else if (model_number == PRO_H54_100_S500_R || model_number == PRO_H54_200_S500_R)
  {
    model_info.value_of_min_radian_position    = -250961;
    model_info.value_of_0_radian_position      = 0;
    model_info.value_of_max_radian_position    = 250961;   
  }
  else
  {
    model_info.value_of_min_radian_position    = -250961;
    model_info.value_of_0_radian_position      = 0;
    model_info.value_of_max_radian_position    = 250961;       
  }

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}


//=========================================================
// Get Servo control table for the specified servo type
//=========================================================
const ControlTableItem* getConrolTableItem(uint16_t model_number)
{
  uint16_t num = model_number;

  const ControlTableItem  *items;

  if (num == AX_12A || num == AX_12W || num == AX_18A)
  {
    items = items_AX;
    the_number_of_item = COUNT_AX_ITEMS;
  }
  else if (num == RX_10 || num == RX_24F || num == RX_28 || num == RX_64)
  {
    items = items_RX;
    the_number_of_item = COUNT_RX_ITEMS;
  }
  else if (num == EX_106)
  {
    items = items_EX;
    the_number_of_item = COUNT_EX_ITEMS;
  }
  else if (num == MX_12W || num == MX_28)
  {
    items = items_MX;
    the_number_of_item = COUNT_MX_ITEMS;
  }
  else if (num == MX_64 || num == MX_106)
  {
    items = items_EXTMX;
    the_number_of_item = COUNT_EXTMX_ITEMS;
  }
  else if (num == MX_28_2)
  {
    items = items_MX2;
    the_number_of_item = COUNT_MX2_ITEMS;
  }
  else if (num == MX_64_2 || num == MX_106_2)
  {
    items = items_EXTMX2;
    the_number_of_item = COUNT_EXTMX2_ITEMS;
  }
  else if (num == XL_320)
  {
    items = items_XL320;
    the_number_of_item = COUNT_XL320_ITEMS;
  }
  else if (num == XL430_W250)
  {
    items = items_XL;
    the_number_of_item = COUNT_XL_ITEMS;
  }
  else if (num == XM430_W210 || num == XM430_W350)
  {
    items = items_XM;
    the_number_of_item = COUNT_XM_ITEMS;
  }
  else if (num == XM540_W150 || num == XM540_W270)
  {
    items = items_EXTXM;
    the_number_of_item = COUNT_EXTXM_ITEMS;
  }
  else if (num == XH430_V210 || num == XH430_V350 || num == XH430_W210 || num == XH430_W350)
  {
    items = items_XH;
    the_number_of_item = COUNT_XH_ITEMS;
  }
  else if (num == PRO_L54_30_S400_R || num == PRO_L54_30_S500_R  || num == PRO_L54_50_S290_R || num == PRO_L54_50_S500_R ||
           num == PRO_M42_10_S260_R || num == PRO_M54_40_S250_R  || num == PRO_M54_60_S250_R || 
           num == PRO_H42_20_S300_R || num == PRO_H54_100_S500_R || num == PRO_H54_200_S500_R)
  {
    items = items_EXTPRO;
    the_number_of_item = COUNT_EXTPRO_ITEMS;
  }
  else if (num == PRO_L42_10_S300_R)
  {
    items = items_PRO;
    the_number_of_item = COUNT_PRO_ITEMS;
  }
  else
  {
    // default to MX...
    items = items_MX;
    the_number_of_item = COUNT_MX_ITEMS;
  }

  return items;
}

ModelInfo* getModelInfo(uint16_t model_number)
{  
  uint16_t num = model_number;

  if (num == AX_12A || num == AX_12W || num == AX_18A)
  {
    setAXInfo();
  }
  else if (num == RX_10 || num == RX_24F || num == RX_28 || num == RX_64)
  {
    setRXInfo();
  }
  else if (num == EX_106)
  {
    setEXInfo();
  }
  else if (num == MX_12W || num == MX_28)
  {
    setMXInfo();
  }
  else if (num == MX_64 || num == MX_106)
  {
    setExtMXInfo();
  }
  else if (num == MX_28_2)
  {
    setMX2Info();
  }
  else if (num == MX_64_2 || num == MX_106_2)
  {
    setExtMX2Info();
  }
  else if (num == XL_320)
  {
    setXL320Info();
  }
  else if (num == XL430_W250)
  {
    setXLInfo();
  }
  else if (num == XM430_W210 || num == XM430_W350)
  {
    setXMInfo();
  }
  else if (num == XM540_W150 || num == XM540_W270)
  {
    setExtXMInfo();
  }
  else if (num == XH430_V210 || num == XH430_V350 || num == XH430_W210 || num == XH430_W350)
  {
    setXHInfo();
  }
  else if (num == PRO_L42_10_S300_R  || num == PRO_L54_30_S400_R || num == PRO_L54_30_S500_R || num == PRO_L54_50_S290_R || num == PRO_L54_50_S500_R  ||
           num == PRO_M42_10_S260_R  || num == PRO_M54_40_S250_R || num == PRO_M54_60_S250_R || num == PRO_H42_20_S300_R || num == PRO_H54_100_S500_R ||
           num == PRO_H54_200_S500_R)
  {
    setPROInfo(num);
  }
  else
  {
    setXMInfo();
  }

  return &model_info;
}

uint8_t getTheNumberOfControlItem()
{
  return the_number_of_item;
}
