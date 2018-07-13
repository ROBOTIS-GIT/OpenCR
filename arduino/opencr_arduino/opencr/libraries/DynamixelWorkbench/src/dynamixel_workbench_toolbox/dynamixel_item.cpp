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

#if defined(__OPENCR__) || defined(__OPENCM904__)
static ControlTableItem item[15]  = {0, };
#else
static ControlTableItem item[60]  = {0, };
#endif

static ModelInfo model_info = {0.0, };

static void setAXItem(void)
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1};
  item[1]  = {4  , "Baud_Rate"                     , 1};
  item[2]  = {6  , "CW_Angle_Limit"                , 2};
  item[3]  = {8  , "CCW_Angle_Limit"               , 2};

  item[4]  = {24 , "Torque_Enable"                 , 1};
  item[5]  = {25 , "LED"                           , 1};
  item[6]  = {30 , "Goal_Position"                 , 2};
  item[7]  = {32 , "Moving_Speed"                  , 2};
  item[8]  = {34 , "Torque_Limit"                  , 2};
  item[9]  = {36 , "Present_Position"              , 2};
  item[10] = {38 , "Present_Speed"                 , 2};
  item[11] = {40 , "Present_Load"                  , 2};
  item[12] = {46 , "Moving"                        , 1};

  the_number_of_item = 13;
#else
  item[0]  = {0  , "Model_Number"                  , 2};
  item[1]  = {2  , "Firmware_Version"              , 1};
  item[2]  = {3  , "ID"                            , 1};
  item[3]  = {4  , "Baud_Rate"                     , 1};
  item[4]  = {5  , "Return_Delay_Time"             , 1};
  item[5]  = {6  , "CW_Angle_Limit"                , 2};
  item[6]  = {8  , "CCW_Angle_Limit"               , 2};
  item[7]  = {11 , "Temperature_Limit"             , 1};
  item[8]  = {12 , "Min_Voltage_Limit"             , 1};
  item[9]  = {13 , "Max_Voltage_Limit"             , 1};
  item[10] = {14 , "Max_Torque"                    , 2};
  item[11] = {16 , "Status_Return_Level"           , 1};
  item[12] = {17 , "Alarm_LED"                     , 1};
  item[13] = {18 , "Shutdown"                      , 1};

  item[14] = {24 , "Torque_Enable"                 , 1};
  item[15] = {25 , "LED"                           , 1};
  item[16] = {26 , "CW_Compliance_Margin"          , 1};
  item[17] = {27 , "CCW_Compliance_Margin"         , 1};
  item[18] = {28 , "CW_Compliance_Slope"           , 1};
  item[19] = {29 , "CCW_Compliance_Slope"          , 1};
  item[20] = {30 , "Goal_Position"                 , 2};
  item[21] = {32 , "Moving_Speed"                  , 2};
  item[22] = {34 , "Torque_Limit"                  , 2};
  item[23] = {36 , "Present_Position"              , 2};
  item[24] = {38 , "Present_Speed"                 , 2};
  item[25] = {40 , "Present_Load"                  , 2};
  item[26] = {42 , "Present_Voltage"               , 1};
  item[27] = {43 , "Present_Temperature"           , 1};
  item[28] = {44 , "Registered"                    , 1};
  item[29] = {46 , "Moving"                        , 1};
  item[30] = {47 , "Lock"                          , 1};
  item[31] = {48 , "Punch"                         , 2};

  the_number_of_item = 32;
#endif  
}

static void setAXInfo()
{
  model_info.velocity_to_value_ratio         = 86.03; // AX series don't support exact speed in wheel mode.
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 512;
  model_info.value_of_max_radian_position    = 1024;

  model_info.min_radian                      = -2.61799;
  model_info.max_radian                      =  2.61799;
}

static void setRXItem()
{
  #if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1};
  item[1]  = {4  , "Baud_Rate"                     , 1};
  item[2]  = {6  , "CW_Angle_Limit"                , 2};
  item[3]  = {8  , "CCW_Angle_Limit"               , 2};

  item[4]  = {24 , "Torque_Enable"                 , 1};
  item[5]  = {25 , "LED"                           , 1};
  item[6]  = {30 , "Goal_Position"                 , 2};
  item[7]  = {32 , "Moving_Speed"                  , 2};
  item[8]  = {34 , "Torque_Limit"                  , 2};
  item[9]  = {36 , "Present_Position"              , 2};
  item[10] = {38 , "Present_Speed"                 , 2};
  item[11] = {40 , "Present_Load"                  , 2};
  item[12] = {46 , "Moving"                        , 1};

  the_number_of_item = 13;
#else
  item[0]  = {0  , "Model_Number"                  , 2};
  item[1]  = {2  , "Firmware_Version"              , 1};
  item[2]  = {3  , "ID"                            , 1};
  item[3]  = {4  , "Baud_Rate"                     , 1};
  item[4]  = {5  , "Return_Delay_Time"             , 1};
  item[5]  = {6  , "CW_Angle_Limit"                , 2};
  item[6]  = {8  , "CCW_Angle_Limit"               , 2};
  item[7]  = {11 , "Temperature_Limit"             , 1};
  item[8]  = {12 , "Min_Voltage_Limit"             , 1};
  item[9]  = {13 , "Max_Voltage_Limit"             , 1};
  item[10] = {14 , "Max_Torque"                    , 2};
  item[11] = {16 , "Status_Return_Level"           , 1};
  item[12] = {17 , "Alarm_LED"                     , 1};
  item[13] = {18 , "Shutdown"                      , 1};

  item[14] = {24 , "Torque_Enable"                 , 1};
  item[15] = {25 , "LED"                           , 1};
  item[16] = {26 , "CW_Compliance_Margin"          , 1};
  item[17] = {27 , "CCW_Compliance_Margin"         , 1};
  item[18] = {28 , "CW_Compliance_Slope"           , 1};
  item[19] = {29 , "CCW_Compliance_Slope"          , 1};
  item[20] = {30 , "Goal_Position"                 , 2};
  item[21] = {32 , "Moving_Speed"                  , 2};
  item[22] = {34 , "Torque_Limit"                  , 2};
  item[23] = {36 , "Present_Position"              , 2};
  item[24] = {38 , "Present_Speed"                 , 2};
  item[25] = {40 , "Present_Load"                  , 2};
  item[26] = {42 , "Present_Voltage"               , 1};
  item[27] = {43 , "Present_Temperature"           , 1};
  item[28] = {44 , "Registered"                    , 1};
  item[29] = {46 , "Moving"                        , 1};
  item[30] = {47 , "Lock"                          , 1};
  item[31] = {48 , "Punch"                         , 2};

  the_number_of_item = 32;
#endif  
}

static void setRXInfo(void)
{
  model_info.velocity_to_value_ratio         = 86.03; // RX series don't support exact speed in wheel mode.
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 512;
  model_info.value_of_max_radian_position    = 1024;

  model_info.min_radian                      = -2.61799;
  model_info.max_radian                      =  2.61799;
}

static void setEXItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1};
  item[1]  = {4  , "Baud_Rate"                     , 1};
  item[2]  = {6  , "CW Angle_Limit"                , 2};
  item[3]  = {8  , "CCW Angle_Limit"               , 2};
  item[4]  = {10 , "Drive_Mode"                    , 1};

  item[5]  = {24 , "Torque_Enable"                 , 1};
  item[6]  = {25 , "LED"                           , 1};
  item[7]  = {30 , "Goal_Position"                 , 2};
  item[8]  = {32 , "Moving_Speed"                  , 2};
  item[9]  = {34 , "Torque_Limit"                  , 2};
  item[10] = {36 , "Present_Position"              , 2};
  item[11] = {38 , "Present_Speed"                 , 2};
  item[12] = {40 , "Present_Load"                  , 2};
  item[13] = {46 , "Moving"                        , 1};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model_Number"                  , 2};
  item[1]  = {2  , "Firmware_Version"              , 1};
  item[2]  = {3  , "ID"                            , 1};
  item[3]  = {4  , "Baud_Rate"                     , 1};
  item[4]  = {5  , "Return_Delay_Time"             , 1};
  item[5]  = {6  , "CW_Angle_Limit"                , 2};
  item[6]  = {8  , "CCW_Angle_Limit"               , 2};
  item[7]  = {10 , "Drive_Mode"                    , 1};
  item[8]  = {11 , "Temperature_Limit"             , 1};
  item[9]  = {12 , "Min_Voltage_Limit"             , 1};
  item[10] = {13 , "Max_Voltage_Limit"             , 1};
  item[11] = {14 , "Max_Torque"                    , 2};
  item[12] = {16 , "Status_Return_Level"           , 1};
  item[13] = {17 , "Alarm_LED"                     , 1};
  item[14] = {18 , "Shutdown"                      , 1};

  item[15] = {24 , "Torque_Enable"                 , 1};
  item[16] = {25 , "LED"                           , 1};
  item[17] = {26 , "CW_Compliance_Margin"          , 1};
  item[18] = {27 , "CCW_Compliance_Margin"         , 1};
  item[19] = {28 , "CW_Compliance_Slope"           , 1};
  item[20] = {29 , "CCW_Compliance_Slope"          , 1};
  item[21] = {30 , "Goal_Position"                 , 2};
  item[22] = {34 , "Moving_Speed"                  , 2};
  item[23] = {35 , "Torque_Limit"                  , 2};
  item[24] = {36 , "Present_Position"              , 2};
  item[25] = {38 , "Present_Speed"                 , 2};
  item[26] = {40 , "Present_Load"                  , 2};
  item[27] = {42 , "Present_Voltage"               , 1};
  item[28] = {43 , "Present_Temperature"           , 1};
  item[29] = {44 , "Registered"                    , 1};
  item[30] = {46 , "Moving"                        , 1};
  item[31] = {47 , "Lock"                          , 1};
  item[32] = {48 , "Punch"                         , 2};
  item[33] = {56 , "Sensored_Current"              , 2};

  the_number_of_item = 34;
#endif  
}

static void setEXInfo()
{
  model_info.velocity_to_value_ratio         = 86.03; // EX series don't support exact speed in wheel mode.
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 2048; 
  model_info.value_of_max_radian_position    = 4096;

  model_info.min_radian                      = -2.18969008;
  model_info.max_radian                      =  2.18969008;
}

static void setMXItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1};
  item[1]  = {4  , "Baud_Rate"                     , 1};
  item[2]  = {6  , "CW_Angle_Limit"                , 2};
  item[3]  = {8  , "CCW_Angle_Limit"               , 2};

  item[4]  = {24 , "Torque_Enable"                 , 1};
  item[5]  = {25 , "LED"                           , 1};
  item[6]  = {30 , "Goal_Position"                 , 2};
  item[7]  = {32 , "Moving_Speed"                  , 2};
  item[8]  = {34 , "Torque_Limit"                  , 2};
  item[9]  = {36 , "Present_Position"              , 2};
  item[10] = {38 , "Present_Speed"                 , 2};
  item[11] = {40 , "Present_Load"                  , 2};
  item[12] = {46 , "Moving"                        , 1};
  item[13] = {73 , "Goal_Acceleration"             , 1};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model_Number"                  , 2};
  item[1]  = {2  , "Firmware_Version"              , 1};
  item[2]  = {3  , "ID"                            , 1};
  item[3]  = {4  , "Baud_Rate"                     , 1};
  item[4]  = {5  , "Return_Delay_Time"             , 1};
  item[5]  = {6  , "CW_Angle_Limit"                , 2};
  item[6]  = {8  , "CCW_Angle_Limit"               , 2};
  item[7]  = {11 , "Temperature_Limit"             , 1};
  item[8]  = {12 , "Min_Voltage_Limit"             , 1};
  item[9]  = {13 , "Max_Voltage_Limit"             , 1};
  item[10] = {14 , "Max_Torque"                    , 2};
  item[11] = {16 , "Status_Return_Level"           , 1};
  item[12] = {17 , "Alarm_LED"                     , 1};
  item[13] = {18 , "Shutdown"                      , 1};
  item[14] = {20 , "Multi_Turn_Offset"             , 2};
  item[15] = {22 , "Resolution_Divider"            , 1};

  item[16] = {24 , "Torque_Enable"                 , 1};
  item[17] = {25 , "LED"                           , 1};
  item[18] = {26 , "D_gain"                        , 1};
  item[19] = {27 , "I_gain"                        , 1};
  item[20] = {28 , "P_gain"                        , 1};
  item[21] = {30 , "Goal_Position"                 , 2};
  item[22] = {32 , "Moving_Speed"                  , 2};
  item[23] = {34 , "Torque_Limit"                  , 2};
  item[24] = {36 , "Present_Position"              , 2};
  item[25] = {38 , "Present_Speed"                 , 2};
  item[26] = {40 , "Present_Load"                  , 2};
  item[27] = {42 , "Present_Voltage"               , 1};
  item[28] = {43 , "Present_Temperature"           , 1};
  item[29] = {44 , "Registered"                    , 1};
  item[30] = {46 , "Moving"                        , 1};
  item[31] = {47 , "Lock"                          , 1};
  item[32] = {48 , "Punch"                         , 2};
  item[33] = {73 , "Goal_Acceleration"             , 1};

  the_number_of_item = 34;
#endif  
}

static void setMXInfo()
{
  model_info.velocity_to_value_ratio         = 86.81;
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 2048;  
  model_info.value_of_max_radian_position    = 4096;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

static void setMX2Item(void)
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud_Rate"             , 1};
  item[2]  = {10 , "Drive_Mode"            , 1};
  item[3]  = {11 , "Operating_Mode"        , 1};

  item[4]  = {64 , "Torque_Enable"         , 1};
  item[5]  = {65 , "LED"                   , 1};
  item[6]  = {104, "Goal_Velocity"         , 4};
  item[7]  = {108, "Profile_Acceleration"  , 4};
  item[8]  = {112, "Profile_Velocity"      , 4};
  item[9]  = {116, "Goal_Position"         , 4};
  item[10] = {122, "Moving"                , 1};
  item[11] = {126, "Present_Load"          , 2};
  item[12] = {128, "Present_Velocity"      , 4};
  item[13] = {132, "Present_Position"      , 4};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model_Number"          , 2};
  item[1]  = {6  , "Firmware_Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud_Rate"             , 1};
  item[4]  = {9  , "Return_Delay_Time"     , 1};
  item[5]  = {10 , "Drive_Mode"            , 1};
  item[6]  = {11 , "Operating_Mode"        , 1};
  item[7]  = {12 , "Secondary_ID"          , 1};
  item[8]  = {13 , "Protocol_Version"      , 1};
  item[9]  = {20 , "Homing_Offset"         , 4};
  item[10] = {24 , "Moving_Threshold"      , 4};
  item[11] = {31 , "Temperature_Limit"     , 1};
  item[12] = {32 , "Max_Voltage_Limit"     , 2};
  item[13] = {34 , "Min_Voltage_Limit"     , 2};
  item[14] = {36 , "PWM_Limit"             , 2};
  item[15] = {40 , "Acceleration_Limit"    , 4};
  item[16] = {44 , "Velocity_Limit"        , 4};
  item[17] = {48 , "Max_Position_Limit"    , 4};
  item[18] = {52 , "Min_Position_Limit"    , 4};
  item[19] = {63 , "Shutdown"              , 1};

  item[20] = {64 , "Torque_Enable"         , 1};
  item[21] = {65 , "LED"                   , 1};
  item[22] = {68 , "Status_Return_Level"   , 1};
  item[23] = {69 , "Registered_Instruction", 1};
  item[24] = {70 , "Hardware_Error_Status" , 1};
  item[25] = {76 , "Velocity_I_Gain"       , 2};
  item[26] = {78 , "Velocity_P_Gain"       , 2};
  item[27] = {80 , "Position_D_Gain"       , 2};
  item[28] = {82 , "Position_I_Gain"       , 2};
  item[29] = {84 , "Position_P_Gain"       , 2};
  item[30] = {88 , "Feedforward_2nd_Gain"  , 2};
  item[31] = {90 , "Feedforward_1st_Gain"  , 2};
  item[32] = {98 , "Bus_Watchdog"          , 1};
  item[33] = {100, "Goal_PWM"              , 2};
  item[34] = {104, "Goal_Velocity"         , 4};
  item[35] = {108, "Profile_Acceleration"  , 4};
  item[36] = {112, "Profile_Velocity"      , 4};
  item[37] = {116, "Goal_Position"         , 4};
  item[38] = {120, "Realtime_Tick"         , 2};
  item[39] = {122, "Moving"                , 1};
  item[40] = {123, "Moving_Status"         , 1};
  item[41] = {124, "Present_PWM"           , 2};
  item[42] = {126, "Present_Load"          , 2};
  item[43] = {128, "Present_Velocity"      , 4};
  item[44] = {132, "Present_Position"      , 4};
  item[45] = {136, "Velocity_Trajectory"   , 4};
  item[46] = {140, "Position_Trajectory"   , 4};
  item[47] = {144, "Present_Input_Voltage" , 2};
  item[48] = {146, "Present_Temperature"   , 1};

  the_number_of_item = 49;
#endif  
}

static void setMX2Info(void)
{
  model_info.velocity_to_value_ratio         = 41.70;
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 2048;  
  model_info.value_of_max_radian_position    = 4096;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

static void setExtMXItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1};
  item[1]  = {4  , "Baud_Rate"                     , 1};
  item[2]  = {6  , "CW_Angle_Limit"                , 2};
  item[3]  = {8  , "CCW_Angle_Limit"               , 2};

  item[4]  = {24 , "Torque_Enable"                 , 1};
  item[5]  = {25 , "LED"                           , 1};
  item[6]  = {30 , "Goal_Position"                 , 2};
  item[7]  = {32 , "Moving_Speed"                  , 2};
  item[8]  = {34 , "Torque_Limit"                  , 2};
  item[9]  = {36 , "Present_Position"              , 2};
  item[10] = {38 , "Present_Speed"                 , 2};
  item[11] = {40 , "Present_Load"                  , 2};
  item[12] = {46 , "Moving"                        , 1};
  // item[13] = {68 , "Current"                       , 2};
  // item[14] = {70 , "Torque_Control_Mode_Enable"    , 1};
  // item[15] = {71 , "Goal_Torque"                   , 2};
  // item[16] = {73 , "Goal_Acceleration"             , 1};

  the_number_of_item = 13;
#else
  item[0]  = {0  , "Model_Number"                  , 2};
  item[1]  = {2  , "Firmware_Version"              , 1};
  item[2]  = {3  , "ID"                            , 1};
  item[3]  = {4  , "Baud_Rate"                     , 1};
  item[4]  = {5  , "Return_Delay_Time"             , 1};
  item[5]  = {6  , "CW_Angle_Limit"                , 2};
  item[6]  = {8  , "CCW_Angle_Limit"               , 2};
  item[7]  = {11 , "Temperature_Limit"             , 1};
  item[8]  = {12 , "Min_Voltage_Limit"             , 1};
  item[9]  = {13 , "Max_Voltage_Limit"             , 1};
  item[10] = {14 , "Max_Torque"                    , 2};
  item[11] = {16 , "Status_Return_Level"           , 1};
  item[12] = {17 , "Alarm_LED"                     , 1};
  item[13] = {18 , "Shutdown"                      , 1};
  item[14] = {20 , "Multi_Turn_Offset"             , 2};
  item[15] = {22 , "Resolution_Divider"            , 1};

  item[16] = {24 , "Torque_Enable"                 , 1};
  item[17] = {25 , "LED"                           , 1};
  item[18] = {26 , "D_gain"                        , 1};
  item[19] = {27 , "I_gain"                        , 1};
  item[20] = {28 , "P_gain"                        , 1};
  item[21] = {30 , "Goal_Position"                 , 2};
  item[22] = {32 , "Moving_Speed"                  , 2};
  item[23] = {34 , "Torque_Limit"                  , 2};
  item[24] = {36 , "Present_Position"              , 2};
  item[25] = {38 , "Present_Speed"                 , 2};
  item[26] = {40 , "Present_Load"                  , 2};
  item[27] = {42 , "Present_Voltage"               , 1};
  item[28] = {43 , "Present_Temperature"           , 1};
  item[29] = {44 , "Registered"                    , 1};
  item[30] = {46 , "Moving"                        , 1};
  item[31] = {47 , "Lock"                          , 1};
  item[32] = {48 , "Punch"                         , 2};
  item[33] = {68 , "Current"                       , 2};
  item[34] = {70 , "Torque_Control_Mode_Enable"    , 1};
  item[35] = {71 , "Goal_Torque"                   , 2};
  item[36] = {73 , "Goal_Acceleration"             , 1};

  the_number_of_item = 37;
#endif  
}

static void setExtMXInfo()
{
  model_info.velocity_to_value_ratio         = 86.81;
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 2048;  
  model_info.value_of_max_radian_position    = 4096;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

static void setExtMX2Item(void)
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud_Rate"             , 1};
  item[2]  = {11 , "Operating_Mode"        , 1};

  item[3]  = {64 , "Torque_Enable"         , 1};
  item[4]  = {65 , "LED"                   , 1};
  item[5]  = {102, "Goal_Current"          , 2};
  item[6]  = {104, "Goal_Velocity"         , 4};
  item[7]  = {108, "Profile_Acceleration"  , 4};
  item[8]  = {112, "Profile_Velocity"      , 4};
  item[9]  = {116, "Goal_Position"         , 4};
  item[10] = {122, "Moving"                , 1};
  item[11] = {126, "Present_Current"       , 2};
  item[12] = {128, "Present_Velocity"      , 4};
  item[13] = {132, "Present_Position"      , 4};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model_Number"          , 2};
  item[1]  = {6  , "Firmware_Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud_Rate"             , 1};
  item[4]  = {9  , "Return_Delay_Time"     , 1};
  item[5]  = {10 , "Drive_Mode"            , 1};
  item[6]  = {11 , "Operating_Mode"        , 1};
  item[7]  = {12 , "Secondary_ID"          , 1};
  item[8]  = {13 , "Protocol_Version"      , 1};
  item[9]  = {20 , "Homing_Offset"         , 4};
  item[10] = {24 , "Moving_Threshold"      , 4};
  item[11] = {31 , "Temperature_Limit"     , 1};
  item[12] = {32 , "Max_Voltage_Limit"     , 2};
  item[13] = {34 , "Min_Voltage_Limit"     , 2};
  item[14] = {36 , "PWM_Limit"             , 2};
  item[15] = {40 , "Current_Limit"         , 2};
  item[16] = {40 , "Acceleration_Limit"    , 4};
  item[17] = {44 , "Velocity_Limit"        , 4};
  item[18] = {48 , "Max_Position_Limit"    , 4};
  item[19] = {52 , "Min_Position_Limit"    , 4};
  item[20] = {63 , "Shutdown"              , 1};

  item[21] = {64 , "Torque_Enable"         , 1};
  item[22] = {65 , "LED"                   , 1};
  item[23] = {68 , "Status_Return_Level"   , 1};
  item[24] = {69 , "Registered_Instruction", 1};
  item[25] = {70 , "Hardware_Error_Status" , 1};
  item[26] = {76 , "Velocity_I_Gain"       , 2};
  item[27] = {78 , "Velocity_P_Gain"       , 2};
  item[28] = {80 , "Position_D_Gain"       , 2};
  item[29] = {82 , "Position_I_Gain"       , 2};
  item[30] = {84 , "Position_P_Gain"       , 2};
  item[31] = {88 , "Feedforward_2nd_Gain"  , 2};
  item[32] = {90 , "Feedforward_1st_Gain"  , 2};
  item[33] = {98 , "Bus_Watchdog"          , 1};
  item[34] = {100, "Goal_PWM"              , 2};
  item[35] = {102, "Goal_Current"          , 2};
  item[36] = {104, "Goal_Velocity"         , 4};
  item[37] = {108, "Profile_Acceleration"  , 4};
  item[38] = {112, "Profile_Velocity"      , 4};
  item[39] = {116, "Goal_Position"         , 4};
  item[40] = {120, "Realtime_Tick"         , 2};
  item[41] = {122, "Moving"                , 1};
  item[42] = {123, "Moving_Status"         , 1};
  item[43] = {124, "Present_PWM"           , 2};
  item[44] = {126, "Present_Current"       , 2};
  item[45] = {128, "Present_Velocity"      , 4};
  item[46] = {132, "Present_Position"      , 4};
  item[47] = {136, "Velocity_Trajectory"   , 4};
  item[48] = {140, "Position_Trajectory"   , 4};
  item[49] = {144, "Present_Input Voltage" , 2};
  item[50] = {146, "Present_Temperature"   , 1};

  the_number_of_item = 51;
#endif  
}

static void setExtMX2Info(void)
{
  model_info.velocity_to_value_ratio         = 41.70;
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 2048;  
  model_info.value_of_max_radian_position    = 4096;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

static void setXL320Item()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1};
  item[1]  = {4  , "Baud_Rate"                     , 1};
  item[2]  = {6  , "CW_Angle_Limit"                , 2};
  item[3]  = {8  , "CCW_Angle_Limit"               , 2};
  item[4]  = {11 , "Control_Mode"                  , 1};

  item[5]  = {24 , "Torque_ON/OFF"                 , 1};
  item[6]  = {25 , "LED"                           , 1};
  item[7]  = {30 , "Goal_Position"                 , 2};
  item[8]  = {32 , "Moving_Speed"                  , 2};
  item[9]  = {34 , "Torque_Limit"                  , 2};
  item[10] = {37 , "Present_Position"              , 2};
  item[11] = {39 , "Present_Speed"                 , 2};
  item[12] = {41 , "Present_Load"                  , 2};
  item[13] = {49 , "Moving"                        , 1};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model_Number"                  , 2};
  item[1]  = {2  , "Firmware_Version"              , 1};
  item[2]  = {3  , "ID"                            , 1};
  item[3]  = {4  , "Baud_Rate"                     , 1};
  item[4]  = {5  , "Return_Delay_Time"             , 1};
  item[5]  = {6  , "CW_Angle_Limit"                , 2};
  item[6]  = {8  , "CCW_Angle_Limit"               , 2};
  item[7]  = {11 , "Control_Mode"                  , 1};
  item[8]  = {12 , "Temperature_Limit"             , 1};
  item[9]  = {13 , "Min_Voltage_Limit"             , 1};
  item[10] = {14 , "Max_Voltage_Limit"             , 1};
  item[11] = {15 , "Max_Torque"                    , 2};
  item[12] = {17 , "Status_Return_Level"           , 1};
  item[13] = {18 , "Shutdown"                      , 1};

  item[14] = {24 , "Torque_Enable"                 , 1};
  item[15] = {25 , "LED"                           , 1};
  item[16] = {27 , "D_gain"                        , 1};
  item[17] = {28 , "I_gain"                        , 1};
  item[18] = {29 , "P_gain"                        , 1};
  item[19] = {30 , "Goal_Position"                 , 2};
  item[20] = {32 , "Moving_Speed"                  , 2};
  item[21] = {34 , "Torque_Limit"                  , 2};
  item[22] = {37 , "Present_Position"              , 2};
  item[23] = {39 , "Present_Speed"                 , 2};
  item[24] = {41 , "Present_Load"                  , 2};
  item[25] = {45 , "Present_Voltage"               , 1};
  item[26] = {46 , "Present_Temperature"           , 1};
  item[27] = {47 , "Registered"                    , 1};
  item[28] = {49 , "Moving"                        , 1};
  item[29] = {50 , "Hardware_Error_Status"         , 1};
  item[30] = {51 , "Punch"                         , 2};

  the_number_of_item = 31;
#endif  
}

static void setXL320Info()
{
  model_info.velocity_to_value_ratio         = 86.03; // XL320 don't support exact speed in wheel mode.
  
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 512;  
  model_info.value_of_max_radian_position    = 1024;

  model_info.min_radian                      = -2.61799;
  model_info.max_radian                      =  2.61799;
}

static void setXLItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud_Rate"             , 1};
  item[2]  = {11 , "Operating_Mode"        , 1};

  item[3]  = {64 , "Torque_Enable"         , 1};
  item[4]  = {65 , "LED"                   , 1};
  item[5]  = {102, "Goal_Current"          , 2};
  item[6]  = {104, "Goal_Velocity"         , 4};
  item[7]  = {108, "Profile_Acceleration"  , 4};
  item[8]  = {112, "Profile_Velocity"      , 4};
  item[9]  = {116, "Goal_Position"         , 4};
  item[10] = {122, "Moving"                , 1};
  item[11] = {126, "Present_Load"          , 2};
  item[12] = {128, "Present_Velocity"      , 4};
  item[13] = {132, "Present_Position"      , 4};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model_Number"          , 2};
  item[1]  = {6  , "Firmware_Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud_Rate"             , 1};
  item[4]  = {9  , "Return_Delay_Time"     , 1};
  item[5]  = {10 , "Drive_Mode"            , 1};
  item[6]  = {11 , "Operating_Mode"        , 1};
  item[7]  = {12 , "Secondary_ID"          , 1};
  item[8]  = {13 , "Protocol_Version"      , 1};
  item[9]  = {20 , "Homing_Offset"         , 4};
  item[10] = {24 , "Moving_Threshold"      , 4};
  item[11] = {31 , "Temperature_Limit"     , 1};
  item[12] = {32 , "Max_Voltage_Limit"     , 2};
  item[13] = {34 , "Min_Voltage_Limit"     , 2};
  item[14] = {36 , "PWM_Limit"             , 2};
  item[15] = {40 , "Acceleration_Limit"    , 4};
  item[16] = {44 , "Velocity_Limit"        , 4};
  item[17] = {48 , "Max_Position_Limit"    , 4};
  item[18] = {52 , "Min_Position_Limit"    , 4};
  item[19] = {63 , "Shutdown"              , 1};

  item[20] = {64 , "Torque_Enable"         , 1};
  item[21] = {65 , "LED"                   , 1};
  item[22] = {68 , "Status_Return_Level"   , 1};
  item[23] = {69 , "Registered_Instruction", 1};
  item[24] = {70 , "Hardware_Error_Status" , 1};
  item[25] = {76 , "Velocity_I_Gain"       , 2};
  item[26] = {78 , "Velocity_P_Gain"       , 2};
  item[27] = {80 , "Position_D_Gain"       , 2};
  item[28] = {82 , "Position_I_Gain"       , 2};
  item[29] = {84 , "Position_P_Gain"       , 2};
  item[30] = {88 , "Feedforward_2nd_Gain"  , 2};
  item[31] = {90 , "Feedforward_1st_Gain"  , 2};
  item[32] = {98 , "Bus_Watchdog"          , 1};
  item[33] = {100, "Goal_PWM"              , 2};
  item[34] = {104, "Goal_Velocity"         , 4};
  item[35] = {108, "Profile_Acceleration"  , 4};
  item[36] = {112, "Profile_Velocity"      , 4};
  item[37] = {116, "Goal_Position"         , 4};
  item[38] = {120, "Realtime_Tick"         , 2};
  item[39] = {122, "Moving"                , 1};
  item[40] = {123, "Moving_Status"         , 1};
  item[41] = {124, "Present_PWM"           , 2};
  item[42] = {126, "Present_Load"          , 2};
  item[43] = {128, "Present_Velocity"      , 4};
  item[44] = {132, "Present_Position"      , 4};
  item[45] = {136, "Velocity_Trajectory"   , 4};
  item[46] = {140, "Position_Trajectory"   , 4};
  item[47] = {144, "Present_Input_Voltage" , 2};
  item[48] = {146, "Present_Temperature"   , 1};

  the_number_of_item = 49;
#endif  
}

static void setXLInfo()
{
  model_info.velocity_to_value_ratio         = 41.70;

  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_0_radian_position      = 2048;  
  model_info.value_of_max_radian_position    = 4096;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

static void setXMItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud_Rate"             , 1};
  item[2]  = {11 , "Operating_Mode"        , 1};

  item[3]  = {64 , "Torque_Enable"         , 1};
  item[4]  = {65 , "LED"                   , 1};
  item[5]  = {102, "Goal_Current"          , 2};
  item[6]  = {104, "Goal_Velocity"         , 4};
  item[7]  = {108, "Profile_Acceleration"  , 4};
  item[8]  = {112, "Profile_Velocity"      , 4};
  item[9]  = {116, "Goal_Position"         , 4};
  item[10] = {122, "Moving"                , 1};
  item[11] = {126, "Present_Current"       , 2};
  item[12] = {128, "Present_Velocity"      , 4};
  item[13] = {132, "Present_Position"      , 4};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model_Number"          , 2};
  item[1]  = {6  , "Firmware_Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud_Rate"             , 1};
  item[4]  = {9  , "Return_Delay_Time"     , 1};
  item[5]  = {10 , "Drive_Mode"            , 1};
  item[6]  = {11 , "Operating_Mode"        , 1};
  item[7]  = {12 , "Secondary_ID"          , 1};
  item[8]  = {13 , "Protocol_Version"      , 1};
  item[9]  = {20 , "Homing_Offset"         , 4};
  item[10] = {24 , "Moving_Threshold"      , 4};
  item[11] = {31 , "Temperature_Limit"     , 1};
  item[12] = {32 , "Max_Voltage_Limit"     , 2};
  item[13] = {34 , "Min_Voltage_Limit"     , 2};
  item[14] = {36 , "PWM_Limit"             , 2};
  item[15] = {40 , "Current_Limit"         , 2};
  item[16] = {40 , "Acceleration_Limit"    , 4};
  item[17] = {44 , "Velocity_Limit"        , 4};
  item[18] = {48 , "Max_Position_Limit"    , 4};
  item[19] = {52 , "Min_Position_Limit"    , 4};
  item[20] = {63 , "Shutdown"              , 1};

  item[21] = {64 , "Torque_Enable"         , 1};
  item[22] = {65 , "LED"                   , 1};
  item[23] = {68 , "Status_Return_Level"   , 1};
  item[24] = {69 , "Registered_Instruction", 1};
  item[25] = {70 , "Hardware_Error_Status" , 1};
  item[26] = {76 , "Velocity_I_Gain"       , 2};
  item[27] = {78 , "Velocity_P_Gain"       , 2};
  item[28] = {80 , "Position_D_Gain"       , 2};
  item[29] = {82 , "Position_I_Gain"       , 2};
  item[30] = {84 , "Position_P_Gain"       , 2};
  item[31] = {88 , "Feedforward_2nd_Gain"  , 2};
  item[32] = {90 , "Feedforward_1st_Gain"  , 2};
  item[33] = {98 , "Bus_Watchdog"          , 1};
  item[34] = {100, "Goal_PWM"              , 2};
  item[35] = {102, "Goal_Current"          , 2};
  item[36] = {104, "Goal_Velocity"         , 4};
  item[37] = {108, "Profile_Acceleration"  , 4};
  item[38] = {112, "Profile_Velocity"      , 4};
  item[39] = {116, "Goal_Position"         , 4};
  item[40] = {120, "Realtime_Tick"         , 2};
  item[41] = {122, "Moving"                , 1};
  item[42] = {123, "Moving_Status"         , 1};
  item[43] = {124, "Present_PWM"           , 2};
  item[44] = {126, "Present_Current"       , 2};
  item[45] = {128, "Present_Velocity"      , 4};
  item[46] = {132, "Present_Position"      , 4};
  item[47] = {136, "Velocity_Trajectory"   , 4};
  item[48] = {140, "Position_Trajectory"   , 4};
  item[49] = {144, "Present_Input_Voltage" , 2};
  item[50] = {146, "Present_Temperature"   , 1};

  the_number_of_item = 51;
#endif  
}

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

static void setExtXMItem(void)
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud_Rate"             , 1};
  item[2]  = {11 , "Operating_Mode"        , 1};

  item[3]  = {64 , "Torque_Enable"         , 1};
  item[4]  = {65 , "LED"                   , 1};
  item[5]  = {102, "Goal_Current"          , 2};
  item[6]  = {104, "Goal_Velocity"         , 4};
  item[7]  = {108, "Profile_Acceleration"  , 4};
  item[8]  = {112, "Profile_Velocity"      , 4};
  item[9] = {116, "Goal_Position"         , 4};
  item[10] = {122, "Moving"                , 1};
  item[11] = {126, "Present_Current"       , 2};
  item[12] = {128, "Present_Velocity"      , 4};
  item[13] = {132, "Present_Position"      , 4};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model_Number"          , 2};
  item[1]  = {6  , "Firmware_Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud_Rate"             , 1};
  item[4]  = {9  , "Return_Delay_Time"     , 1};
  item[5]  = {10 , "Drive_Mode"            , 1};
  item[6]  = {11 , "Operating_Mode"        , 1};
  item[7]  = {12 , "Secondary_ID"          , 1};
  item[8]  = {13 , "Protocol_Version"      , 1};
  item[9]  = {20 , "Homing_Offset"         , 4};
  item[10] = {24 , "Moving_Threshold"      , 4};
  item[11] = {31 , "Temperature_Limit"     , 1};
  item[12] = {32 , "Max Voltage_Limit"     , 2};
  item[13] = {34 , "Min Voltage_Limit"     , 2};
  item[14] = {36 , "PWM_Limit"             , 2};
  item[15] = {40 , "Current_Limit"         , 2};
  item[16] = {40 , "Acceleration_Limit"    , 4};
  item[17] = {44 , "Velocity_Limit"        , 4};
  item[18] = {48 , "Max_Position_Limit"    , 4};
  item[19] = {52 , "Min_Position_Limit"    , 4};
  item[20] = {56 , "External_Port_Mode_1"  , 1};
  item[21] = {57 , "External_Port_Mode_2"  , 1};
  item[22] = {58 , "External_Port_Mode_3"  , 1};
  item[23] = {63 , "Shutdown"              , 1};

  item[24] = {64 , "Torque_Enable"         , 1};
  item[25] = {65 , "LED"                   , 1};
  item[26] = {68 , "Status_Return_Level"   , 1};
  item[27] = {69 , "Registered_Instruction", 1};
  item[28] = {70 , "Hardware_Error_Status" , 1};
  item[29] = {76 , "Velocity_I_Gain"       , 2};
  item[30] = {78 , "Velocity_P_Gain"       , 2};
  item[31] = {80 , "Position_D_Gain"       , 2};
  item[32] = {82 , "Position_I_Gain"       , 2};
  item[33] = {84 , "Position_P_Gain"       , 2};
  item[34] = {88 , "Feedforward_2nd_Gain"  , 2};
  item[35] = {90 , "Feedforward_1st_Gain"  , 2};
  item[36] = {98 , "Bus_Watchdog"          , 1};
  item[37] = {100, "Goal_PWM"              , 2};
  item[38] = {102, "Goal_Current"          , 2};
  item[39] = {104, "Goal_Velocity"         , 4};
  item[40] = {108, "Profile_Acceleration"  , 4};
  item[41] = {112, "Profile_Velocity"      , 4};
  item[42] = {116, "Goal_Position"         , 4};
  item[43] = {120, "Realtime_Tick"         , 2};
  item[44] = {122, "Moving"                , 1};
  item[45] = {123, "Moving_Status"         , 1};
  item[46] = {124, "Present_PWM"           , 2};
  item[47] = {126, "Present_Current"       , 2};
  item[48] = {128, "Present_Velocity"      , 4};
  item[49] = {132, "Present_Position"      , 4};
  item[50] = {136, "Velocity_Trajectory"   , 4};
  item[51] = {140, "Position_Trajectory"   , 4};
  item[52] = {144, "Present_Input_Voltage" , 2};
  item[53] = {146, "Present_Temperature"   , 1};

  the_number_of_item = 54;
#endif  
}

static void setExtXMInfo(void)
{
  model_info.velocity_to_value_ratio       = 41.70;

  model_info.value_of_min_radian_position  = 0;
  model_info.value_of_0_radian_position    = 2048;
  model_info.value_of_max_radian_position  = 4096;

  model_info.min_radian = -3.14159265;
  model_info.max_radian =  3.14159265;
}

static void setXHItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud_Rate"             , 1};
  item[2]  = {11 , "Operating_Mode"        , 1};

  item[3]  = {64 , "Torque_Enable"         , 1};
  item[4]  = {65 , "LED"                   , 1};
  item[5]  = {102, "Goal_Current"          , 2};
  item[6]  = {104, "Goal_Velocity"         , 4};
  item[7]  = {108, "Profile_Acceleration"  , 4};
  item[8]  = {112, "Profile_Velocity"      , 4};
  item[9]  = {116, "Goal_Position"         , 4};
  item[10] = {122, "Moving"                , 1};
  item[11] = {126, "Present_Current"       , 2};
  item[12] = {128, "Present_Velocity"      , 4};
  item[13] = {132, "Present_Position"      , 4};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model_Number"          , 2};
  item[1]  = {6  , "Firmware_Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud_Rate"             , 1};
  item[4]  = {9  , "Return_Delay_Time"     , 1};
  item[5]  = {10 , "Drive_Mode"            , 1};
  item[6]  = {11 , "Operating_Mode"        , 1};
  item[7]  = {12 , "Secondary_ID"          , 1};
  item[8]  = {13 , "Protocol_Version"      , 1};
  item[9]  = {20 , "Homing_Offset"         , 4};
  item[10] = {24 , "Moving_Threshold"      , 4};
  item[11] = {31 , "Temperature_Limit"     , 1};
  item[12] = {32 , "Max_Voltage_Limit"     , 2};
  item[13] = {34 , "Min_Voltage_Limit"     , 2};
  item[14] = {36 , "PWM_Limit"             , 2};
  item[15] = {40 , "Current_Limit"         , 2};
  item[16] = {40 , "Acceleration_Limit"    , 4};
  item[17] = {44 , "Velocity_Limit"        , 4};
  item[18] = {48 , "Max_Position_Limit"    , 4};
  item[19] = {52 , "Min_Position_Limit"    , 4};
  item[20] = {63 , "Shutdown"              , 1};

  item[21] = {64 , "Torque_Enable"         , 1};
  item[22] = {65 , "LED"                   , 1};
  item[23] = {68 , "Status_Return_Level"   , 1};
  item[24] = {69 , "Registered_Instruction", 1};
  item[25] = {70 , "Hardware_Error_Status" , 1};
  item[26] = {76 , "Velocity_I_Gain"       , 2};
  item[27] = {78 , "Velocity_P_Gain"       , 2};
  item[28] = {80 , "Position_D_Gain"       , 2};
  item[29] = {82 , "Position_I_Gain"       , 2};
  item[30] = {84 , "Position_P_Gain"       , 2};
  item[31] = {88 , "Feedforward_2nd_Gain"  , 2};
  item[32] = {90 , "Feedforward_1st_Gain"  , 2};
  item[33] = {98 , "Bus_Watchdog"          , 1};
  item[34] = {100, "Goal_PWM"              , 2};
  item[35] = {102, "Goal_Current"          , 2};
  item[36] = {104, "Goal_Velocity"         , 4};
  item[37] = {108, "Profile_Acceleration"  , 4};
  item[38] = {112, "Profile_Velocity"      , 4};
  item[39] = {116, "Goal_Position"         , 4};
  item[40] = {120, "Realtime_Tick"         , 2};
  item[41] = {122, "Moving"                , 1};
  item[42] = {123, "Moving_Status"         , 1};
  item[43] = {124, "Present_PWM"           , 2};
  item[44] = {126, "Present_Current"       , 2};
  item[45] = {128, "Present_Velocity"      , 4};
  item[46] = {132, "Present_Position"      , 4};
  item[47] = {136, "Velocity_Trajectory"   , 4};
  item[48] = {140, "Position_Trajectory"   , 4};
  item[49] = {144, "Present_Input_Voltage" , 2};
  item[50] = {146, "Present_Temperature"   , 1};

  the_number_of_item = 51;
#endif  
}

static void setXHInfo()
{
  model_info.velocity_to_value_ratio       = 41.71;

  model_info.value_of_min_radian_position  = 0;
  model_info.value_of_0_radian_position    = 2048;
  model_info.value_of_max_radian_position  = 4096;

  model_info.min_radian = -3.14159265;
  model_info.max_radian = 3.14159265;
}

static void setPROItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud_Rate"             , 1};
  item[2]  = {11 , "Operating_Mode"        , 1};

  item[3]  = {562, "Torque_Enable"         , 1};
  item[4]  = {563, "LED_RED"               , 1};
  item[5]  = {596, "Goal_Position"         , 4};
  item[6]  = {600, "Goal_Velocity"         , 4};
  item[7]  = {604, "Goal_Torque"           , 2};
  item[8]  = {606, "Goal_Acceleration"     , 4};
  item[9]  = {610, "Moving"                , 1};
  item[10] = {611, "Present_Position"      , 4};
  item[11] = {615, "Present_Velocity"      , 4};
  item[12] = {621, "Present_Current"       , 2};

  the_number_of_item = 13;
#else
  item[0]  = {0  , "Model_Number"          , 2};
  item[1]  = {6  , "Firmware_Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud_Rate"             , 1};
  item[4]  = {9  , "Return_Delay_Time"     , 1};
  item[5]  = {11 , "Operating_Mode"        , 1};
  item[6]  = {17 , "Moving_Threshold"      , 4};
  item[7]  = {21 , "Temperature_Limit"     , 1};
  item[8]  = {22 , "Max_Voltage_Limit"     , 2};
  item[9]  = {24 , "Min_Voltage_Limit"     , 2};
  item[10] = {26 , "Acceleration_Limit"    , 4};
  item[11] = {30 , "Torque_Limit"          , 2};
  item[12] = {32 , "Velocity_Limit"        , 4};
  item[13] = {36 , "Max_Position_Limit"    , 4};
  item[14] = {40 , "Min_Position_Limit"    , 4};
  item[15] = {44 , "External_Port_Mode_1"  , 1};
  item[16] = {45 , "External_Port_Mode_2"  , 1};
  item[17] = {46 , "External_Port_Mode_3"  , 1};
  item[18] = {47 , "External_Port_Mode_4"  , 1};
  item[19] = {48 , "Shutdown"              , 1};

  item[20] = {562, "Torque_Enable"         , 1};
  item[21] = {563, "LED_RED"               , 1};
  item[22] = {564, "LED_GREEN"             , 1};
  item[23] = {565, "LED_BLUE"              , 1};
  item[24] = {586, "Velocity_I_Gain"       , 2};
  item[25] = {588, "Velocity_P_Gain"       , 2};
  item[26] = {594, "Position_P_Gain"       , 2};
  item[27] = {596, "Goal_Position"         , 4};
  item[28] = {600, "Goal_Velocity"         , 4};
  item[29] = {604, "Goal_Torque"           , 2};
  item[30] = {606, "Goal_Acceleration"     , 4};
  item[31] = {610, "Moving"                , 1};
  item[32] = {611, "Present_Position"      , 4};
  item[33] = {615, "Present_Velocity"      , 4};
  item[34] = {621, "Present_Current"       , 2};
  item[35] = {623, "Present_Input_Voltage" , 2};
  item[36] = {625, "Present_Temperature"   , 1};
  item[37] = {626, "External_Port_Mode_1"  , 2};
  item[38] = {628, "External_Port_Mode_2"  , 2};
  item[39] = {630, "External_Port_Mode_3"  , 2};
  item[40] = {632, "External_Port_Mode_4"  , 2};
  item[41] = {890, "Registered_Instruction", 1};
  item[42] = {891, "Status_Return_Level"   , 1};
  item[43] = {892, "Hardware_Error_Status" , 1};

  the_number_of_item = 44;
#endif  
}

static void setExtPROItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud_Rate"             , 1};
  item[2]  = {11 , "Operating_Mode"        , 1};

  item[3]  = {562, "Torque_Enable"         , 1};
  item[4]  = {563, "LED_RED"               , 1};
  item[5]  = {596, "Goal_Position"         , 4};
  item[6]  = {600, "Goal_Velocity"         , 4};
  item[7]  = {604, "Goal_Torque"           , 2};
  item[8]  = {606, "Goal_Acceleration"     , 4};
  item[9]  = {610, "Moving"                , 1};
  item[10] = {611, "Present_Position"      , 4};
  item[11] = {615, "Present_Velocity"      , 4};
  item[12] = {621, "Present_Current"       , 2};

  the_number_of_item = 13;
#else
  item[0]  = {0  , "Model_Number"          , 2};
  item[1]  = {6  , "Firmware_Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud_Rate"             , 1};
  item[4]  = {9  , "Return_Delay_Time"     , 1};
  item[5]  = {11 , "Operating_Mode"        , 1};
  item[6]  = {13 , "Homing_Offset"         , 4};
  item[7]  = {17 , "Moving_Threshold"      , 4};
  item[8]  = {21 , "Temperature_Limit"     , 1};
  item[9]  = {22 , "Max_Voltage_Limit"     , 2};
  item[10] = {24 , "Min_Voltage_Limit"     , 2};
  item[11] = {26 , "Acceleration_Limit"    , 4};
  item[12] = {30 , "Torque_Limit"          , 2};
  item[13] = {32 , "Velocity_Limit"        , 4};
  item[14] = {36 , "Max_Position_Limit"    , 4};
  item[15] = {40 , "Min_Position_Limit"    , 4};
  item[16] = {44 , "External_Port_Mode_1"  , 1};
  item[17] = {45 , "External_Port_Mode_2"  , 1};
  item[18] = {46 , "External_Port_Mode_3"  , 1};
  item[19] = {47 , "External_Port_Mode_4"  , 1};
  item[20] = {48 , "Shutdown"              , 1};

  item[20] = {562, "Torque_Enable"         , 1};
  item[21] = {563, "LED_RED"               , 1};
  item[22] = {564, "LED_GREEN"             , 1};
  item[23] = {565, "LED_BLUE"              , 1};
  item[24] = {586, "Velocity_I_Gain"       , 2};
  item[25] = {588, "Velocity_P_Gain"       , 2};
  item[26] = {594, "Position_P_Gain"       , 2};
  item[27] = {596, "Goal_Position"         , 4};
  item[28] = {600, "Goal_Velocity"         , 4};
  item[29] = {604, "Goal_Torque"           , 2};
  item[30] = {606, "Goal_Acceleration"     , 4};
  item[31] = {610, "Moving"                , 1};
  item[32] = {611, "Present_Position"      , 4};
  item[33] = {615, "Present_Velocity"      , 4};
  item[34] = {621, "Present_Current"       , 2};
  item[35] = {623, "Present_Input_Voltage" , 2};
  item[36] = {625, "Present_Temperature"   , 1};
  item[37] = {626, "External_Port_Mode_1"  , 2};
  item[38] = {628, "External_Port_Mode_2"  , 2};
  item[39] = {630, "External_Port_Mode_3"  , 2};
  item[40] = {632, "External_Port_Mode_4"  , 2};
  item[41] = {890, "Registered_Instruction", 1};
  item[42] = {891, "Status_Return_Level"   , 1};
  item[43] = {892, "Hardware_Error_Status" , 1};

  the_number_of_item = 44;
#endif  
}

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

ControlTableItem* getConrolTableItem(uint16_t model_number)
{
  uint16_t num = model_number;

  if (num == AX_12A || num == AX_12W || num == AX_18A)
  {
    setAXItem();
  }
  else if (num == RX_10 || num == RX_24F || num == RX_28 || num == RX_64)
  {
    setRXItem();
  }
  else if (num == EX_106)
  {
    setEXItem();
  }
  else if (num == MX_12W || num == MX_28)
  {
    setMXItem();
  }
  else if (num == MX_64 || num == MX_106)
  {
    setExtMXItem();
  }
  else if (num == MX_28_2)
  {
    setMX2Item();
  }
  else if (num == MX_64_2 || num == MX_106_2)
  {
    setExtMX2Item();
  }
  else if (num == XL_320)
  {
    setXL320Item();
  }
  else if (num == XL430_W250)
  {
    setXLItem();
  }
  else if (num == XM430_W210 || num == XM430_W350)
  {
    setXMItem();
  }
  else if (num == XM540_W150 || num == XM540_W270)
  {
    setExtXMItem();
  }
  else if (num == XH430_V210 || num == XH430_V350 || num == XH430_W210 || num == XH430_W350)
  {
    setXHItem();
  }
  else if (num == PRO_L54_30_S400_R || num == PRO_L54_30_S500_R  || num == PRO_L54_50_S290_R || num == PRO_L54_50_S500_R ||
           num == PRO_M42_10_S260_R || num == PRO_M54_40_S250_R  || num == PRO_M54_60_S250_R || 
           num == PRO_H42_20_S300_R || num == PRO_H54_100_S500_R || num == PRO_H54_200_S500_R)
  {
    setExtPROItem();
  }
  else if (num == PRO_L42_10_S300_R)
  {
    setPROItem();
  }
  else
  {
    setXMItem();
  }

  return item;
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
