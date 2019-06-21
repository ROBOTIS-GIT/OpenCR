/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Taehun Lim (Darby), Ryan Shim */

#include "../../include/dynamixel_workbench_toolbox/dynamixel_item.h"

//=========================================================
// Servo register definitions
//=========================================================

//_________________________________________________________

static const char s_Acceleration_Limit[] = "Acceleration_Limit";
static const char s_Alarm_LED[] = "Alarm_LED";
static const char s_Baud_Rate[] = "Baud_Rate";
static const char s_Bus_Watchdog[] = "Bus_Watchdog";
static const char s_CCW_Angle_Limit[] = "CCW_Angle_Limit";
static const char s_CCW_Compliance_Margin[] = "CCW_Compliance_Margin";
static const char s_CCW_Compliance_Slope[] = "CCW_Compliance_Slope";
static const char s_Control_Mode[] = "Control_Mode";
static const char s_Current[] = "Current";
static const char s_Current_Limit[] = "Current_Limit";
static const char s_CW_Angle_Limit[] = "CW_Angle_Limit";
static const char s_CW_Compliance_Margin[] = "CW_Compliance_Margin";
static const char s_CW_Compliance_Slope[] = "CW_Compliance_Slope";
static const char s_D_gain[] = "D_gain";
static const char s_Drive_Mode[] = "Drive_Mode";
static const char s_External_Port_Mode_1[] = "External_Port_Mode_1";
static const char s_External_Port_Mode_2[] = "External_Port_Mode_2";
static const char s_External_Port_Mode_3[] = "External_Port_Mode_3";
static const char s_External_Port_Mode_4[] = "External_Port_Mode_4";
static const char s_Feedforward_1st_Gain[] = "Feedforward_1st_Gain";
static const char s_Feedforward_2nd_Gain[] = "Feedforward_2nd_Gain";
static const char s_Firmware_Version[] = "Firmware_Version";
static const char s_Goal_Acceleration[] = "Goal_Acceleration";
static const char s_Goal_Current[] = "Goal_Current";
static const char s_Goal_Position[] = "Goal_Position";
static const char s_Goal_PWM[] = "Goal_PWM";
static const char s_Goal_Torque[] = "Goal_Torque";
static const char s_Goal_Velocity[] = "Goal_Velocity";
static const char s_Hardware_Error_Status[] = "Hardware_Error_Status";
static const char s_Homing_Offset[] = "Homing_Offset";
static const char s_I_gain[] = "I_gain";
static const char s_ID[] = "ID";
static const char s_LED[] = "LED";
static const char s_LED_BLUE[] = "LED_BLUE";
static const char s_LED_GREEN[] = "LED_GREEN";
static const char s_LED_RED[] = "LED_RED";
static const char s_Lock[] = "Lock";
static const char s_Max_Position_Limit[] = "Max_Position_Limit";
static const char s_Max_Torque[] = "Max_Torque";
static const char s_Max_Voltage_Limit[] = "Max_Voltage_Limit";
static const char s_Min_Position_Limit[] = "Min_Position_Limit";
static const char s_Min_Voltage_Limit[] = "Min_Voltage_Limit";
static const char s_Model_Number[] = "Model_Number";
static const char s_Moving[] = "Moving";
static const char s_Moving_Speed[] = "Moving_Speed";
static const char s_Moving_Status[] = "Moving_Status";
static const char s_Moving_Threshold[] = "Moving_Threshold";
static const char s_Multi_Turn_Offset[] = "Multi_Turn_Offset";
static const char s_Operating_Mode[] = "Operating_Mode";
static const char s_P_gain[] = "P_gain";
static const char s_Position_D_Gain[] = "Position_D_Gain";
static const char s_Position_I_Gain[] = "Position_I_Gain";
static const char s_Position_P_Gain[] = "Position_P_Gain";
static const char s_Position_Trajectory[] = "Position_Trajectory";
static const char s_Present_Current[] = "Present_Current";
static const char s_Present_Input[] = "Present_Input";
static const char s_Present_Input_Voltage[] = "Present_Input_Voltage";
static const char s_Present_Load[] = "Present_Load";
static const char s_Present_Position[] = "Present_Position";
static const char s_Present_PWM[] = "Present_PWM";
static const char s_Present_Speed[] = "Present_Speed";
static const char s_Present_Temperature[] = "Present_Temperature";
static const char s_Present_Velocity[] = "Present_Velocity";
static const char s_Present_Voltage[] = "Present_Voltage";
static const char s_Profile_Acceleration[] = "Profile_Acceleration";
static const char s_Profile_Velocity[] = "Profile_Velocity";
static const char s_Protocol_Version[] = "Protocol_Version";
static const char s_Punch[] = "Punch";
static const char s_PWM_Limit[] = "PWM_Limit";
static const char s_Realtime_Tick[] = "Realtime_Tick";
static const char s_Registered[] = "Registered";
static const char s_Registered_Instruction[] = "Registered_Instruction";
static const char s_Resolution_Divider[] = "Resolution_Divider";
static const char s_Return_Delay_Time[] = "Return_Delay_Time";
static const char s_Secondary_ID[] = "Secondary_ID";
static const char s_Sensored_Current[] = "Sensored_Current";
static const char s_Shutdown[] = "Shutdown";
static const char s_Status_Return_Level[] = "Status_Return_Level";
static const char s_Temperature_Limit[] = "Temperature_Limit";
static const char s_Torque_Control_Mode_Enable[] = "Torque_Control_Mode_Enable";
static const char s_Torque_Enable[] = "Torque_Enable";
static const char s_Torque_Limit[] = "Torque_Limit";
static const char s_Velocity_I_Gain[] = "Velocity_I_Gain";
static const char s_Velocity_Limit[] = "Velocity_Limit";
static const char s_Velocity_P_Gain[] = "Velocity_P_Gain";
static const char s_Velocity_Trajectory[] = "Velocity_Trajectory";

//_________________________________________________________

//---------------------------------------------------------
// AX servos - (num == AX_12A || num == AX_12W || num == AX_18A)
//---------------------------------------------------------
static const ControlItem items_AX[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 2, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 3, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 4, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 5, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_CW_Angle_Limit, 6, sizeof(s_CW_Angle_Limit) - 1, 2},
    {s_CCW_Angle_Limit, 8, sizeof(s_CCW_Angle_Limit) - 1, 2},
    {s_Temperature_Limit, 11, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Min_Voltage_Limit, 12, sizeof(s_Min_Voltage_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 13, sizeof(s_Max_Voltage_Limit) - 1, 1},
    {s_Max_Torque, 14, sizeof(s_Max_Torque) - 1, 2},
    {s_Status_Return_Level, 16, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Alarm_LED, 17, sizeof(s_Alarm_LED) - 1, 1},
    {s_Shutdown, 18, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 24, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED, 25, sizeof(s_LED) - 1, 1},
    {s_CW_Compliance_Margin, 26, sizeof(s_CW_Compliance_Margin) - 1, 1},
    {s_CCW_Compliance_Margin, 27, sizeof(s_CCW_Compliance_Margin) - 1, 1},
    {s_CW_Compliance_Slope, 28, sizeof(s_CW_Compliance_Slope) - 1, 1},
    {s_CCW_Compliance_Slope, 29, sizeof(s_CCW_Compliance_Slope) - 1, 1},
    {s_Goal_Position, 30, sizeof(s_Goal_Position) - 1, 2},
    {s_Moving_Speed, 32, sizeof(s_Moving_Speed) - 1, 2},
    {s_Torque_Limit, 34, sizeof(s_Torque_Limit) - 1, 2},
    {s_Present_Position, 36, sizeof(s_Present_Position) - 1, 2},
    {s_Present_Speed, 38, sizeof(s_Present_Speed) - 1, 2},
    {s_Present_Load, 40, sizeof(s_Present_Load) - 1, 2},
    {s_Present_Voltage, 42, sizeof(s_Present_Voltage) - 1, 1},
    {s_Present_Temperature, 43, sizeof(s_Present_Temperature) - 1, 1},
    {s_Registered, 44, sizeof(s_Registered) - 1, 1},
    {s_Moving, 46, sizeof(s_Moving) - 1, 1},
    {s_Lock, 47, sizeof(s_Lock) - 1, 1},
    {s_Punch, 48, sizeof(s_Punch) - 1, 2}};
#define COUNT_AX_ITEMS (sizeof(items_AX) / sizeof(items_AX[0]))

static const ModelInfo info_AX = {0.11,
                                  0,
                                  512,
                                  1024,
                                  -2.61799, 
                                  2.61799};

//---------------------------------------------------------
// RX servos - (num == RX_10 || num == RX_24F || num == RX_28 || num == RX_64)
//---------------------------------------------------------
static const ControlItem items_RX[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 2, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 3, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 4, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 5, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_CW_Angle_Limit, 6, sizeof(s_CW_Angle_Limit) - 1, 2},
    {s_CCW_Angle_Limit, 8, sizeof(s_CCW_Angle_Limit) - 1, 2},
    {s_Temperature_Limit, 11, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Min_Voltage_Limit, 12, sizeof(s_Min_Voltage_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 13, sizeof(s_Max_Voltage_Limit) - 1, 1},
    {s_Max_Torque, 14, sizeof(s_Max_Torque) - 1, 2},
    {s_Status_Return_Level, 16, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Alarm_LED, 17, sizeof(s_Alarm_LED) - 1, 1},
    {s_Shutdown, 18, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 24, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED, 25, sizeof(s_LED) - 1, 1},
    {s_CW_Compliance_Margin, 26, sizeof(s_CW_Compliance_Margin) - 1, 1},
    {s_CCW_Compliance_Margin, 27, sizeof(s_CCW_Compliance_Margin) - 1, 1},
    {s_CW_Compliance_Slope, 28, sizeof(s_CW_Compliance_Slope) - 1, 1},
    {s_CCW_Compliance_Slope, 29, sizeof(s_CCW_Compliance_Slope) - 1, 1},
    {s_Goal_Position, 30, sizeof(s_Goal_Position) - 1, 2},
    {s_Moving_Speed, 32, sizeof(s_Moving_Speed) - 1, 2},
    {s_Torque_Limit, 34, sizeof(s_Torque_Limit) - 1, 2},
    {s_Present_Position, 36, sizeof(s_Present_Position) - 1, 2},
    {s_Present_Speed, 38, sizeof(s_Present_Speed) - 1, 2},
    {s_Present_Load, 40, sizeof(s_Present_Load) - 1, 2},
    {s_Present_Voltage, 42, sizeof(s_Present_Voltage) - 1, 1},
    {s_Present_Temperature, 43, sizeof(s_Present_Temperature) - 1, 1},
    {s_Registered, 44, sizeof(s_Registered) - 1, 1},
    {s_Moving, 46, sizeof(s_Moving) - 1, 1},
    {s_Lock, 47, sizeof(s_Lock) - 1, 1},
    {s_Punch, 48, sizeof(s_Punch) - 1, 2}};

#define COUNT_RX_ITEMS (sizeof(items_RX) / sizeof(items_RX[0]))

static const ModelInfo info_RX = {0.11,
                                  0,
                                  512,
                                  1024,
                                  -2.61799, 
                                  2.61799};

//---------------------------------------------------------
// EX servos - (num == EX_106)
//---------------------------------------------------------
static const ControlItem items_EX[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 2, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 3, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 4, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 5, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_CW_Angle_Limit, 6, sizeof(s_CW_Angle_Limit) - 1, 2},
    {s_CCW_Angle_Limit, 8, sizeof(s_CCW_Angle_Limit) - 1, 2},
    {s_Drive_Mode, 10, sizeof(s_Drive_Mode) - 1, 1},
    {s_Temperature_Limit, 11, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Min_Voltage_Limit, 12, sizeof(s_Min_Voltage_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 13, sizeof(s_Max_Voltage_Limit) - 1, 1},
    {s_Max_Torque, 14, sizeof(s_Max_Torque) - 1, 2},
    {s_Status_Return_Level, 16, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Alarm_LED, 17, sizeof(s_Alarm_LED) - 1, 1},
    {s_Shutdown, 18, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 24, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED, 25, sizeof(s_LED) - 1, 1},
    {s_CW_Compliance_Margin, 26, sizeof(s_CW_Compliance_Margin) - 1, 1},
    {s_CCW_Compliance_Margin, 27, sizeof(s_CCW_Compliance_Margin) - 1, 1},
    {s_CW_Compliance_Slope, 28, sizeof(s_CW_Compliance_Slope) - 1, 1},
    {s_CCW_Compliance_Slope, 29, sizeof(s_CCW_Compliance_Slope) - 1, 1},
    {s_Goal_Position, 30, sizeof(s_Goal_Position) - 1, 2},
    {s_Moving_Speed, 34, sizeof(s_Moving_Speed) - 1, 2},
    {s_Torque_Limit, 35, sizeof(s_Torque_Limit) - 1, 2},
    {s_Present_Position, 36, sizeof(s_Present_Position) - 1, 2},
    {s_Present_Speed, 38, sizeof(s_Present_Speed) - 1, 2},
    {s_Present_Load, 40, sizeof(s_Present_Load) - 1, 2},
    {s_Present_Voltage, 42, sizeof(s_Present_Voltage) - 1, 1},
    {s_Present_Temperature, 43, sizeof(s_Present_Temperature) - 1, 1},
    {s_Registered, 44, sizeof(s_Registered) - 1, 1},
    {s_Moving, 46, sizeof(s_Moving) - 1, 1},
    {s_Lock, 47, sizeof(s_Lock) - 1, 1},
    {s_Punch, 48, sizeof(s_Punch) - 1, 2},
    {s_Sensored_Current, 56, sizeof(s_Sensored_Current) - 1, 2}};

#define COUNT_EX_ITEMS (sizeof(items_EX) / sizeof(items_EX[0]))

static const ModelInfo info_EX = {0.11,
                                  0,
                                  2048,
                                  4096,
                                  -2.18969008, 
                                  2.18969008};

//---------------------------------------------------------
// MX Protocol 1 servos - (num == MX_12W || num == MX_28)
//---------------------------------------------------------
static const ControlItem items_MX[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 2, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 3, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 4, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 5, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_CW_Angle_Limit, 6, sizeof(s_CW_Angle_Limit) - 1, 2},
    {s_CCW_Angle_Limit, 8, sizeof(s_CCW_Angle_Limit) - 1, 2},
    {s_Temperature_Limit, 11, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Min_Voltage_Limit, 12, sizeof(s_Min_Voltage_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 13, sizeof(s_Max_Voltage_Limit) - 1, 1},
    {s_Max_Torque, 14, sizeof(s_Max_Torque) - 1, 2},
    {s_Status_Return_Level, 16, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Alarm_LED, 17, sizeof(s_Alarm_LED) - 1, 1},
    {s_Shutdown, 18, sizeof(s_Shutdown) - 1, 1},
    {s_Multi_Turn_Offset, 20, sizeof(s_Multi_Turn_Offset) - 1, 2},
    {s_Resolution_Divider, 22, sizeof(s_Resolution_Divider) - 1, 1},

    {s_Torque_Enable, 24, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED, 25, sizeof(s_LED) - 1, 1},
    {s_D_gain, 26, sizeof(s_D_gain) - 1, 1},
    {s_I_gain, 27, sizeof(s_I_gain) - 1, 1},
    {s_P_gain, 28, sizeof(s_P_gain) - 1, 1},
    {s_Goal_Position, 30, sizeof(s_Goal_Position) - 1, 2},
    {s_Moving_Speed, 32, sizeof(s_Moving_Speed) - 1, 2},
    {s_Torque_Limit, 34, sizeof(s_Torque_Limit) - 1, 2},
    {s_Present_Position, 36, sizeof(s_Present_Position) - 1, 2},
    {s_Present_Speed, 38, sizeof(s_Present_Speed) - 1, 2},
    {s_Present_Load, 40, sizeof(s_Present_Load) - 1, 2},
    {s_Present_Voltage, 42, sizeof(s_Present_Voltage) - 1, 1},
    {s_Present_Temperature, 43, sizeof(s_Present_Temperature) - 1, 1},
    {s_Registered, 44, sizeof(s_Registered) - 1, 1},
    {s_Moving, 46, sizeof(s_Moving) - 1, 1},
    {s_Lock, 47, sizeof(s_Lock) - 1, 1},
    {s_Punch, 48, sizeof(s_Punch) - 1, 2},
    {s_Goal_Acceleration, 73, sizeof(s_Goal_Acceleration) - 1, 1}};

#define COUNT_MX_ITEMS (sizeof(items_MX) / sizeof(items_MX[0]))

static const ModelInfo info_MX = {0.11,
                                  0,
                                  2048,
                                  4096,
                                  -3.14159265, 
                                  3.14159265};

//---------------------------------------------------------
// MX Protocol 2 servos - (num == MX_28_2)
//---------------------------------------------------------
static const ControlItem items_MX2[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 6, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 7, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 8, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 9, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_Drive_Mode, 10, sizeof(s_Drive_Mode) - 1, 1},
    {s_Operating_Mode, 11, sizeof(s_Operating_Mode) - 1, 1},
    {s_Secondary_ID, 12, sizeof(s_Secondary_ID) - 1, 1},
    {s_Protocol_Version, 13, sizeof(s_Protocol_Version) - 1, 1},
    {s_Homing_Offset, 20, sizeof(s_Homing_Offset) - 1, 4},
    {s_Moving_Threshold, 24, sizeof(s_Moving_Threshold) - 1, 4},
    {s_Temperature_Limit, 31, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 32, sizeof(s_Max_Voltage_Limit) - 1, 2},
    {s_Min_Voltage_Limit, 34, sizeof(s_Min_Voltage_Limit) - 1, 2},
    {s_PWM_Limit, 36, sizeof(s_PWM_Limit) - 1, 2},
    {s_Acceleration_Limit, 40, sizeof(s_Acceleration_Limit) - 1, 4},
    {s_Velocity_Limit, 44, sizeof(s_Velocity_Limit) - 1, 4},
    {s_Max_Position_Limit, 48, sizeof(s_Max_Position_Limit) - 1, 4},
    {s_Min_Position_Limit, 52, sizeof(s_Min_Position_Limit) - 1, 4},
    {s_Shutdown, 63, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 64, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED, 65, sizeof(s_LED) - 1, 1},
    {s_Status_Return_Level, 68, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Registered_Instruction, 69, sizeof(s_Registered_Instruction) - 1, 1},
    {s_Hardware_Error_Status, 70, sizeof(s_Hardware_Error_Status) - 1, 1},
    {s_Velocity_I_Gain, 76, sizeof(s_Velocity_I_Gain) - 1, 2},
    {s_Velocity_P_Gain, 78, sizeof(s_Velocity_P_Gain) - 1, 2},
    {s_Position_D_Gain, 80, sizeof(s_Position_D_Gain) - 1, 2},
    {s_Position_I_Gain, 82, sizeof(s_Position_I_Gain) - 1, 2},
    {s_Position_P_Gain, 84, sizeof(s_Position_P_Gain) - 1, 2},
    {s_Feedforward_2nd_Gain, 88, sizeof(s_Feedforward_2nd_Gain) - 1, 2},
    {s_Feedforward_1st_Gain, 90, sizeof(s_Feedforward_1st_Gain) - 1, 2},
    {s_Bus_Watchdog, 98, sizeof(s_Bus_Watchdog) - 1, 1},
    {s_Goal_PWM, 100, sizeof(s_Goal_PWM) - 1, 2},
    {s_Goal_Velocity, 104, sizeof(s_Goal_Velocity) - 1, 4},
    {s_Profile_Acceleration, 108, sizeof(s_Profile_Acceleration) - 1, 4},
    {s_Profile_Velocity, 112, sizeof(s_Profile_Velocity) - 1, 4},
    {s_Goal_Position, 116, sizeof(s_Goal_Position) - 1, 4},
    {s_Realtime_Tick, 120, sizeof(s_Realtime_Tick) - 1, 2},
    {s_Moving, 122, sizeof(s_Moving) - 1, 1},
    {s_Moving_Status, 123, sizeof(s_Moving_Status) - 1, 1},
    {s_Present_PWM, 124, sizeof(s_Present_PWM) - 1, 2},
    {s_Present_Load, 126, sizeof(s_Present_Load) - 1, 2},
    {s_Present_Velocity, 128, sizeof(s_Present_Velocity) - 1, 4},
    {s_Present_Position, 132, sizeof(s_Present_Position) - 1, 4},
    {s_Velocity_Trajectory, 136, sizeof(s_Velocity_Trajectory) - 1, 4},
    {s_Position_Trajectory, 140, sizeof(s_Position_Trajectory) - 1, 4},
    {s_Present_Input_Voltage, 144, sizeof(s_Present_Input_Voltage) - 1, 2},
    {s_Present_Temperature, 146, sizeof(s_Present_Temperature) - 1, 1}};

#define COUNT_MX2_ITEMS (sizeof(items_MX2) / sizeof(items_MX2[0]))

static const ModelInfo info_MX2 = {0.229,
                                  0,
                                  2048,
                                  4096,
                                  -3.14159265, 
                                  3.14159265};

//---------------------------------------------------------
// EXT MX Protocol 1 servos - (num == MX_64 || num == MX_106)
//---------------------------------------------------------
static const ControlItem items_EXTMX[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 2, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 3, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 4, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 5, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_CW_Angle_Limit, 6, sizeof(s_CW_Angle_Limit) - 1, 2},
    {s_CCW_Angle_Limit, 8, sizeof(s_CCW_Angle_Limit) - 1, 2},
    {s_Temperature_Limit, 11, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Min_Voltage_Limit, 12, sizeof(s_Min_Voltage_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 13, sizeof(s_Max_Voltage_Limit) - 1, 1},
    {s_Max_Torque, 14, sizeof(s_Max_Torque) - 1, 2},
    {s_Status_Return_Level, 16, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Alarm_LED, 17, sizeof(s_Alarm_LED) - 1, 1},
    {s_Shutdown, 18, sizeof(s_Shutdown) - 1, 1},
    {s_Multi_Turn_Offset, 20, sizeof(s_Multi_Turn_Offset) - 1, 2},
    {s_Resolution_Divider, 22, sizeof(s_Resolution_Divider) - 1, 1},

    {s_Torque_Enable, 24, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED, 25, sizeof(s_LED) - 1, 1},
    {s_D_gain, 26, sizeof(s_D_gain) - 1, 1},
    {s_I_gain, 27, sizeof(s_I_gain) - 1, 1},
    {s_P_gain, 28, sizeof(s_P_gain) - 1, 1},
    {s_Goal_Position, 30, sizeof(s_Goal_Position) - 1, 2},
    {s_Moving_Speed, 32, sizeof(s_Moving_Speed) - 1, 2},
    {s_Torque_Limit, 34, sizeof(s_Torque_Limit) - 1, 2},
    {s_Present_Position, 36, sizeof(s_Present_Position) - 1, 2},
    {s_Present_Speed, 38, sizeof(s_Present_Speed) - 1, 2},
    {s_Present_Load, 40, sizeof(s_Present_Load) - 1, 2},
    {s_Present_Voltage, 42, sizeof(s_Present_Voltage) - 1, 1},
    {s_Present_Temperature, 43, sizeof(s_Present_Temperature) - 1, 1},
    {s_Registered, 44, sizeof(s_Registered) - 1, 1},
    {s_Moving, 46, sizeof(s_Moving) - 1, 1},
    {s_Lock, 47, sizeof(s_Lock) - 1, 1},
    {s_Punch, 48, sizeof(s_Punch) - 1, 2},
    {s_Current, 68, sizeof(s_Current) - 1, 2},
    {s_Torque_Control_Mode_Enable, 70, sizeof(s_Torque_Control_Mode_Enable) - 1, 1},
    {s_Goal_Torque, 71, sizeof(s_Goal_Torque) - 1, 2},
    {s_Goal_Acceleration, 73, sizeof(s_Goal_Acceleration) - 1, 1}};

#define COUNT_EXTMX_ITEMS (sizeof(items_EXTMX) / sizeof(items_EXTMX[0]))

static const ModelInfo info_EXTMX = {0.11,
                                  0,
                                  2048,
                                  4096,
                                  -3.14159265, 
                                  3.14159265};

//---------------------------------------------------------
// EXT MX Protocol 2 Servos - (num == MX_64_2 || num == MX_106_2)
//---------------------------------------------------------
static const ControlItem items_EXTMX2[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 6, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 7, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 8, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 9, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_Drive_Mode, 10, sizeof(s_Drive_Mode) - 1, 1},
    {s_Operating_Mode, 11, sizeof(s_Operating_Mode) - 1, 1},
    {s_Secondary_ID, 12, sizeof(s_Secondary_ID) - 1, 1},
    {s_Protocol_Version, 13, sizeof(s_Protocol_Version) - 1, 1},
    {s_Homing_Offset, 20, sizeof(s_Homing_Offset) - 1, 4},
    {s_Moving_Threshold, 24, sizeof(s_Moving_Threshold) - 1, 4},
    {s_Temperature_Limit, 31, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 32, sizeof(s_Max_Voltage_Limit) - 1, 2},
    {s_Min_Voltage_Limit, 34, sizeof(s_Min_Voltage_Limit) - 1, 2},
    {s_PWM_Limit, 36, sizeof(s_PWM_Limit) - 1, 2},
    {s_Current_Limit, 38, sizeof(s_Current_Limit) - 1, 2},
    {s_Acceleration_Limit, 40, sizeof(s_Acceleration_Limit) - 1, 4},
    {s_Velocity_Limit, 44, sizeof(s_Velocity_Limit) - 1, 4},
    {s_Max_Position_Limit, 48, sizeof(s_Max_Position_Limit) - 1, 4},
    {s_Min_Position_Limit, 52, sizeof(s_Min_Position_Limit) - 1, 4},
    {s_Shutdown, 63, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 64, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED, 65, sizeof(s_LED) - 1, 1},
    {s_Status_Return_Level, 68, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Registered_Instruction, 69, sizeof(s_Registered_Instruction) - 1, 1},
    {s_Hardware_Error_Status, 70, sizeof(s_Hardware_Error_Status) - 1, 1},
    {s_Velocity_I_Gain, 76, sizeof(s_Velocity_I_Gain) - 1, 2},
    {s_Velocity_P_Gain, 78, sizeof(s_Velocity_P_Gain) - 1, 2},
    {s_Position_D_Gain, 80, sizeof(s_Position_D_Gain) - 1, 2},
    {s_Position_I_Gain, 82, sizeof(s_Position_I_Gain) - 1, 2},
    {s_Position_P_Gain, 84, sizeof(s_Position_P_Gain) - 1, 2},
    {s_Feedforward_2nd_Gain, 88, sizeof(s_Feedforward_2nd_Gain) - 1, 2},
    {s_Feedforward_1st_Gain, 90, sizeof(s_Feedforward_1st_Gain) - 1, 2},
    {s_Bus_Watchdog, 98, sizeof(s_Bus_Watchdog) - 1, 1},
    {s_Goal_PWM, 100, sizeof(s_Goal_PWM) - 1, 2},
    {s_Goal_Current, 102, sizeof(s_Goal_Current) - 1, 2},
    {s_Goal_Velocity, 104, sizeof(s_Goal_Velocity) - 1, 4},
    {s_Profile_Acceleration, 108, sizeof(s_Profile_Acceleration) - 1, 4},
    {s_Profile_Velocity, 112, sizeof(s_Profile_Velocity) - 1, 4},
    {s_Goal_Position, 116, sizeof(s_Goal_Position) - 1, 4},
    {s_Realtime_Tick, 120, sizeof(s_Realtime_Tick) - 1, 2},
    {s_Moving, 122, sizeof(s_Moving) - 1, 1},
    {s_Moving_Status, 123, sizeof(s_Moving_Status) - 1, 1},
    {s_Present_PWM, 124, sizeof(s_Present_PWM) - 1, 2},
    {s_Present_Current, 126, sizeof(s_Present_Current) - 1, 2},
    {s_Present_Velocity, 128, sizeof(s_Present_Velocity) - 1, 4},
    {s_Present_Position, 132, sizeof(s_Present_Position) - 1, 4},
    {s_Velocity_Trajectory, 136, sizeof(s_Velocity_Trajectory) - 1, 4},
    {s_Position_Trajectory, 140, sizeof(s_Position_Trajectory) - 1, 4},
    {s_Present_Input_Voltage, 144, sizeof(s_Present_Input_Voltage) - 1, 2},
    {s_Present_Temperature, 146, sizeof(s_Present_Temperature) - 1, 1}};

#define COUNT_EXTMX2_ITEMS (sizeof(items_EXTMX2) / sizeof(items_EXTMX2[0]))

static const ModelInfo info_EXTMX2 = {0.229,
                                  0,
                                  2048,
                                  4096,
                                  -3.14159265, 
                                  3.14159265};

//---------------------------------------------------------
// XL320 - (num == XL_320)
//---------------------------------------------------------
static const ControlItem items_XL320[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 2, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 3, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 4, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 5, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_CW_Angle_Limit, 6, sizeof(s_CW_Angle_Limit) - 1, 2},
    {s_CCW_Angle_Limit, 8, sizeof(s_CCW_Angle_Limit) - 1, 2},
    {s_Control_Mode, 11, sizeof(s_Control_Mode) - 1, 1},
    {s_Temperature_Limit, 12, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Min_Voltage_Limit, 13, sizeof(s_Min_Voltage_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 14, sizeof(s_Max_Voltage_Limit) - 1, 1},
    {s_Max_Torque, 15, sizeof(s_Max_Torque) - 1, 2},
    {s_Status_Return_Level, 17, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Shutdown, 18, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 24, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED, 25, sizeof(s_LED) - 1, 1},
    {s_D_gain, 27, sizeof(s_D_gain) - 1, 1},
    {s_I_gain, 28, sizeof(s_I_gain) - 1, 1},
    {s_P_gain, 29, sizeof(s_P_gain) - 1, 1},
    {s_Goal_Position, 30, sizeof(s_Goal_Position) - 1, 2},
    {s_Moving_Speed, 32, sizeof(s_Moving_Speed) - 1, 2},
    {s_Torque_Limit, 34, sizeof(s_Torque_Limit) - 1, 2},
    {s_Present_Position, 37, sizeof(s_Present_Position) - 1, 2},
    {s_Present_Speed, 39, sizeof(s_Present_Speed) - 1, 2},
    {s_Present_Load, 41, sizeof(s_Present_Load) - 1, 2},
    {s_Present_Voltage, 45, sizeof(s_Present_Voltage) - 1, 1},
    {s_Present_Temperature, 46, sizeof(s_Present_Temperature) - 1, 1},
    {s_Registered, 47, sizeof(s_Registered) - 1, 1},
    {s_Moving, 49, sizeof(s_Moving) - 1, 1},
    {s_Hardware_Error_Status, 50, sizeof(s_Hardware_Error_Status) - 1, 1},
    {s_Punch, 51, sizeof(s_Punch) - 1, 2}};

#define COUNT_XL320_ITEMS (sizeof(items_XL320) / sizeof(items_XL320[0]))

static const ModelInfo info_XL320 = {0.11,
                                  0,
                                  512,
                                  1024,
                                  -2.61799, 
                                  2.61799};

//---------------------------------------------------------
// XL - (num == XL430_W250, XL430_W250_2, XC430_W150, XC430_W240)
//---------------------------------------------------------
static const ControlItem items_XL[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 6, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 7, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 8, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 9, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_Drive_Mode, 10, sizeof(s_Drive_Mode) - 1, 1},
    {s_Operating_Mode, 11, sizeof(s_Operating_Mode) - 1, 1},
    {s_Secondary_ID, 12, sizeof(s_Secondary_ID) - 1, 1},
    {s_Protocol_Version, 13, sizeof(s_Protocol_Version) - 1, 1},
    {s_Homing_Offset, 20, sizeof(s_Homing_Offset) - 1, 4},
    {s_Moving_Threshold, 24, sizeof(s_Moving_Threshold) - 1, 4},
    {s_Temperature_Limit, 31, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 32, sizeof(s_Max_Voltage_Limit) - 1, 2},
    {s_Min_Voltage_Limit, 34, sizeof(s_Min_Voltage_Limit) - 1, 2},
    {s_PWM_Limit, 36, sizeof(s_PWM_Limit) - 1, 2},
    {s_Acceleration_Limit, 40, sizeof(s_Acceleration_Limit) - 1, 4},
    {s_Velocity_Limit, 44, sizeof(s_Velocity_Limit) - 1, 4},
    {s_Max_Position_Limit, 48, sizeof(s_Max_Position_Limit) - 1, 4},
    {s_Min_Position_Limit, 52, sizeof(s_Min_Position_Limit) - 1, 4},
    {s_Shutdown, 63, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 64, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED, 65, sizeof(s_LED) - 1, 1},
    {s_Status_Return_Level, 68, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Registered_Instruction, 69, sizeof(s_Registered_Instruction) - 1, 1},
    {s_Hardware_Error_Status, 70, sizeof(s_Hardware_Error_Status) - 1, 1},
    {s_Velocity_I_Gain, 76, sizeof(s_Velocity_I_Gain) - 1, 2},
    {s_Velocity_P_Gain, 78, sizeof(s_Velocity_P_Gain) - 1, 2},
    {s_Position_D_Gain, 80, sizeof(s_Position_D_Gain) - 1, 2},
    {s_Position_I_Gain, 82, sizeof(s_Position_I_Gain) - 1, 2},
    {s_Position_P_Gain, 84, sizeof(s_Position_P_Gain) - 1, 2},
    {s_Feedforward_2nd_Gain, 88, sizeof(s_Feedforward_2nd_Gain) - 1, 2},
    {s_Feedforward_1st_Gain, 90, sizeof(s_Feedforward_1st_Gain) - 1, 2},
    {s_Bus_Watchdog, 98, sizeof(s_Bus_Watchdog) - 1, 1},
    {s_Goal_PWM, 100, sizeof(s_Goal_PWM) - 1, 2},
    {s_Goal_Velocity, 104, sizeof(s_Goal_Velocity) - 1, 4},
    {s_Profile_Acceleration, 108, sizeof(s_Profile_Acceleration) - 1, 4},
    {s_Profile_Velocity, 112, sizeof(s_Profile_Velocity) - 1, 4},
    {s_Goal_Position, 116, sizeof(s_Goal_Position) - 1, 4},
    {s_Realtime_Tick, 120, sizeof(s_Realtime_Tick) - 1, 2},
    {s_Moving, 122, sizeof(s_Moving) - 1, 1},
    {s_Moving_Status, 123, sizeof(s_Moving_Status) - 1, 1},
    {s_Present_PWM, 124, sizeof(s_Present_PWM) - 1, 2},
    {s_Present_Load, 126, sizeof(s_Present_Load) - 1, 2},
    {s_Present_Velocity, 128, sizeof(s_Present_Velocity) - 1, 4},
    {s_Present_Position, 132, sizeof(s_Present_Position) - 1, 4},
    {s_Velocity_Trajectory, 136, sizeof(s_Velocity_Trajectory) - 1, 4},
    {s_Position_Trajectory, 140, sizeof(s_Position_Trajectory) - 1, 4},
    {s_Present_Input_Voltage, 144, sizeof(s_Present_Input_Voltage) - 1, 2},
    {s_Present_Temperature, 146, sizeof(s_Present_Temperature) - 1, 1}};

#define COUNT_XL_ITEMS (sizeof(items_XL) / sizeof(items_XL[0]))

static const ModelInfo info_XL = {0.229,
                                  0,
                                  2048,
                                  4096,
                                  -3.14159265, 
                                  3.14159265};

//---------------------------------------------------------
// XM - (num == XM430_W210 || num == XM430_W350)
//---------------------------------------------------------
static const ControlItem items_XM[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 6, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 7, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 8, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 9, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_Drive_Mode, 10, sizeof(s_Drive_Mode) - 1, 1},
    {s_Operating_Mode, 11, sizeof(s_Operating_Mode) - 1, 1},
    {s_Secondary_ID, 12, sizeof(s_Secondary_ID) - 1, 1},
    {s_Protocol_Version, 13, sizeof(s_Protocol_Version) - 1, 1},
    {s_Homing_Offset, 20, sizeof(s_Homing_Offset) - 1, 4},
    {s_Moving_Threshold, 24, sizeof(s_Moving_Threshold) - 1, 4},
    {s_Temperature_Limit, 31, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 32, sizeof(s_Max_Voltage_Limit) - 1, 2},
    {s_Min_Voltage_Limit, 34, sizeof(s_Min_Voltage_Limit) - 1, 2},
    {s_PWM_Limit, 36, sizeof(s_PWM_Limit) - 1, 2},
    {s_Current_Limit, 38, sizeof(s_Current_Limit) - 1, 2},
    {s_Acceleration_Limit, 40, sizeof(s_Acceleration_Limit) - 1, 4},
    {s_Velocity_Limit, 44, sizeof(s_Velocity_Limit) - 1, 4},
    {s_Max_Position_Limit, 48, sizeof(s_Max_Position_Limit) - 1, 4},
    {s_Min_Position_Limit, 52, sizeof(s_Min_Position_Limit) - 1, 4},
    {s_Shutdown, 63, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 64, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED, 65, sizeof(s_LED) - 1, 1},
    {s_Status_Return_Level, 68, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Registered_Instruction, 69, sizeof(s_Registered_Instruction) - 1, 1},
    {s_Hardware_Error_Status, 70, sizeof(s_Hardware_Error_Status) - 1, 1},
    {s_Velocity_I_Gain, 76, sizeof(s_Velocity_I_Gain) - 1, 2},
    {s_Velocity_P_Gain, 78, sizeof(s_Velocity_P_Gain) - 1, 2},
    {s_Position_D_Gain, 80, sizeof(s_Position_D_Gain) - 1, 2},
    {s_Position_I_Gain, 82, sizeof(s_Position_I_Gain) - 1, 2},
    {s_Position_P_Gain, 84, sizeof(s_Position_P_Gain) - 1, 2},
    {s_Feedforward_2nd_Gain, 88, sizeof(s_Feedforward_2nd_Gain) - 1, 2},
    {s_Feedforward_1st_Gain, 90, sizeof(s_Feedforward_1st_Gain) - 1, 2},
    {s_Bus_Watchdog, 98, sizeof(s_Bus_Watchdog) - 1, 1},
    {s_Goal_PWM, 100, sizeof(s_Goal_PWM) - 1, 2},
    {s_Goal_Current, 102, sizeof(s_Goal_Current) - 1, 2},
    {s_Goal_Velocity, 104, sizeof(s_Goal_Velocity) - 1, 4},
    {s_Profile_Acceleration, 108, sizeof(s_Profile_Acceleration) - 1, 4},
    {s_Profile_Velocity, 112, sizeof(s_Profile_Velocity) - 1, 4},
    {s_Goal_Position, 116, sizeof(s_Goal_Position) - 1, 4},
    {s_Realtime_Tick, 120, sizeof(s_Realtime_Tick) - 1, 2},
    {s_Moving, 122, sizeof(s_Moving) - 1, 1},
    {s_Moving_Status, 123, sizeof(s_Moving_Status) - 1, 1},
    {s_Present_PWM, 124, sizeof(s_Present_PWM) - 1, 2},
    {s_Present_Current, 126, sizeof(s_Present_Current) - 1, 2},
    {s_Present_Velocity, 128, sizeof(s_Present_Velocity) - 1, 4},
    {s_Present_Position, 132, sizeof(s_Present_Position) - 1, 4},
    {s_Velocity_Trajectory, 136, sizeof(s_Velocity_Trajectory) - 1, 4},
    {s_Position_Trajectory, 140, sizeof(s_Position_Trajectory) - 1, 4},
    {s_Present_Input_Voltage, 144, sizeof(s_Present_Input_Voltage) - 1, 2},
    {s_Present_Temperature, 146, sizeof(s_Present_Temperature) - 1, 1}};

#define COUNT_XM_ITEMS (sizeof(items_XM) / sizeof(items_XM[0]))

static const ModelInfo info_XM = {0.229,
                                  0,
                                  2048,
                                  4096,
                                  -3.14159265, 
                                  3.14159265};

//---------------------------------------------------------
// EXTXM - (num == XM540_W150 || num == XM540_W270)
//---------------------------------------------------------
static const ControlItem items_EXTXM[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 6, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 7, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 8, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 9, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_Drive_Mode, 10, sizeof(s_Drive_Mode) - 1, 1},
    {s_Operating_Mode, 11, sizeof(s_Operating_Mode) - 1, 1},
    {s_Secondary_ID, 12, sizeof(s_Secondary_ID) - 1, 1},
    {s_Protocol_Version, 13, sizeof(s_Protocol_Version) - 1, 1},
    {s_Homing_Offset, 20, sizeof(s_Homing_Offset) - 1, 4},
    {s_Moving_Threshold, 24, sizeof(s_Moving_Threshold) - 1, 4},
    {s_Temperature_Limit, 31, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 32, sizeof(s_Max_Voltage_Limit) - 1, 2},
    {s_Min_Voltage_Limit, 34, sizeof(s_Min_Voltage_Limit) - 1, 2},
    {s_PWM_Limit, 36, sizeof(s_PWM_Limit) - 1, 2},
    {s_Current_Limit, 38, sizeof(s_Current_Limit) - 1, 2},
    {s_Acceleration_Limit, 40, sizeof(s_Acceleration_Limit) - 1, 4},
    {s_Velocity_Limit, 44, sizeof(s_Velocity_Limit) - 1, 4},
    {s_Max_Position_Limit, 48, sizeof(s_Max_Position_Limit) - 1, 4},
    {s_Min_Position_Limit, 52, sizeof(s_Min_Position_Limit) - 1, 4},
    {s_External_Port_Mode_1, 56, sizeof(s_External_Port_Mode_1) - 1, 1},
    {s_External_Port_Mode_2, 57, sizeof(s_External_Port_Mode_2) - 1, 1},
    {s_External_Port_Mode_3, 58, sizeof(s_External_Port_Mode_3) - 1, 1},
    {s_Shutdown, 63, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 64, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED, 65, sizeof(s_LED) - 1, 1},
    {s_Status_Return_Level, 68, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Registered_Instruction, 69, sizeof(s_Registered_Instruction) - 1, 1},
    {s_Hardware_Error_Status, 70, sizeof(s_Hardware_Error_Status) - 1, 1},
    {s_Velocity_I_Gain, 76, sizeof(s_Velocity_I_Gain) - 1, 2},
    {s_Velocity_P_Gain, 78, sizeof(s_Velocity_P_Gain) - 1, 2},
    {s_Position_D_Gain, 80, sizeof(s_Position_D_Gain) - 1, 2},
    {s_Position_I_Gain, 82, sizeof(s_Position_I_Gain) - 1, 2},
    {s_Position_P_Gain, 84, sizeof(s_Position_P_Gain) - 1, 2},
    {s_Feedforward_2nd_Gain, 88, sizeof(s_Feedforward_2nd_Gain) - 1, 2},
    {s_Feedforward_1st_Gain, 90, sizeof(s_Feedforward_1st_Gain) - 1, 2},
    {s_Bus_Watchdog, 98, sizeof(s_Bus_Watchdog) - 1, 1},
    {s_Goal_PWM, 100, sizeof(s_Goal_PWM) - 1, 2},
    {s_Goal_Current, 102, sizeof(s_Goal_Current) - 1, 2},
    {s_Goal_Velocity, 104, sizeof(s_Goal_Velocity) - 1, 4},
    {s_Profile_Acceleration, 108, sizeof(s_Profile_Acceleration) - 1, 4},
    {s_Profile_Velocity, 112, sizeof(s_Profile_Velocity) - 1, 4},
    {s_Goal_Position, 116, sizeof(s_Goal_Position) - 1, 4},
    {s_Realtime_Tick, 120, sizeof(s_Realtime_Tick) - 1, 2},
    {s_Moving, 122, sizeof(s_Moving) - 1, 1},
    {s_Moving_Status, 123, sizeof(s_Moving_Status) - 1, 1},
    {s_Present_PWM, 124, sizeof(s_Present_PWM) - 1, 2},
    {s_Present_Current, 126, sizeof(s_Present_Current) - 1, 2},
    {s_Present_Velocity, 128, sizeof(s_Present_Velocity) - 1, 4},
    {s_Present_Position, 132, sizeof(s_Present_Position) - 1, 4},
    {s_Velocity_Trajectory, 136, sizeof(s_Velocity_Trajectory) - 1, 4},
    {s_Position_Trajectory, 140, sizeof(s_Position_Trajectory) - 1, 4},
    {s_Present_Input_Voltage, 144, sizeof(s_Present_Input_Voltage) - 1, 2},
    {s_Present_Temperature, 146, sizeof(s_Present_Temperature) - 1, 1}};

#define COUNT_EXTXM_ITEMS (sizeof(items_EXTXM) / sizeof(items_EXTXM[0]))

static const ModelInfo info_EXTXM = {0.229,
                                  0,
                                  2048,
                                  4096,
                                  -3.14159265, 
                                  3.14159265};

//---------------------------------------------------------
// XH - (num == XH430_V210 || num == XH430_V350 || num == XH430_W210 || num == XH430_W350)
//---------------------------------------------------------
static const ControlItem items_XH[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 6, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 7, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 8, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 9, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_Drive_Mode, 10, sizeof(s_Drive_Mode) - 1, 1},
    {s_Operating_Mode, 11, sizeof(s_Operating_Mode) - 1, 1},
    {s_Secondary_ID, 12, sizeof(s_Secondary_ID) - 1, 1},
    {s_Protocol_Version, 13, sizeof(s_Protocol_Version) - 1, 1},
    {s_Homing_Offset, 20, sizeof(s_Homing_Offset) - 1, 4},
    {s_Moving_Threshold, 24, sizeof(s_Moving_Threshold) - 1, 4},
    {s_Temperature_Limit, 31, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 32, sizeof(s_Max_Voltage_Limit) - 1, 2},
    {s_Min_Voltage_Limit, 34, sizeof(s_Min_Voltage_Limit) - 1, 2},
    {s_PWM_Limit, 36, sizeof(s_PWM_Limit) - 1, 2},
    {s_Current_Limit, 38, sizeof(s_Current_Limit) - 1, 2},
    {s_Acceleration_Limit, 40, sizeof(s_Acceleration_Limit) - 1, 4},
    {s_Velocity_Limit, 44, sizeof(s_Velocity_Limit) - 1, 4},
    {s_Max_Position_Limit, 48, sizeof(s_Max_Position_Limit) - 1, 4},
    {s_Min_Position_Limit, 52, sizeof(s_Min_Position_Limit) - 1, 4},
    {s_Shutdown, 63, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 64, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED, 65, sizeof(s_LED) - 1, 1},
    {s_Status_Return_Level, 68, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Registered_Instruction, 69, sizeof(s_Registered_Instruction) - 1, 1},
    {s_Hardware_Error_Status, 70, sizeof(s_Hardware_Error_Status) - 1, 1},
    {s_Velocity_I_Gain, 76, sizeof(s_Velocity_I_Gain) - 1, 2},
    {s_Velocity_P_Gain, 78, sizeof(s_Velocity_P_Gain) - 1, 2},
    {s_Position_D_Gain, 80, sizeof(s_Position_D_Gain) - 1, 2},
    {s_Position_I_Gain, 82, sizeof(s_Position_I_Gain) - 1, 2},
    {s_Position_P_Gain, 84, sizeof(s_Position_P_Gain) - 1, 2},
    {s_Feedforward_2nd_Gain, 88, sizeof(s_Feedforward_2nd_Gain) - 1, 2},
    {s_Feedforward_1st_Gain, 90, sizeof(s_Feedforward_1st_Gain) - 1, 2},
    {s_Bus_Watchdog, 98, sizeof(s_Bus_Watchdog) - 1, 1},
    {s_Goal_PWM, 100, sizeof(s_Goal_PWM) - 1, 2},
    {s_Goal_Current, 102, sizeof(s_Goal_Current) - 1, 2},
    {s_Goal_Velocity, 104, sizeof(s_Goal_Velocity) - 1, 4},
    {s_Profile_Acceleration, 108, sizeof(s_Profile_Acceleration) - 1, 4},
    {s_Profile_Velocity, 112, sizeof(s_Profile_Velocity) - 1, 4},
    {s_Goal_Position, 116, sizeof(s_Goal_Position) - 1, 4},
    {s_Realtime_Tick, 120, sizeof(s_Realtime_Tick) - 1, 2},
    {s_Moving, 122, sizeof(s_Moving) - 1, 1},
    {s_Moving_Status, 123, sizeof(s_Moving_Status) - 1, 1},
    {s_Present_PWM, 124, sizeof(s_Present_PWM) - 1, 2},
    {s_Present_Current, 126, sizeof(s_Present_Current) - 1, 2},
    {s_Present_Velocity, 128, sizeof(s_Present_Velocity) - 1, 4},
    {s_Present_Position, 132, sizeof(s_Present_Position) - 1, 4},
    {s_Velocity_Trajectory, 136, sizeof(s_Velocity_Trajectory) - 1, 4},
    {s_Position_Trajectory, 140, sizeof(s_Position_Trajectory) - 1, 4},
    {s_Present_Input_Voltage, 144, sizeof(s_Present_Input_Voltage) - 1, 2},
    {s_Present_Temperature, 146, sizeof(s_Present_Temperature) - 1, 1}};

#define COUNT_XH_ITEMS (sizeof(items_XH) / sizeof(items_XH[0]))

static const ModelInfo info_XH = {0.229,
                                  0,
                                  2048,
                                  4096,
                                  -3.14159265, 
                                  3.14159265};

//---------------------------------------------------------
// EXTXH - (num == XH540_W150 || num == XH540_W270 || num == XH540_V150 || num == XH540_W270)
//---------------------------------------------------------
static const ControlItem items_EXTXH[]{
    {s_Model_Number,       0,  sizeof(s_Model_Number) - 1,       2},
    {s_Firmware_Version,   6,  sizeof(s_Firmware_Version) - 1,   1},
    {s_ID,                 7,  sizeof(s_ID) - 1,                 1},
    {s_Baud_Rate,          8,  sizeof(s_Baud_Rate) - 1,          1},
    {s_Return_Delay_Time,  9,  sizeof(s_Return_Delay_Time) - 1,  1},
    {s_Drive_Mode,         10, sizeof(s_Drive_Mode) - 1,         1},
    {s_Operating_Mode,     11, sizeof(s_Operating_Mode) - 1,     1},
    {s_Secondary_ID,       12, sizeof(s_Secondary_ID) - 1,       1},
    {s_Protocol_Version,   13, sizeof(s_Protocol_Version) - 1,   1},
    {s_Homing_Offset,      20, sizeof(s_Homing_Offset) - 1,      4},
    {s_Moving_Threshold,   24, sizeof(s_Moving_Threshold) - 1,   4},
    {s_Temperature_Limit,  31, sizeof(s_Temperature_Limit) - 1,  1},
    {s_Max_Voltage_Limit,  32, sizeof(s_Max_Voltage_Limit) - 1,  2},
    {s_Min_Voltage_Limit,  34, sizeof(s_Min_Voltage_Limit) - 1,  2},
    {s_PWM_Limit,          36, sizeof(s_PWM_Limit) - 1,          2},
    {s_Current_Limit,      38, sizeof(s_Current_Limit) - 1,      2},
    {s_Acceleration_Limit, 40, sizeof(s_Acceleration_Limit) - 1, 4},
    {s_Velocity_Limit,     44, sizeof(s_Velocity_Limit) - 1,     4},
    {s_Max_Position_Limit, 48, sizeof(s_Max_Position_Limit) - 1, 4},
    {s_Min_Position_Limit, 52, sizeof(s_Min_Position_Limit) - 1, 4},
    {s_Shutdown,           63, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable,          64,  sizeof(s_Torque_Enable) - 1,          1},
    {s_LED,                    65,  sizeof(s_LED) - 1,                    1},
    {s_Status_Return_Level,    68,  sizeof(s_Status_Return_Level) - 1,    1},
    {s_Registered_Instruction, 69,  sizeof(s_Registered_Instruction) - 1, 1},
    {s_Hardware_Error_Status,  70,  sizeof(s_Hardware_Error_Status) - 1,  1},
    {s_Velocity_I_Gain,        76,  sizeof(s_Velocity_I_Gain) - 1,        2},
    {s_Velocity_P_Gain,        78,  sizeof(s_Velocity_P_Gain) - 1,        2},
    {s_Position_D_Gain,        80,  sizeof(s_Position_D_Gain) - 1,        2},
    {s_Position_I_Gain,        82,  sizeof(s_Position_I_Gain) - 1,        2},
    {s_Position_P_Gain,        84,  sizeof(s_Position_P_Gain) - 1,        2},
    {s_Feedforward_2nd_Gain,   88,  sizeof(s_Feedforward_2nd_Gain) - 1,   2},
    {s_Feedforward_1st_Gain,   90,  sizeof(s_Feedforward_1st_Gain) - 1,   2},
    {s_Bus_Watchdog,           98,  sizeof(s_Bus_Watchdog) - 1,           1},
    {s_Goal_PWM,               100, sizeof(s_Goal_PWM) - 1,               2},
    {s_Goal_Current,           102, sizeof(s_Goal_Current) - 1,           2},
    {s_Goal_Velocity,          104, sizeof(s_Goal_Velocity) - 1,          4},
    {s_Profile_Acceleration,   108, sizeof(s_Profile_Acceleration) - 1,   4},
    {s_Profile_Velocity,       112, sizeof(s_Profile_Velocity) - 1,       4},
    {s_Goal_Position,          116, sizeof(s_Goal_Position) - 1,          4},
    {s_Realtime_Tick,          120, sizeof(s_Realtime_Tick) - 1,          2},
    {s_Moving,                 122, sizeof(s_Moving) - 1,                 1},
    {s_Moving_Status,          123, sizeof(s_Moving_Status) - 1,          1},
    {s_Present_PWM,            124, sizeof(s_Present_PWM) - 1,            2},
    {s_Present_Current,        126, sizeof(s_Present_Current) - 1,        2},
    {s_Present_Velocity,       128, sizeof(s_Present_Velocity) - 1,       4},
    {s_Present_Position,       132, sizeof(s_Present_Position) - 1,       4},
    {s_Velocity_Trajectory,    136, sizeof(s_Velocity_Trajectory) - 1,    4},
    {s_Position_Trajectory,    140, sizeof(s_Position_Trajectory) - 1,    4},
    {s_Present_Input_Voltage,  144, sizeof(s_Present_Input_Voltage) - 1,  2},
    {s_Present_Temperature,    146, sizeof(s_Present_Temperature) - 1,    1}};

#define COUNT_EXTXH_ITEMS (sizeof(items_EXTXH) / sizeof(items_EXTXH[0]))

static const ModelInfo info_EXTXH = {0.229,
                                  0,
                                  2048,
                                  4096,
                                  -3.14159265, 
                                  3.14159265};

//---------------------------------------------------------
// PRO - (num == PRO_L42_10_S300_R)
//---------------------------------------------------------
static const ControlItem items_PRO[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 6, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 7, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 8, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 9, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_Operating_Mode, 11, sizeof(s_Operating_Mode) - 1, 1},
    {s_Moving_Threshold, 17, sizeof(s_Moving_Threshold) - 1, 4},
    {s_Temperature_Limit, 21, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 22, sizeof(s_Max_Voltage_Limit) - 1, 2},
    {s_Min_Voltage_Limit, 24, sizeof(s_Min_Voltage_Limit) - 1, 2},
    {s_Acceleration_Limit, 26, sizeof(s_Acceleration_Limit) - 1, 4},
    {s_Torque_Limit, 30, sizeof(s_Torque_Limit) - 1, 2},
    {s_Velocity_Limit, 32, sizeof(s_Velocity_Limit) - 1, 4},
    {s_Max_Position_Limit, 36, sizeof(s_Max_Position_Limit) - 1, 4},
    {s_Min_Position_Limit, 40, sizeof(s_Min_Position_Limit) - 1, 4},
    {s_External_Port_Mode_1, 44, sizeof(s_External_Port_Mode_1) - 1, 1},
    {s_External_Port_Mode_2, 45, sizeof(s_External_Port_Mode_2) - 1, 1},
    {s_External_Port_Mode_3, 46, sizeof(s_External_Port_Mode_3) - 1, 1},
    {s_External_Port_Mode_4, 47, sizeof(s_External_Port_Mode_4) - 1, 1},
    {s_Shutdown, 48, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 562, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED_RED, 563, sizeof(s_LED_RED) - 1, 1},
    {s_LED_GREEN, 564, sizeof(s_LED_GREEN) - 1, 1},
    {s_LED_BLUE, 565, sizeof(s_LED_BLUE) - 1, 1},
    {s_Velocity_I_Gain, 586, sizeof(s_Velocity_I_Gain) - 1, 2},
    {s_Velocity_P_Gain, 588, sizeof(s_Velocity_P_Gain) - 1, 2},
    {s_Position_P_Gain, 594, sizeof(s_Position_P_Gain) - 1, 2},
    {s_Goal_Position, 596, sizeof(s_Goal_Position) - 1, 4},
    {s_Goal_Velocity, 600, sizeof(s_Goal_Velocity) - 1, 4},
    {s_Goal_Torque, 604, sizeof(s_Goal_Torque) - 1, 2},
    {s_Goal_Acceleration, 606, sizeof(s_Goal_Acceleration) - 1, 4},
    {s_Moving, 610, sizeof(s_Moving) - 1, 1},
    {s_Present_Position, 611, sizeof(s_Present_Position) - 1, 4},
    {s_Present_Velocity, 615, sizeof(s_Present_Velocity) - 1, 4},
    {s_Present_Current, 621, sizeof(s_Present_Current) - 1, 2},
    {s_Present_Input_Voltage, 623, sizeof(s_Present_Input_Voltage) - 1, 2},
    {s_Present_Temperature, 625, sizeof(s_Present_Temperature) - 1, 1},
    {s_External_Port_Mode_1, 626, sizeof(s_External_Port_Mode_1) - 1, 2},
    {s_External_Port_Mode_2, 628, sizeof(s_External_Port_Mode_2) - 1, 2},
    {s_External_Port_Mode_3, 630, sizeof(s_External_Port_Mode_3) - 1, 2},
    {s_External_Port_Mode_4, 632, sizeof(s_External_Port_Mode_4) - 1, 2},
    {s_Registered_Instruction, 890, sizeof(s_Registered_Instruction) - 1, 1},
    {s_Status_Return_Level, 891, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Hardware_Error_Status, 892, sizeof(s_Hardware_Error_Status) - 1, 1}};

#define COUNT_PRO_ITEMS (sizeof(items_PRO) / sizeof(items_PRO[0]))

static const ModelInfo info_PRO = {0.114,
                                  -2048,
                                  0,
                                  2048,
                                  -3.14159265, 
                                  3.14159265};

//---------------------------------------------------------
// EXT PRO - All Other Pros...
//---------------------------------------------------------
static const ControlItem items_EXTPRO[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 6, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 7, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 8, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 9, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_Operating_Mode, 11, sizeof(s_Operating_Mode) - 1, 1},
    {s_Homing_Offset, 13, sizeof(s_Homing_Offset) - 1, 4},
    {s_Moving_Threshold, 17, sizeof(s_Moving_Threshold) - 1, 4},
    {s_Temperature_Limit, 21, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 22, sizeof(s_Max_Voltage_Limit) - 1, 2},
    {s_Min_Voltage_Limit, 24, sizeof(s_Min_Voltage_Limit) - 1, 2},
    {s_Acceleration_Limit, 26, sizeof(s_Acceleration_Limit) - 1, 4},
    {s_Torque_Limit, 30, sizeof(s_Torque_Limit) - 1, 2},
    {s_Velocity_Limit, 32, sizeof(s_Velocity_Limit) - 1, 4},
    {s_Max_Position_Limit, 36, sizeof(s_Max_Position_Limit) - 1, 4},
    {s_Min_Position_Limit, 40, sizeof(s_Min_Position_Limit) - 1, 4},
    {s_External_Port_Mode_1, 44, sizeof(s_External_Port_Mode_1) - 1, 1},
    {s_External_Port_Mode_2, 45, sizeof(s_External_Port_Mode_2) - 1, 1},
    {s_External_Port_Mode_3, 46, sizeof(s_External_Port_Mode_3) - 1, 1},
    {s_External_Port_Mode_4, 47, sizeof(s_External_Port_Mode_4) - 1, 1},
    {s_Shutdown, 48, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 562, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED_RED, 563, sizeof(s_LED_RED) - 1, 1},
    {s_LED_GREEN, 564, sizeof(s_LED_GREEN) - 1, 1},
    {s_LED_BLUE, 565, sizeof(s_LED_BLUE) - 1, 1},
    {s_Velocity_I_Gain, 586, sizeof(s_Velocity_I_Gain) - 1, 2},
    {s_Velocity_P_Gain, 588, sizeof(s_Velocity_P_Gain) - 1, 2},
    {s_Position_P_Gain, 594, sizeof(s_Position_P_Gain) - 1, 2},
    {s_Goal_Position, 596, sizeof(s_Goal_Position) - 1, 4},
    {s_Goal_Velocity, 600, sizeof(s_Goal_Velocity) - 1, 4},
    {s_Goal_Torque, 604, sizeof(s_Goal_Torque) - 1, 2},
    {s_Goal_Acceleration, 606, sizeof(s_Goal_Acceleration) - 1, 4},
    {s_Moving, 610, sizeof(s_Moving) - 1, 1},
    {s_Present_Position, 611, sizeof(s_Present_Position) - 1, 4},
    {s_Present_Velocity, 615, sizeof(s_Present_Velocity) - 1, 4},
    {s_Present_Current, 621, sizeof(s_Present_Current) - 1, 2},
    {s_Present_Input_Voltage, 623, sizeof(s_Present_Input_Voltage) - 1, 2},
    {s_Present_Temperature, 625, sizeof(s_Present_Temperature) - 1, 1},
    {s_External_Port_Mode_1, 626, sizeof(s_External_Port_Mode_1) - 1, 2},
    {s_External_Port_Mode_2, 628, sizeof(s_External_Port_Mode_2) - 1, 2},
    {s_External_Port_Mode_3, 630, sizeof(s_External_Port_Mode_3) - 1, 2},
    {s_External_Port_Mode_4, 632, sizeof(s_External_Port_Mode_4) - 1, 2},
    {s_Registered_Instruction, 890, sizeof(s_Registered_Instruction) - 1, 1},
    {s_Status_Return_Level, 891, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Hardware_Error_Status, 892, sizeof(s_Hardware_Error_Status) - 1, 1}};

#define COUNT_EXTPRO_ITEMS (sizeof(items_EXTPRO) / sizeof(items_EXTPRO[0]))

static const ModelInfo info_EXTPRO[] = {
      {0.00249657, -144197, 0, 144197, -3.14159265, 3.14159265},  // PRO_L54_30_S400_R
      {0.00199234, -180692, 0, 180692, -3.14159265, 3.14159265},  // PRO_L54_30_S500_R, PRO_L54_50_S500_R
      {0.00346667, -103846, 0, 103846, -3.14159265, 3.14159265},  // PRO_L54_50_S290_R
      {0.00389076, -131593, 0, 131593, -3.14159265, 3.14159265},  // PRO_M42_10_S260_R
      {0.00397746, -125708, 0, 125708, -3.14159265, 3.14159265},  // PRO_M54_40_S250_R, PRO_M54_60_S250_R
      {0.00329218, -151875, 0, 151875, -3.14159265, 3.14159265},  // PRO_H42_20_S300_R
      {0.00199234, -250961, 0, 250961, -3.14159265, 3.14159265}}; // PRO_H54_100_S500_R, PRO_H54_200_S500_R

//---------------------------------------------------------
// EXT PRO (A Firmware_Version) 
//---------------------------------------------------------
static const ControlItem items_EXTPRO_A[]{
    {s_Model_Number,         0, sizeof(s_Model_Number) - 1,          2},
    {s_Firmware_Version,     6, sizeof(s_Firmware_Version) - 1,      1},
    {s_ID,                   7, sizeof(s_ID) - 1,                    1},
    {s_Baud_Rate,            8, sizeof(s_Baud_Rate) - 1,             1},
    {s_Return_Delay_Time,    9, sizeof(s_Return_Delay_Time) - 1,     1},
    {s_Operating_Mode,       11, sizeof(s_Operating_Mode) - 1,       1},
    {s_Homing_Offset,        20, sizeof(s_Homing_Offset) - 1,        4},
    {s_Moving_Threshold,     24, sizeof(s_Moving_Threshold) - 1,     4},
    {s_Temperature_Limit,    31, sizeof(s_Temperature_Limit) - 1,    1},
    {s_Max_Voltage_Limit,    32, sizeof(s_Max_Voltage_Limit) - 1,    2},
    {s_Min_Voltage_Limit,    34, sizeof(s_Min_Voltage_Limit) - 1,    2},
    {s_Current_Limit,        38, sizeof(s_Current_Limit) - 1,        2},
    {s_Acceleration_Limit,   40, sizeof(s_Acceleration_Limit) - 1,   4},
    {s_Velocity_Limit,       44, sizeof(s_Velocity_Limit) - 1,       4},
    {s_Max_Position_Limit,   48, sizeof(s_Max_Position_Limit) - 1,   4},
    {s_Min_Position_Limit,   52, sizeof(s_Min_Position_Limit) - 1,   4},
    {s_External_Port_Mode_1, 56, sizeof(s_External_Port_Mode_1) - 1, 1},
    {s_External_Port_Mode_2, 57, sizeof(s_External_Port_Mode_2) - 1, 1},
    {s_External_Port_Mode_3, 58, sizeof(s_External_Port_Mode_3) - 1, 1},
    {s_External_Port_Mode_4, 59, sizeof(s_External_Port_Mode_4) - 1, 1},
    {s_Shutdown,             63, sizeof(s_Shutdown) - 1,             1},

    {s_Torque_Enable,          512, sizeof(s_Torque_Enable) - 1,          1},
    {s_LED_RED,                513, sizeof(s_LED_RED) - 1,                1},
    {s_LED_GREEN,              514, sizeof(s_LED_GREEN) - 1,              1},
    {s_LED_BLUE,               515, sizeof(s_LED_BLUE) - 1,               1},
    {s_Velocity_I_Gain,        524, sizeof(s_Velocity_I_Gain) - 1,        2},
    {s_Velocity_P_Gain,        526, sizeof(s_Velocity_P_Gain) - 1,        2},
    {s_Position_D_Gain,        528, sizeof(s_Position_D_Gain) - 1,        2},
    {s_Position_P_Gain,        532, sizeof(s_Position_P_Gain) - 1,        2},
    {s_Position_I_Gain,        530, sizeof(s_Position_I_Gain) - 1,        2},
    {s_Goal_Position,          564, sizeof(s_Goal_Position) - 1,          4},
    {s_Goal_Velocity,          552, sizeof(s_Goal_Velocity) - 1,          4},
    {s_Goal_Current,           604, sizeof(s_Goal_Current) - 1,           2},
    {s_Profile_Acceleration,   556, sizeof(s_Profile_Acceleration) - 1,   4},
    {s_Profile_Velocity,       560, sizeof(s_Profile_Velocity) - 1,       4},
    {s_Moving,                 570, sizeof(s_Moving) - 1,                 1},
    {s_Present_Position,       580, sizeof(s_Present_Position) - 1,       4},
    {s_Present_Velocity,       576, sizeof(s_Present_Velocity) - 1,       4},
    {s_Present_Current,        574, sizeof(s_Present_Current) - 1,        2},
    {s_Present_Input_Voltage,  592, sizeof(s_Present_Input_Voltage) - 1,  2},
    {s_Present_Temperature,    594, sizeof(s_Present_Temperature) - 1,    1},
    {s_External_Port_Mode_1,   600, sizeof(s_External_Port_Mode_1) - 1,   2},
    {s_External_Port_Mode_2,   602, sizeof(s_External_Port_Mode_2) - 1,   2},
    {s_External_Port_Mode_3,   604, sizeof(s_External_Port_Mode_3) - 1,   2},
    {s_External_Port_Mode_4,   606, sizeof(s_External_Port_Mode_4) - 1,   2}};

#define COUNT_EXTPRO_A_ITEMS (sizeof(items_EXTPRO_A) / sizeof(items_EXTPRO_A[0]))

static const ModelInfo info_EXTPRO_A[] = {
      {0.00389076, -131593, 0, 131593, -3.14159265, 3.14159265},  // PRO_M42_10_S260_R_A
      {0.00397746, -125708, 0, 125708, -3.14159265, 3.14159265},  // PRO_M54_40_S250_R_A, PRO_M54_60_S250_R_A
      {0.00329218, -151875, 0, 151875, -3.14159265, 3.14159265},  // PRO_H42_20_S300_R_A
      {0.00199234, -250961, 0, 250961, -3.14159265, 3.14159265}}; // PRO_H54_100_S500_R_A, PRO_H54_200_S500_R_A

//---------------------------------------------------------
// PRO PLUS - (num == PRO_H42P_020_S300_R, PRO_H54P_100_S500_R, PRO_H54P_200_S500_R)
//---------------------------------------------------------
static const ControlItem items_PRO_PLUS[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 6, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 7, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 8, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 9, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_Drive_Mode, 10, sizeof(s_Drive_Mode) - 1, 1},
    {s_Operating_Mode, 11, sizeof(s_Operating_Mode) - 1, 1},
    {s_Secondary_ID, 12, sizeof(s_Secondary_ID) - 1, 1},
    {s_Homing_Offset, 20, sizeof(s_Homing_Offset) - 1, 4},
    {s_Moving_Threshold, 24, sizeof(s_Moving_Threshold) - 1, 4},
    {s_Temperature_Limit, 31, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 32, sizeof(s_Max_Voltage_Limit) - 1, 2},
    {s_Min_Voltage_Limit, 34, sizeof(s_Min_Voltage_Limit) - 1, 2},
    {s_PWM_Limit, 36, sizeof(s_PWM_Limit) - 1, 2},
    {s_Current_Limit, 38, sizeof(s_Current_Limit) - 1, 2},
    {s_Acceleration_Limit, 40, sizeof(s_Acceleration_Limit) - 1, 4},
    {s_Velocity_Limit, 44, sizeof(s_Velocity_Limit) - 1, 4},
    {s_Max_Position_Limit, 48, sizeof(s_Max_Position_Limit) - 1, 4},
    {s_Min_Position_Limit, 52, sizeof(s_Min_Position_Limit) - 1, 4},
    {s_External_Port_Mode_1, 56, sizeof(s_External_Port_Mode_1) - 1, 1},
    {s_External_Port_Mode_2, 57, sizeof(s_External_Port_Mode_2) - 1, 1},
    {s_External_Port_Mode_3, 58, sizeof(s_External_Port_Mode_3) - 1, 1},
    {s_External_Port_Mode_4, 59, sizeof(s_External_Port_Mode_4) - 1, 1},
    {s_Shutdown, 63, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 512, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED_RED, 513, sizeof(s_LED_RED) - 1, 1},
    {s_LED_GREEN, 514, sizeof(s_LED_GREEN) - 1, 1},
    {s_LED_BLUE, 515, sizeof(s_LED_BLUE) - 1, 1},
    {s_Status_Return_Level, 516, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Registered_Instruction, 517, sizeof(s_Registered_Instruction) - 1, 1},
    {s_Hardware_Error_Status, 518, sizeof(s_Hardware_Error_Status) - 1, 1},
    {s_Velocity_I_Gain, 524, sizeof(s_Velocity_I_Gain) - 1, 2},
    {s_Velocity_P_Gain, 526, sizeof(s_Velocity_P_Gain) - 1, 2},
    {s_Position_D_Gain, 528, sizeof(s_Position_D_Gain) - 1, 2},
    {s_Position_I_Gain, 530, sizeof(s_Position_I_Gain) - 1, 2},
    {s_Position_P_Gain, 532, sizeof(s_Position_P_Gain) - 1, 2},
    {s_Feedforward_2nd_Gain, 536, sizeof(s_Feedforward_2nd_Gain) - 1, 2},
    {s_Feedforward_1st_Gain, 538, sizeof(s_Feedforward_1st_Gain) - 1, 2},
    {s_Bus_Watchdog, 546, sizeof(s_Bus_Watchdog) - 1, 1},
    {s_Goal_PWM, 548, sizeof(s_Goal_PWM) - 1, 2},
    {s_Goal_Current, 550, sizeof(s_Goal_Current) - 1, 2},
    {s_Goal_Velocity, 552, sizeof(s_Goal_Velocity) - 1, 4},
    {s_Profile_Acceleration, 556, sizeof(s_Profile_Acceleration) - 1, 4},
    {s_Profile_Velocity, 560, sizeof(s_Profile_Velocity) - 1, 4},
    {s_Goal_Position, 564, sizeof(s_Goal_Position) - 1, 4},    
    {s_Realtime_Tick, 568, sizeof(s_Realtime_Tick) - 1, 2},
    {s_Moving, 570, sizeof(s_Moving) - 1, 1},
    {s_Moving_Status, 571, sizeof(s_Moving) - 1, 1},
    {s_Present_PWM, 572, sizeof(s_Present_PWM) - 1, 2},
    {s_Present_Current, 574, sizeof(s_Present_Current) - 1, 2},
    {s_Present_Velocity, 576, sizeof(s_Present_Velocity) - 1, 4},
    {s_Present_Position, 580, sizeof(s_Present_Position) - 1, 4},
    {s_Velocity_Trajectory, 584, sizeof(s_Velocity_Trajectory) - 1, 4},
    {s_Position_Trajectory, 588, sizeof(s_Position_Trajectory) - 1, 4},    
    {s_Present_Input_Voltage, 592, sizeof(s_Present_Input_Voltage) - 1, 2},
    {s_Present_Temperature, 594, sizeof(s_Present_Temperature) - 1, 1},
    {s_External_Port_Mode_1, 600, sizeof(s_External_Port_Mode_1) - 1, 2},
    {s_External_Port_Mode_2, 602, sizeof(s_External_Port_Mode_2) - 1, 2},
    {s_External_Port_Mode_3, 604, sizeof(s_External_Port_Mode_3) - 1, 2},
    {s_External_Port_Mode_4, 606, sizeof(s_External_Port_Mode_4) - 1, 2}};

#define COUNT_EXTPRO_PLUS_ITEMS (sizeof(items_PRO_PLUS) / sizeof(items_PRO_PLUS[0]))

static const ModelInfo info_PRO_PLUS[] = {
      {0.01, -251173, 0, 251173, -3.14159265, 3.14159265},  // PRO_PLUS_M42P_010_S260_R
      {0.01, -251173, 0, 251173, -3.14159265, 3.14159265},  // PRO_PLUS_M54P_040_S250_R
      {0.01, -262931, 0, 262931, -3.14159265, 3.14159265},  // PRO_PLUS_M54P_060_S250_R
      {0.01, -303454, 0, 303454, -3.14159265, 3.14159265},  // PRO_PLUS_H42P_020_S300_R
      {0.01, -501433, 0, 501433, -3.14159265, 3.14159265},  // PRO_PLUS_H54P_100_S500_R
      {0.01, -501433, 0, 501433, -3.14159265, 3.14159265}}; // PRO_PLUS_H54P_200_S500_R

//---------------------------------------------------------
// Gripper - (num == RH_P12_RN)
//---------------------------------------------------------
static const ControlItem items_Gripper[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 6, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 7, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 8, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 9, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_Operating_Mode, 11, sizeof(s_Operating_Mode) - 1, 1},
    {s_Homing_Offset, 13, sizeof(s_Homing_Offset) - 1, 4},
    {s_Moving_Threshold, 17, sizeof(s_Moving_Threshold) - 1, 4},
    {s_Temperature_Limit, 21, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 22, sizeof(s_Max_Voltage_Limit) - 1, 2},
    {s_Min_Voltage_Limit, 24, sizeof(s_Min_Voltage_Limit) - 1, 2},
    {s_Acceleration_Limit, 26, sizeof(s_Acceleration_Limit) - 1, 4},
    {s_Torque_Limit, 30, sizeof(s_Torque_Limit) - 1, 2},
    {s_Velocity_Limit, 32, sizeof(s_Velocity_Limit) - 1, 4},
    {s_Max_Position_Limit, 36, sizeof(s_Max_Position_Limit) - 1, 4},
    {s_Min_Position_Limit, 40, sizeof(s_Min_Position_Limit) - 1, 4},
    {s_External_Port_Mode_1, 44, sizeof(s_External_Port_Mode_1) - 1, 1},
    {s_External_Port_Mode_2, 45, sizeof(s_External_Port_Mode_2) - 1, 1},
    {s_External_Port_Mode_3, 46, sizeof(s_External_Port_Mode_3) - 1, 1},
    {s_External_Port_Mode_4, 47, sizeof(s_External_Port_Mode_4) - 1, 1},
    {s_Shutdown, 48, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 562, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED_RED, 563, sizeof(s_LED_RED) - 1, 1},
    {s_LED_GREEN, 564, sizeof(s_LED_GREEN) - 1, 1},
    {s_LED_BLUE, 565, sizeof(s_LED_BLUE) - 1, 1},
    {s_Velocity_I_Gain, 586, sizeof(s_Velocity_I_Gain) - 1, 2},
    {s_Velocity_P_Gain, 588, sizeof(s_Velocity_P_Gain) - 1, 2},
    {s_Position_P_Gain, 594, sizeof(s_Position_P_Gain) - 1, 2},
    {s_Goal_Position, 596, sizeof(s_Goal_Position) - 1, 4},
    {s_Goal_Velocity, 600, sizeof(s_Goal_Velocity) - 1, 4},
    {s_Goal_Torque, 604, sizeof(s_Goal_Torque) - 1, 2},
    {s_Goal_Acceleration, 606, sizeof(s_Goal_Acceleration) - 1, 4},
    {s_Moving, 610, sizeof(s_Moving) - 1, 1},
    {s_Present_Position, 611, sizeof(s_Present_Position) - 1, 4},
    {s_Present_Velocity, 615, sizeof(s_Present_Velocity) - 1, 4},
    {s_Present_Current, 621, sizeof(s_Present_Current) - 1, 2},
    {s_Present_Input_Voltage, 623, sizeof(s_Present_Input_Voltage) - 1, 2},
    {s_Present_Temperature, 625, sizeof(s_Present_Temperature) - 1, 1},
    {s_External_Port_Mode_1, 626, sizeof(s_External_Port_Mode_1) - 1, 2},
    {s_External_Port_Mode_2, 628, sizeof(s_External_Port_Mode_2) - 1, 2},
    {s_External_Port_Mode_3, 630, sizeof(s_External_Port_Mode_3) - 1, 2},
    {s_External_Port_Mode_4, 632, sizeof(s_External_Port_Mode_4) - 1, 2},
    {s_Registered_Instruction, 890, sizeof(s_Registered_Instruction) - 1, 1},
    {s_Status_Return_Level, 891, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Hardware_Error_Status, 892, sizeof(s_Hardware_Error_Status) - 1, 1}};
#define COUNT_Gripper_ITEMS (sizeof(items_Gripper) / sizeof(items_Gripper[0]))

static const ModelInfo info_Gripper = {0.01,
                                       0,
                                       0,
                                       740,
                                       0, 
                                       1.1345703125};

//---------------------------------------------------------
// Gripper A Firmware - (num == RH_P12_RN_A)
//---------------------------------------------------------
static const ControlItem items_EXTGripper[]{
    {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
    {s_Firmware_Version, 6, sizeof(s_Firmware_Version) - 1, 1},
    {s_ID, 7, sizeof(s_ID) - 1, 1},
    {s_Baud_Rate, 8, sizeof(s_Baud_Rate) - 1, 1},
    {s_Return_Delay_Time, 9, sizeof(s_Return_Delay_Time) - 1, 1},
    {s_Drive_Mode, 10, sizeof(s_Drive_Mode) - 1, 1},
    {s_Operating_Mode, 11, sizeof(s_Operating_Mode) - 1, 1},
    {s_Secondary_ID, 12, sizeof(s_Secondary_ID) - 1, 1},
    {s_Homing_Offset, 20, sizeof(s_Homing_Offset) - 1, 4},
    {s_Moving_Threshold, 24, sizeof(s_Moving_Threshold) - 1, 4},
    {s_Temperature_Limit, 31, sizeof(s_Temperature_Limit) - 1, 1},
    {s_Max_Voltage_Limit, 32, sizeof(s_Max_Voltage_Limit) - 1, 2},
    {s_Min_Voltage_Limit, 34, sizeof(s_Min_Voltage_Limit) - 1, 2},
    {s_PWM_Limit, 36, sizeof(s_PWM_Limit) - 1, 2},
    {s_Current_Limit, 38, sizeof(s_Current_Limit) - 1, 2},
    {s_Acceleration_Limit, 40, sizeof(s_Acceleration_Limit) - 1, 4},
    {s_Velocity_Limit, 44, sizeof(s_Velocity_Limit) - 1, 4},
    {s_Max_Position_Limit, 48, sizeof(s_Max_Position_Limit) - 1, 4},
    {s_Min_Position_Limit, 52, sizeof(s_Min_Position_Limit) - 1, 4},
    {s_External_Port_Mode_1, 56, sizeof(s_External_Port_Mode_1) - 1, 1},
    {s_External_Port_Mode_2, 57, sizeof(s_External_Port_Mode_2) - 1, 1},
    {s_External_Port_Mode_3, 58, sizeof(s_External_Port_Mode_3) - 1, 1},
    {s_External_Port_Mode_4, 59, sizeof(s_External_Port_Mode_4) - 1, 1},
    {s_Shutdown, 63, sizeof(s_Shutdown) - 1, 1},

    {s_Torque_Enable, 512, sizeof(s_Torque_Enable) - 1, 1},
    {s_LED_RED, 513, sizeof(s_LED_RED) - 1, 1},
    {s_LED_GREEN, 514, sizeof(s_LED_GREEN) - 1, 1},
    {s_LED_BLUE, 515, sizeof(s_LED_BLUE) - 1, 1},
    {s_Status_Return_Level, 516, sizeof(s_Status_Return_Level) - 1, 1},
    {s_Registered_Instruction, 517, sizeof(s_Registered_Instruction) - 1, 1},
    {s_Hardware_Error_Status, 518, sizeof(s_Hardware_Error_Status) - 1, 1},
    {s_Velocity_I_Gain, 524, sizeof(s_Velocity_I_Gain) - 1, 2},
    {s_Velocity_P_Gain, 526, sizeof(s_Velocity_P_Gain) - 1, 2},
    {s_Position_D_Gain, 528, sizeof(s_Position_D_Gain) - 1, 2},
    {s_Position_I_Gain, 530, sizeof(s_Position_I_Gain) - 1, 2},
    {s_Position_P_Gain, 532, sizeof(s_Position_P_Gain) - 1, 2},
    {s_Feedforward_2nd_Gain, 536, sizeof(s_Feedforward_2nd_Gain) - 1, 2},
    {s_Feedforward_1st_Gain, 538, sizeof(s_Feedforward_1st_Gain) - 1, 2},
    {s_Bus_Watchdog, 546, sizeof(s_Bus_Watchdog) - 1, 1},
    {s_Goal_PWM, 548, sizeof(s_Goal_PWM) - 1, 2},
    {s_Goal_Current, 550, sizeof(s_Goal_Current) - 1, 2},
    {s_Goal_Velocity, 552, sizeof(s_Goal_Velocity) - 1, 4},
    {s_Profile_Acceleration, 556, sizeof(s_Profile_Acceleration) - 1, 4},
    {s_Profile_Velocity, 560, sizeof(s_Profile_Velocity) - 1, 4},
    {s_Goal_Position, 564, sizeof(s_Goal_Position) - 1, 4},    
    {s_Realtime_Tick, 568, sizeof(s_Realtime_Tick) - 1, 2},
    {s_Moving, 570, sizeof(s_Moving) - 1, 1},
    {s_Moving_Status, 571, sizeof(s_Moving) - 1, 1},
    {s_Present_PWM, 572, sizeof(s_Present_PWM) - 1, 2},
    {s_Present_Current, 574, sizeof(s_Present_Current) - 1, 2},
    {s_Present_Velocity, 576, sizeof(s_Present_Velocity) - 1, 4},
    {s_Present_Position, 580, sizeof(s_Present_Position) - 1, 4},
    {s_Velocity_Trajectory, 584, sizeof(s_Velocity_Trajectory) - 1, 4},
    {s_Position_Trajectory, 588, sizeof(s_Position_Trajectory) - 1, 4},    
    {s_Present_Input_Voltage, 592, sizeof(s_Present_Input_Voltage) - 1, 2},
    {s_Present_Temperature, 594, sizeof(s_Present_Temperature) - 1, 1},
    {s_External_Port_Mode_1, 600, sizeof(s_External_Port_Mode_1) - 1, 2},
    {s_External_Port_Mode_2, 602, sizeof(s_External_Port_Mode_2) - 1, 2},
    {s_External_Port_Mode_3, 604, sizeof(s_External_Port_Mode_3) - 1, 2},
    {s_External_Port_Mode_4, 606, sizeof(s_External_Port_Mode_4) - 1, 2}};
#define COUNT_EXTGripper_ITEMS (sizeof(items_EXTGripper) / sizeof(items_EXTGripper[0]))

static const ModelInfo info_EXTGripper = {0.01,
                                          0,
                                          0,
                                          740,
                                          0, 
                                          1.1345703125};

//=========================================================
// Get Servo control table for the specified servo type
//=========================================================
static uint8_t the_number_of_item = 0;
const ControlItem *DynamixelItem::getControlTable(uint16_t model_number)
{
  uint16_t num = model_number;

  const ControlItem *control_table;
  if (num == AX_12A || num == AX_12W || num == AX_18A)
  {
    control_table = items_AX;
    the_number_of_item = COUNT_AX_ITEMS;
  }
  else if (num == RX_10 || num == RX_24F || num == RX_28 || num == RX_64)
  {
    control_table = items_RX;
    the_number_of_item = COUNT_RX_ITEMS;
  }
  else if (num == EX_106)
  {
    control_table = items_EX;
    the_number_of_item = COUNT_EX_ITEMS;
  }
  else if (num == MX_12W || num == MX_28)
  {
    control_table = items_MX;
    the_number_of_item = COUNT_MX_ITEMS;
  }
  else if (num == MX_64 || num == MX_106)
  {
    control_table = items_EXTMX;
    the_number_of_item = COUNT_EXTMX_ITEMS;
  }
  else if (num == MX_28_2)
  {
    control_table = items_MX2;
    the_number_of_item = COUNT_MX2_ITEMS;
  }
  else if (num == MX_64_2 || num == MX_106_2)
  {
    control_table = items_EXTMX2;
    the_number_of_item = COUNT_EXTMX2_ITEMS;
  }
  else if (num == XL_320)
  {
    control_table = items_XL320;
    the_number_of_item = COUNT_XL320_ITEMS;
  }
  else if (num == XL430_W250 || num == XL430_W250_2 || num == XC430_W150 || num == XC430_W240)
  {
    control_table = items_XL;
    the_number_of_item = COUNT_XL_ITEMS;
  }
  else if (num == XM430_W210 || num == XM430_W350)
  {
    control_table = items_XM;
    the_number_of_item = COUNT_XM_ITEMS;
  }
  else if (num == XM540_W150 || num == XM540_W270)
  {
    control_table = items_EXTXM;
    the_number_of_item = COUNT_EXTXM_ITEMS;
  }
  else if (num == XH430_V210 || num == XH430_V350 || num == XH430_W210 || num == XH430_W350)
  {
    control_table = items_XH;
    the_number_of_item = COUNT_XH_ITEMS;
  }
  else if (num == XH540_W150 || num == XH540_W270 || num == XH540_V150 || num == XH540_V270)
  {
    control_table = items_EXTXH;
    the_number_of_item = COUNT_EXTXH_ITEMS;
  }
  else if (num == PRO_L42_10_S300_R)
  {
    control_table = items_PRO;
    the_number_of_item = COUNT_PRO_ITEMS;
  }
  else if (num == PRO_L54_30_S400_R || num == PRO_L54_30_S500_R || num == PRO_L54_50_S290_R || num == PRO_L54_50_S500_R ||
           num == PRO_M42_10_S260_R || num == PRO_M54_40_S250_R || num == PRO_M54_60_S250_R ||
           num == PRO_H42_20_S300_R || num == PRO_H54_100_S500_R || num == PRO_H54_200_S500_R)
  {
    control_table = items_EXTPRO;
    the_number_of_item = COUNT_EXTPRO_ITEMS;
  }
  else if (num == PRO_M42_10_S260_R_A || num == PRO_M54_40_S250_R_A  || num == PRO_M54_60_S250_R_A ||
           num == PRO_H42_20_S300_R_A || num == PRO_H54_100_S500_R_A || num == PRO_H54_200_S500_R_A)
  {
    control_table = items_EXTPRO_A;
    the_number_of_item = COUNT_EXTPRO_A_ITEMS;
  }
  else if (num == PRO_PLUS_H42P_020_S300_R || num == PRO_PLUS_H54P_100_S500_R || num == PRO_PLUS_H54P_200_S500_R ||
           num == PRO_PLUS_M42P_010_S260_R || num == PRO_PLUS_M54P_040_S250_R || num == PRO_PLUS_M54P_060_S250_R)
  {
    control_table = items_PRO_PLUS;
    the_number_of_item = COUNT_EXTPRO_PLUS_ITEMS;
  }
  else if (num == RH_P12_RN)
  {
    control_table = items_Gripper;
    the_number_of_item = COUNT_Gripper_ITEMS;
  }
  else if (num == RH_P12_RN_A)
  {
    control_table = items_EXTGripper;
    the_number_of_item = COUNT_EXTGripper_ITEMS;
  }
  else
  {
    control_table = NULL;
    the_number_of_item = 0;
  }

  return control_table;
}

const ModelInfo *DynamixelItem::getModelInfo(uint16_t model_number)
{
  uint16_t num = model_number;

  const ModelInfo *info;

  if (num == AX_12A || num == AX_12W || num == AX_18A)
  {
    info = &info_AX;
  }
  else if (num == RX_10 || num == RX_24F || num == RX_28 || num == RX_64)
  {
    info = &info_RX;
  }
  else if (num == EX_106)
  {
    info = &info_EX;
  }
  else if (num == MX_12W || num == MX_28)
  {
    info = &info_MX;
  }
  else if (num == MX_64 || num == MX_106)
  {
    info = &info_EXTMX;
  }
  else if (num == MX_28_2)
  {
    info = &info_MX2;
  }
  else if (num == MX_64_2 || num == MX_106_2)
  {
    info = &info_EXTMX2;
  }
  else if (num == XL_320)
  {
    info = &info_XL320;
  }
  else if (num == XL430_W250 || num == XL430_W250_2 || num == XC430_W150 || num == XC430_W240)
  {
    info = &info_XL;
  }
  else if (num == XM430_W210 || num == XM430_W350)
  {
    info = &info_XM;
  }
  else if (num == XM540_W150 || num == XM540_W270)
  {
    info = &info_EXTXM;
  }
  else if (num == XH430_V210 || num == XH430_V350 || num == XH430_W210 || num == XH430_W350)
  {
    info = &info_XH;
  }
  else if (num == XH540_W150 || num == XH540_W270 || num == XH540_V150 || num == XH540_V270)
  {
    info = &info_EXTXH;
  }
  else if (num == PRO_L42_10_S300_R)
  {
    info = &info_PRO;
  } 
  else if (num == PRO_L54_30_S400_R)
  {
    info = &info_EXTPRO[0];
  }
  else if (num == PRO_L54_30_S500_R || num == PRO_L54_50_S500_R)
  {
    info = &info_EXTPRO[1];
  }
  else if (num == PRO_L54_50_S290_R)
  {
    info = &info_EXTPRO[2];
  }
  else if (num == PRO_M42_10_S260_R)
  {
    info = &info_EXTPRO[3];
  }
  else if (num == PRO_M54_40_S250_R || num == PRO_M54_60_S250_R)
  {
    info = &info_EXTPRO[4];
  }
  else if (num == PRO_H42_20_S300_R)
  {
    info = &info_EXTPRO[5];
  }
  else if (num == PRO_H54_100_S500_R || num == PRO_H54_200_S500_R)
  { 
    info = &info_EXTPRO[6];
  }
  else if (num == PRO_M42_10_S260_R_A)
  {
    info = &info_EXTPRO_A[0];
  }
  else if (num == PRO_M54_40_S250_R_A || num == PRO_M54_60_S250_R_A)
  {
    info = &info_EXTPRO_A[1];
  }
  else if (num == PRO_H42_20_S300_R_A)
  {
    info = &info_EXTPRO_A[2];
  }
  else if (num == PRO_H54_100_S500_R_A || num == PRO_H54_200_S500_R_A)
  { 
    info = &info_EXTPRO_A[3];
  }
  else if (num == PRO_PLUS_H42P_020_S300_R)
  { 
    info = &info_PRO_PLUS[0];
  }
  else if (num == PRO_PLUS_H54P_100_S500_R)
  { 
    info = &info_PRO_PLUS[1];
  }
  else if (num == PRO_PLUS_H54P_200_S500_R)
  { 
    info = &info_PRO_PLUS[2];
  }
  else if (num == PRO_PLUS_M42P_010_S260_R)
  { 
    info = &info_PRO_PLUS[3];
  }
  else if (num == PRO_PLUS_M54P_040_S250_R)
  { 
    info = &info_PRO_PLUS[4];
  }
  else if (num == PRO_PLUS_M54P_060_S250_R)
  { 
    info = &info_PRO_PLUS[5];
  }
  else if (num == RH_P12_RN)
  {
    info = &info_Gripper;
  } 
  else if (num == RH_P12_RN_A)
  {
    info = &info_EXTGripper;
  }
  else
  {
    info = NULL;
  }

  return info;
}

uint8_t DynamixelItem::getTheNumberOfControlItem()
{
  return the_number_of_item;
}
