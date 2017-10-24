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

#include "../../include/dynamixel_workbench/dynamixel_item.h"

static uint8_t control_table_size = 0;
static ControlTableItem item[60]  = {0, };

static ModelInfo model_info       = {0.0, };

void setAXItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1 , READ_WRITE , EEPROM};
  item[1]  = {4  , "Baud Rate"                     , 1 , READ_WRITE , EEPROM};
  item[2]  = {6  , "CW Angle Limit"                , 2 , READ_WRITE , EEPROM};
  item[3]  = {8  , "CCW Angle Limit"               , 2 , READ_WRITE , EEPROM};

  item[4]  = {24 , "Torque ON/OFF"                 , 1 , READ_WRITE , RAM};
  item[5]  = {25 , "LED"                           , 1 , READ_WRITE , RAM};
  item[6]  = {26 , "CW Compliance Margin"          , 1 , READ_WRITE , RAM};
  item[7]  = {27 , "CCW Compliance Margin"         , 1 , READ_WRITE , RAM};
  item[8]  = {28 , "CW Compliance Slope"           , 1 , READ_WRITE , RAM};
  item[9]  = {29 , "CCW Compliance Slope"          , 1 , READ_WRITE , RAM};
  item[10] = {30 , "Goal Position"                 , 2 , READ_WRITE , RAM};
  item[11] = {32 , "Moving Speed"                  , 2 , READ_WRITE , RAM};
  item[12] = {35 , "Torque Limit"                  , 2 , READ_WRITE , RAM};
  item[13] = {36 , "Present Position"              , 2 , READ       , RAM};
  item[14] = {38 , "Present Speed"                 , 2 , READ       , RAM};
  item[15] = {40 , "Present Load"                  , 2 , READ       , RAM};
  item[16] = {42 , "Present Voltage"               , 1 , READ       , RAM};
  item[17] = {43 , "Present Temperature"           , 1 , READ       , RAM};
  item[18] = {44 , "Registered"                    , 1 , READ       , RAM};
  item[19] = {46 , "Moving"                        , 1 , READ       , RAM};
  item[20] = {47 , "Lock"                          , 1 , READ_WRITE , RAM};
  item[21] = {48 , "Punch"                         , 2 , READ_WRITE , RAM};

  control_table_size = 22;
#else
  item[0]  = {0  , "Model Number"                  , 2 , READ       , EEPROM};
  item[1]  = {2  , "Version of Firmware"           , 1 , READ       , EEPROM};
  item[2]  = {3  , "ID"                            , 1 , READ_WRITE , EEPROM};
  item[3]  = {4  , "Baud Rate"                     , 1 , READ_WRITE , EEPROM};
  item[4]  = {5  , "Return Delay Time"             , 1 , READ_WRITE , EEPROM};
  item[5]  = {6  , "CW Angle Limit"                , 2 , READ_WRITE , EEPROM};
  item[6]  = {8  , "CCW Angle Limit"               , 2 , READ_WRITE , EEPROM};
  item[7]  = {11 , "the Highest Limit Temperature" , 1 , READ_WRITE , EEPROM};
  item[8]  = {12 , "the Lowest Limit Voltage"      , 1 , READ_WRITE , EEPROM};
  item[9]  = {13 , "the Highest Limit Voltage"     , 1 , READ_WRITE , EEPROM};
  item[10] = {14 , "Max Torque"                    , 2 , READ_WRITE , EEPROM};
  item[11] = {16 , "Status Return Level"           , 1 , READ_WRITE , EEPROM};
  item[12] = {17 , "Alarm LED"                     , 1 , READ_WRITE , EEPROM};
  item[13] = {18 , "Alarm Shutdown"                , 1 , READ_WRITE , EEPROM};

  item[14] = {24 , "Torque ON/OFF"                 , 1 , READ_WRITE , RAM};
  item[15] = {25 , "LED"                           , 1 , READ_WRITE , RAM};
  item[16] = {26 , "CW Compliance Margin"          , 1 , READ_WRITE , RAM};
  item[17] = {27 , "CCW Compliance Margin"         , 1 , READ_WRITE , RAM};
  item[18] = {28 , "CW Compliance Slope"           , 1 , READ_WRITE , RAM};
  item[19] = {29 , "CCW Compliance Slope"          , 1 , READ_WRITE , RAM};
  item[20] = {30 , "Goal Position"                 , 2 , READ_WRITE , RAM};
  item[21] = {32 , "Moving Speed"                  , 2 , READ_WRITE , RAM};
  item[22] = {35 , "Torque Limit"                  , 2 , READ_WRITE , RAM};
  item[23] = {36 , "Present Position"              , 2 , READ       , RAM};
  item[24] = {38 , "Present Speed"                 , 2 , READ       , RAM};
  item[25] = {40 , "Present Load"                  , 2 , READ       , RAM};
  item[26] = {42 , "Present Voltage"               , 1 , READ       , RAM};
  item[27] = {43 , "Present Temperature"           , 1 , READ       , RAM};
  item[28] = {44 , "Registered"                    , 1 , READ       , RAM};
  item[29] = {46 , "Moving"                        , 1 , READ       , RAM};
  item[30] = {47 , "Lock"                          , 1 , READ_WRITE , RAM};
  item[31] = {48 , "Punch"                         , 2 , READ_WRITE , RAM};

  control_table_size = 32;
#endif  
}

void setAXInfo()
{
  model_info.velocity_to_value_ratio         = 86.03;
  
  model_info.value_of_0_radian_position      = 512;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 1023;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

void setRXItem()
{
  #if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1 , READ_WRITE , EEPROM};
  item[1]  = {4  , "Baud Rate"                     , 1 , READ_WRITE , EEPROM};
  item[2]  = {6  , "CW Angle Limit"                , 2 , READ_WRITE , EEPROM};
  item[3]  = {8  , "CCW Angle Limit"               , 2 , READ_WRITE , EEPROM};

  item[4]  = {24 , "Torque ON/OFF"                 , 1 , READ_WRITE , RAM};
  item[5]  = {25 , "LED"                           , 1 , READ_WRITE , RAM};
  item[6]  = {26 , "CW Compliance Margin"          , 1 , READ_WRITE , RAM};
  item[7]  = {27 , "CCW Compliance Margin"         , 1 , READ_WRITE , RAM};
  item[8]  = {28 , "CW Compliance Slope"           , 1 , READ_WRITE , RAM};
  item[9]  = {29 , "CCW Compliance Slope"          , 1 , READ_WRITE , RAM};
  item[10] = {30 , "Goal Position"                 , 2 , READ_WRITE , RAM};
  item[11] = {32 , "Moving Speed"                  , 2 , READ_WRITE , RAM};
  item[12] = {35 , "Torque Limit"                  , 2 , READ_WRITE , RAM};
  item[13] = {36 , "Present Position"              , 2 , READ       , RAM};
  item[14] = {38 , "Present Speed"                 , 2 , READ       , RAM};
  item[15] = {40 , "Present Load"                  , 2 , READ       , RAM};
  item[16] = {42 , "Present Voltage"               , 1 , READ       , RAM};
  item[17] = {43 , "Present Temperature"           , 1 , READ       , RAM};
  item[18] = {44 , "Registered"                    , 1 , READ       , RAM};
  item[19] = {46 , "Moving"                        , 1 , READ       , RAM};
  item[20] = {47 , "Lock"                          , 1 , READ_WRITE , RAM};
  item[21] = {48 , "Punch"                         , 2 , READ_WRITE , RAM};

  control_table_size = 22;
#else
  item[0]  = {0  , "Model Number"                  , 2 , READ       , EEPROM};
  item[1]  = {2  , "Version of Firmware"           , 1 , READ       , EEPROM};
  item[2]  = {3  , "ID"                            , 1 , READ_WRITE , EEPROM};
  item[3]  = {4  , "Baud Rate"                     , 1 , READ_WRITE , EEPROM};
  item[4]  = {5  , "Return Delay Time"             , 1 , READ_WRITE , EEPROM};
  item[5]  = {6  , "CW Angle Limit"                , 2 , READ_WRITE , EEPROM};
  item[6]  = {8  , "CCW Angle Limit"               , 2 , READ_WRITE , EEPROM};
  item[7]  = {11 , "the Highest Limit Temperature" , 1 , READ_WRITE , EEPROM};
  item[8]  = {12 , "the Lowest Limit Voltage"      , 1 , READ_WRITE , EEPROM};
  item[9]  = {13 , "the Highest Limit Voltage"     , 1 , READ_WRITE , EEPROM};
  item[10] = {14 , "Max Torque"                    , 2 , READ_WRITE , EEPROM};
  item[11] = {16 , "Status Return Level"           , 1 , READ_WRITE , EEPROM};
  item[12] = {17 , "Alarm LED"                     , 1 , READ_WRITE , EEPROM};
  item[13] = {18 , "Alarm Shutdown"                , 1 , READ_WRITE , EEPROM};

  item[14] = {24 , "Torque ON/OFF"                 , 1 , READ_WRITE , RAM};
  item[15] = {25 , "LED"                           , 1 , READ_WRITE , RAM};
  item[16] = {26 , "CW Compliance Margin"          , 1 , READ_WRITE , RAM};
  item[17] = {27 , "CCW Compliance Margin"         , 1 , READ_WRITE , RAM};
  item[18] = {28 , "CW Compliance Slope"           , 1 , READ_WRITE , RAM};
  item[19] = {29 , "CCW Compliance Slope"          , 1 , READ_WRITE , RAM};
  item[20] = {30 , "Goal Position"                 , 2 , READ_WRITE , RAM};
  item[21] = {32 , "Moving Speed"                  , 2 , READ_WRITE , RAM};
  item[22] = {35 , "Torque Limit"                  , 2 , READ_WRITE , RAM};
  item[23] = {36 , "Present Position"              , 2 , READ       , RAM};
  item[24] = {38 , "Present Speed"                 , 2 , READ       , RAM};
  item[25] = {40 , "Present Load"                  , 2 , READ       , RAM};
  item[26] = {42 , "Present Voltage"               , 1 , READ       , RAM};
  item[27] = {43 , "Present Temperature"           , 1 , READ       , RAM};
  item[28] = {44 , "Registered"                    , 1 , READ       , RAM};
  item[29] = {46 , "Moving"                        , 1 , READ       , RAM};
  item[30] = {47 , "Lock"                          , 1 , READ_WRITE , RAM};
  item[31] = {48 , "Punch"                         , 2 , READ_WRITE , RAM};

  control_table_size = 32;
#endif  
}

void setRXInfo()
{
  model_info.velocity_to_value_ratio         = 86.03;
  
  model_info.value_of_0_radian_position      = 512;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 1023;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

void setEXItem()
{
  #if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1 , READ_WRITE , EEPROM};
  item[1]  = {4  , "Baud Rate"                     , 1 , READ_WRITE , EEPROM};
  item[2]  = {6  , "CW Angle Limit"                , 2 , READ_WRITE , EEPROM};
  item[3]  = {8  , "CCW Angle Limit"               , 2 , READ_WRITE , EEPROM};
  item[4]  = {10 , "Drive Mode"                    , 1 , READ_WRITE , EEPROM};

  item[5]  = {24 , "Torque ON/OFF"                 , 1 , READ_WRITE , RAM};
  item[6]  = {25 , "LED"                           , 1 , READ_WRITE , RAM};
  item[7]  = {26 , "CW Compliance Margin"          , 1 , READ_WRITE , RAM};
  item[8]  = {27 , "CCW Compliance Margin"         , 1 , READ_WRITE , RAM};
  item[9]  = {28 , "CW Compliance Slope"           , 1 , READ_WRITE , RAM};
  item[10] = {29 , "CCW Compliance Slope"          , 1 , READ_WRITE , RAM};
  item[11] = {30 , "Goal Position"                 , 2 , READ_WRITE , RAM};
  item[12] = {32 , "Moving Speed"                  , 2 , READ_WRITE , RAM};
  item[13] = {35 , "Torque Limit"                  , 2 , READ_WRITE , RAM};
  item[14] = {36 , "Present Position"              , 2 , READ       , RAM};
  item[15] = {38 , "Present Speed"                 , 2 , READ       , RAM};
  item[16] = {40 , "Present Load"                  , 2 , READ       , RAM};
  item[17] = {42 , "Present Voltage"               , 1 , READ       , RAM};
  item[18] = {43 , "Present Temperature"           , 1 , READ       , RAM};
  item[19] = {44 , "Registered"                    , 1 , READ       , RAM};
  item[20] = {46 , "Moving"                        , 1 , READ       , RAM};
  item[21] = {47 , "Lock"                          , 1 , READ_WRITE , RAM};
  item[22] = {48 , "Punch"                         , 2 , READ_WRITE , RAM};
  item[23] = {56 , "Sensored Current"              , 2 , READ       , RAM};

  control_table_size = 24;
#else
  item[0]  = {0  , "Model Number"                  , 2 , READ       , EEPROM};
  item[1]  = {2  , "Version of Firmware"           , 1 , READ       , EEPROM};
  item[2]  = {3  , "ID"                            , 1 , READ_WRITE , EEPROM};
  item[3]  = {4  , "Baud Rate"                     , 1 , READ_WRITE , EEPROM};
  item[4]  = {5  , "Return Delay Time"             , 1 , READ_WRITE , EEPROM};
  item[5]  = {6  , "CW Angle Limit"                , 2 , READ_WRITE , EEPROM};
  item[6]  = {8  , "CCW Angle Limit"               , 2 , READ_WRITE , EEPROM};
  item[7]  = {10 , "Drive Mode"                    , 1 , READ_WRITE , EEPROM};
  item[8]  = {11 , "the Highest Limit Temperature" , 1 , READ_WRITE , EEPROM};
  item[9]  = {12 , "the Lowest Limit Voltage"      , 1 , READ_WRITE , EEPROM};
  item[10] = {13 , "the Highest Limit Voltage"     , 1 , READ_WRITE , EEPROM};
  item[11] = {14 , "Max Torque"                    , 2 , READ_WRITE , EEPROM};
  item[12] = {16 , "Status Return Level"           , 1 , READ_WRITE , EEPROM};
  item[13] = {17 , "Alarm LED"                     , 1 , READ_WRITE , EEPROM};
  item[14] = {18 , "Alarm Shutdown"                , 1 , READ_WRITE , EEPROM};

  item[15] = {24 , "Torque ON/OFF"                 , 1 , READ_WRITE , RAM};
  item[16] = {25 , "LED"                           , 1 , READ_WRITE , RAM};
  item[17] = {26 , "CW Compliance Margin"          , 1 , READ_WRITE , RAM};
  item[18] = {27 , "CCW Compliance Margin"         , 1 , READ_WRITE , RAM};
  item[19] = {28 , "CW Compliance Slope"           , 1 , READ_WRITE , RAM};
  item[20] = {29 , "CCW Compliance Slope"          , 1 , READ_WRITE , RAM};
  item[21] = {30 , "Goal Position"                 , 2 , READ_WRITE , RAM};
  item[22] = {32 , "Moving Speed"                  , 2 , READ_WRITE , RAM};
  item[23] = {35 , "Torque Limit"                  , 2 , READ_WRITE , RAM};
  item[24] = {36 , "Present Position"              , 2 , READ       , RAM};
  item[25] = {38 , "Present Speed"                 , 2 , READ       , RAM};
  item[26] = {40 , "Present Load"                  , 2 , READ       , RAM};
  item[27] = {42 , "Present Voltage"               , 1 , READ       , RAM};
  item[28] = {43 , "Present Temperature"           , 1 , READ       , RAM};
  item[29] = {44 , "Registered"                    , 1 , READ       , RAM};
  item[30] = {46 , "Moving"                        , 1 , READ       , RAM};
  item[31] = {47 , "Lock"                          , 1 , READ_WRITE , RAM};
  item[32] = {48 , "Punch"                         , 2 , READ_WRITE , RAM};
  item[33] = {56 , "Sensored Current"              , 2 , READ       , RAM};

  control_table_size = 34;
#endif  
}

void setEXInfo()
{
  model_info.velocity_to_value_ratio         = 86.03;
  
  model_info.value_of_0_radian_position      = 2048;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 4095;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

void setMXItem()
{
  #if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1 , READ_WRITE , EEPROM};
  item[1]  = {4  , "Baud Rate"                     , 1 , READ_WRITE , EEPROM};
  item[2]  = {6  , "CW Angle Limit"                , 2 , READ_WRITE , EEPROM};
  item[3]  = {8  , "CCW Angle Limit"               , 2 , READ_WRITE , EEPROM};

  item[4]  = {24 , "Torque ON/OFF"                 , 1 , READ_WRITE , RAM};
  item[5]  = {25 , "LED"                           , 1 , READ_WRITE , RAM};
  item[6]  = {26 , "D gain"                        , 1 , READ_WRITE , RAM};
  item[7]  = {27 , "I gain"                        , 1 , READ_WRITE , RAM};
  item[8]  = {28 , "P gain"                        , 1 , READ_WRITE , RAM};
  item[9]  = {30 , "Goal Position"                 , 2 , READ_WRITE , RAM};
  item[10] = {32 , "Moving Speed"                  , 2 , READ_WRITE , RAM};
  item[11] = {35 , "Torque Limit"                  , 2 , READ_WRITE , RAM};
  item[12] = {36 , "Present Position"              , 2 , READ       , RAM};
  item[13] = {38 , "Present Speed"                 , 2 , READ       , RAM};
  item[14] = {40 , "Present Load"                  , 2 , READ       , RAM};
  item[15] = {42 , "Present Voltage"               , 1 , READ       , RAM};
  item[16] = {43 , "Present Temperature"           , 1 , READ       , RAM};
  item[17] = {44 , "Registered"                    , 1 , READ       , RAM};
  item[18] = {46 , "Moving"                        , 1 , READ       , RAM};
  item[19] = {47 , "Lock"                          , 1 , READ_WRITE , RAM};
  item[20] = {48 , "Punch"                         , 2 , READ_WRITE , RAM};
  item[21] = {73 , "Goal Acceleration"             , 1 , READ_WRITE , RAM};

  control_table_size = 22;
#else
  item[0]  = {0  , "Model Number"                  , 2 , READ       , EEPROM};
  item[1]  = {2  , "Version of Firmware"           , 1 , READ       , EEPROM};
  item[2]  = {3  , "ID"                            , 1 , READ_WRITE , EEPROM};
  item[3]  = {4  , "Baud Rate"                     , 1 , READ_WRITE , EEPROM};
  item[4]  = {5  , "Return Delay Time"             , 1 , READ_WRITE , EEPROM};
  item[5]  = {6  , "CW Angle Limit"                , 2 , READ_WRITE , EEPROM};
  item[6]  = {8  , "CCW Angle Limit"               , 2 , READ_WRITE , EEPROM};
  item[7]  = {11 , "the Highest Limit Temperature" , 1 , READ_WRITE , EEPROM};
  item[8]  = {12 , "the Lowest Limit Voltage"      , 1 , READ_WRITE , EEPROM};
  item[9]  = {13 , "the Highest Limit Voltage"     , 1 , READ_WRITE , EEPROM};
  item[10] = {14 , "Max Torque"                    , 2 , READ_WRITE , EEPROM};
  item[11] = {16 , "Status Return Level"           , 1 , READ_WRITE , EEPROM};
  item[12] = {17 , "Alarm LED"                     , 1 , READ_WRITE , EEPROM};
  item[13] = {18 , "Alarm Shutdown"                , 1 , READ_WRITE , EEPROM};
  item[14] = {20 , "Multi Turn Offset"             , 2 , READ_WRITE , EEPROM};
  item[15] = {22 , "Resolution Divider"            , 1 , READ_WRITE , EEPROM};

  item[16] = {24 , "Torque ON/OFF"                 , 1 , READ_WRITE , RAM};
  item[17] = {25 , "LED"                           , 1 , READ_WRITE , RAM};
  item[18] = {26 , "D gain"                        , 1 , READ_WRITE , RAM};
  item[19] = {27 , "I gain"                        , 1 , READ_WRITE , RAM};
  item[20] = {28 , "P gain"                        , 1 , READ_WRITE , RAM};
  item[21] = {30 , "Goal Position"                 , 2 , READ_WRITE , RAM};
  item[22] = {32 , "Moving Speed"                  , 2 , READ_WRITE , RAM};
  item[23] = {35 , "Torque Limit"                  , 2 , READ_WRITE , RAM};
  item[24] = {36 , "Present Position"              , 2 , READ       , RAM};
  item[25] = {38 , "Present Speed"                 , 2 , READ       , RAM};
  item[26] = {40 , "Present Load"                  , 2 , READ       , RAM};
  item[27] = {42 , "Present Voltage"               , 1 , READ       , RAM};
  item[28] = {43 , "Present Temperature"           , 1 , READ       , RAM};
  item[29] = {44 , "Registered"                    , 1 , READ       , RAM};
  item[30] = {46 , "Moving"                        , 1 , READ       , RAM};
  item[31] = {47 , "Lock"                          , 1 , READ_WRITE , RAM};
  item[32] = {48 , "Punch"                         , 2 , READ_WRITE , RAM};
  item[33] = {73 , "Goal Acceleration"             , 1 , READ_WRITE , RAM};

  control_table_size = 34;
#endif  
}

void setMXInfo()
{
  model_info.velocity_to_value_ratio         = 83.77;
  
  model_info.value_of_0_radian_position      = 2048;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 4095;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

void setExtMXItem()
{
  #if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1 , READ_WRITE , EEPROM};
  item[1]  = {4  , "Baud Rate"                     , 1 , READ_WRITE , EEPROM};
  item[2]  = {6  , "CW Angle Limit"                , 2 , READ_WRITE , EEPROM};
  item[3]  = {8  , "CCW Angle Limit"               , 2 , READ_WRITE , EEPROM};

  item[4]  = {24 , "Torque ON/OFF"                 , 1 , READ_WRITE , RAM};
  item[5]  = {25 , "LED"                           , 1 , READ_WRITE , RAM};
  item[6]  = {26 , "D gain"                        , 1 , READ_WRITE , RAM};
  item[7]  = {27 , "I gain"                        , 1 , READ_WRITE , RAM};
  item[8]  = {28 , "P gain"                        , 1 , READ_WRITE , RAM};
  item[9]  = {30 , "Goal Position"                 , 2 , READ_WRITE , RAM};
  item[10] = {32 , "Moving Speed"                  , 2 , READ_WRITE , RAM};
  item[11] = {35 , "Torque Limit"                  , 2 , READ_WRITE , RAM};
  item[12] = {36 , "Present Position"              , 2 , READ       , RAM};
  item[13] = {38 , "Present Speed"                 , 2 , READ       , RAM};
  item[14] = {40 , "Present Load"                  , 2 , READ       , RAM};
  item[15] = {42 , "Present Voltage"               , 1 , READ       , RAM};
  item[16] = {43 , "Present Temperature"           , 1 , READ       , RAM};
  item[17] = {44 , "Registered"                    , 1 , READ       , RAM};
  item[18] = {46 , "Moving"                        , 1 , READ       , RAM};
  item[19] = {47 , "Lock"                          , 1 , READ_WRITE , RAM};
  item[20] = {48 , "Punch"                         , 2 , READ_WRITE , RAM};
  item[21] = {68 , "Current"                       , 2 , READ       , RAM};
  item[22] = {70 , "Torque Control Mode Enable"    , 1 , READ_WRITE , RAM};
  item[23] = {71 , "Goal Torque"                   , 2 , READ_WRITE , RAM};
  item[24] = {73 , "Goal Acceleration"             , 1 , READ_WRITE , RAM};

  control_table_size = 25;
#else
  item[0]  = {0  , "Model Number"                  , 2 , READ       , EEPROM};
  item[1]  = {2  , "Version of Firmware"           , 1 , READ       , EEPROM};
  item[2]  = {3  , "ID"                            , 1 , READ_WRITE , EEPROM};
  item[3]  = {4  , "Baud Rate"                     , 1 , READ_WRITE , EEPROM};
  item[4]  = {5  , "Return Delay Time"             , 1 , READ_WRITE , EEPROM};
  item[5]  = {6  , "CW Angle Limit"                , 2 , READ_WRITE , EEPROM};
  item[6]  = {8  , "CCW Angle Limit"               , 2 , READ_WRITE , EEPROM};
  item[7]  = {11 , "the Highest Limit Temperature" , 1 , READ_WRITE , EEPROM};
  item[8]  = {12 , "the Lowest Limit Voltage"      , 1 , READ_WRITE , EEPROM};
  item[9]  = {13 , "the Highest Limit Voltage"     , 1 , READ_WRITE , EEPROM};
  item[10] = {14 , "Max Torque"                    , 2 , READ_WRITE , EEPROM};
  item[11] = {16 , "Status Return Level"           , 1 , READ_WRITE , EEPROM};
  item[12] = {17 , "Alarm LED"                     , 1 , READ_WRITE , EEPROM};
  item[13] = {18 , "Alarm Shutdown"                , 1 , READ_WRITE , EEPROM};
  item[14] = {20 , "Multi Turn Offset"             , 2 , READ_WRITE , EEPROM};
  item[15] = {22 , "Resolution Divider"            , 1 , READ_WRITE , EEPROM};

  item[16] = {24 , "Torque ON/OFF"                 , 1 , READ_WRITE , RAM};
  item[17] = {25 , "LED"                           , 1 , READ_WRITE , RAM};
  item[18] = {26 , "D gain"                        , 1 , READ_WRITE , RAM};
  item[19] = {27 , "I gain"                        , 1 , READ_WRITE , RAM};
  item[20] = {28 , "P gain"                        , 1 , READ_WRITE , RAM};
  item[21] = {30 , "Goal Position"                 , 2 , READ_WRITE , RAM};
  item[22] = {32 , "Moving Speed"                  , 2 , READ_WRITE , RAM};
  item[23] = {35 , "Torque Limit"                  , 2 , READ_WRITE , RAM};
  item[24] = {36 , "Present Position"              , 2 , READ       , RAM};
  item[25] = {38 , "Present Speed"                 , 2 , READ       , RAM};
  item[26] = {40 , "Present Load"                  , 2 , READ       , RAM};
  item[27] = {42 , "Present Voltage"               , 1 , READ       , RAM};
  item[28] = {43 , "Present Temperature"           , 1 , READ       , RAM};
  item[29] = {44 , "Registered"                    , 1 , READ       , RAM};
  item[30] = {46 , "Moving"                        , 1 , READ       , RAM};
  item[31] = {47 , "Lock"                          , 1 , READ_WRITE , RAM};
  item[32] = {48 , "Punch"                         , 2 , READ_WRITE , RAM};
  item[33] = {68 , "Current"                       , 2 , READ       , RAM};
  item[34] = {70 , "Torque Control Mode Enable"    , 1 , READ_WRITE , RAM};
  item[35] = {71 , "Goal Torque"                   , 2 , READ_WRITE , RAM};
  item[36] = {73 , "Goal Acceleration"             , 1 , READ_WRITE , RAM};

  control_table_size = 37;
#endif  
}

void setExtMXInfo()
{
  model_info.velocity_to_value_ratio         = 83.77;
  
  model_info.value_of_0_radian_position      = 2048;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 4095;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

void setXL320Item()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1 , READ_WRITE , EEPROM};
  item[1]  = {4  , "Baud Rate"                     , 1 , READ_WRITE , EEPROM};
  item[2]  = {6  , "CW Angle Limit"                , 2 , READ_WRITE , EEPROM};
  item[3]  = {8  , "CCW Angle Limit"               , 2 , READ_WRITE , EEPROM};
  item[4]  = {11 , "Control Mode"                  , 1 , READ_WRITE , EEPROM};

  item[5]  = {24 , "Torque ON/OFF"                 , 1 , READ_WRITE , RAM};
  item[6]  = {25 , "LED"                           , 1 , READ_WRITE , RAM};
  item[7]  = {27 , "D gain"                        , 1 , READ_WRITE , RAM};
  item[8]  = {28 , "I gain"                        , 1 , READ_WRITE , RAM};
  item[9]  = {29 , "P gain"                        , 1 , READ_WRITE , RAM};
  item[10] = {30 , "Goal Position"                 , 2 , READ_WRITE , RAM};
  item[11] = {32 , "Moving Speed"                  , 2 , READ_WRITE , RAM};
  item[12] = {35 , "Torque Limit"                  , 2 , READ_WRITE , RAM};
  item[13] = {37 , "Present Position"              , 2 , READ       , RAM};
  item[14] = {39 , "Present Speed"                 , 2 , READ       , RAM};
  item[15] = {41 , "Present Load"                  , 2 , READ       , RAM};
  item[16] = {45 , "Present Voltage"               , 1 , READ       , RAM};
  item[17] = {46 , "Present Temperature"           , 1 , READ       , RAM};
  item[18] = {47 , "Registered Instruction"        , 1 , READ       , RAM};
  item[19] = {49 , "Moving"                        , 1 , READ       , RAM};
  item[20] = {50 , "Hardware Error Status"         , 1 , READ       , RAM};
  item[21] = {51 , "Punch"                         , 2 , READ_WRITE , RAM};

  control_table_size = 22;
#else
  item[0]  = {0  , "Model Number"                  , 2 , READ       , EEPROM};
  item[1]  = {2  , "Version of Firmware"           , 1 , READ       , EEPROM};
  item[2]  = {3  , "ID"                            , 1 , READ_WRITE , EEPROM};
  item[3]  = {4  , "Baud Rate"                     , 1 , READ_WRITE , EEPROM};
  item[4]  = {5  , "Return Delay Time"             , 1 , READ_WRITE , EEPROM};
  item[5]  = {6  , "CW Angle Limit"                , 2 , READ_WRITE , EEPROM};
  item[6]  = {8  , "CCW Angle Limit"               , 2 , READ_WRITE , EEPROM};
  item[7]  = {11 , "Control Mode"                  , 1 , READ_WRITE , EEPROM};
  item[8]  = {12 , "Limit Temperature"             , 1 , READ_WRITE , EEPROM};
  item[9]  = {13 , "Down Limit Voltage"            , 4 , READ_WRITE , EEPROM};
  item[10] = {14 , "Up Limit Voltage"              , 4 , READ_WRITE , EEPROM};
  item[11] = {15 , "Max Torque"                    , 2 , READ_WRITE , EEPROM};
  item[12] = {17 , "Return Level"                  , 2 , READ_WRITE , EEPROM};
  item[13] = {18 , "Alarm Shutdown"                , 2 , READ_WRITE , EEPROM};

  item[14] = {24 , "Torque ON/OFF"                 , 1 , READ_WRITE , RAM};
  item[15] = {25 , "LED"                           , 1 , READ_WRITE , RAM};
  item[16] = {27 , "D gain"                        , 1 , READ_WRITE , RAM};
  item[17] = {28 , "I gain"                        , 1 , READ_WRITE , RAM};
  item[18] = {29 , "P gain"                        , 1 , READ_WRITE , RAM};
  item[19] = {30 , "Goal Position"                 , 2 , READ_WRITE , RAM};
  item[20] = {32 , "Moving Speed"                  , 2 , READ_WRITE , RAM};
  item[21] = {35 , "Torque Limit"                  , 2 , READ_WRITE , RAM};
  item[22] = {37 , "Present Position"              , 2 , READ       , RAM};
  item[23] = {39 , "Present Speed"                 , 2 , READ       , RAM};
  item[24] = {41 , "Present Load"                  , 2 , READ       , RAM};
  item[25] = {45 , "Present Voltage"               , 1 , READ       , RAM};
  item[26] = {46 , "Present Temperature"           , 1 , READ       , RAM};
  item[27] = {47 , "Registered Instruction"        , 1 , READ       , RAM};
  item[28] = {49 , "Moving"                        , 1 , READ       , RAM};
  item[29] = {50 , "Hardware Error Status"         , 1 , READ       , RAM};
  item[30] = {51 , "Punch"                         , 2 , READ_WRITE , RAM};

  control_table_size = 31;
#endif  
}

void setXL320Info()
{
  model_info.velocity_to_value_ratio         = 86.03;
  
  model_info.value_of_0_radian_position      = 512;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 1024;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

void setXLItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1 , READ_WRITE , EEPROM};
  item[1]  = {8  , "Baud Rate"             , 1 , READ_WRITE , EEPROM};
  item[2]  = {11 , "Operating Mode"        , 1 , READ_WRITE , EEPROM};

  item[3]  = {64 , "Torque Enable"         , 1 , READ_WRITE , RAM};
  item[4]  = {65 , "LED"                   , 1 , READ_WRITE , RAM};
  item[5]  = {100, "Goal PWM"              , 2 , READ_WRITE , RAM};
  item[6]  = {102, "Goal Current"          , 2 , READ_WRITE , RAM};
  item[7]  = {104, "Goal Velocity"         , 4 , READ_WRITE , RAM};
  item[8]  = {108, "Profile Acceleration"  , 4 , READ_WRITE , RAM};
  item[9]  = {112, "Profile Velocity"      , 4 , READ_WRITE , RAM};
  item[10] = {116, "Goal Position"         , 4 , READ_WRITE , RAM};
  item[11] = {120, "Realtime Tick"         , 2 , READ       , RAM};
  item[12] = {122, "Moving"                , 1 , READ       , RAM};
  item[13] = {123, "Moving Status"         , 1 , READ       , RAM};
  item[14] = {124, "Present PWM"           , 2 , READ       , RAM};
  item[15] = {126, "Present Current"       , 2 , READ       , RAM};
  item[16] = {128, "Present Velocity"      , 4 , READ       , RAM};
  item[17] = {132, "Present Position"      , 4 , READ       , RAM};
  item[18] = {136, "Velocity Trajectory"   , 4 , READ       , RAM};
  item[19] = {140, "Position Trajectory"   , 4 , READ       , RAM};
  item[20] = {144, "Present Input Voltage" , 2 , READ       , RAM};
  item[21] = {146, "Present Temperature"   , 1 , READ       , RAM};

  control_table_size = 22;
#else
  item[0]  = {0  , "Model Number"          , 2 , READ       , EEPROM};
  item[1]  = {6  , "Version of Firmware"   , 1 , READ       , EEPROM};
  item[2]  = {7  , "ID"                    , 1 , READ_WRITE , EEPROM};
  item[3]  = {8  , "Baud Rate"             , 1 , READ_WRITE , EEPROM};
  item[4]  = {9  , "Return Delay Time"     , 1 , READ_WRITE , EEPROM};
  item[5]  = {10 , "Drive Mode"            , 1 , READ_WRITE , EEPROM};
  item[6]  = {11 , "Operating Mode"        , 1 , READ_WRITE , EEPROM};
  item[7]  = {12 , "Secondary ID"          , 1 , READ_WRITE , EEPROM};
  item[8]  = {13 , "Protocol Version"      , 1 , READ_WRITE , EEPROM};
  item[9]  = {20 , "Homing Offset"         , 4 , READ_WRITE , EEPROM};
  item[10] = {24 , "Moving Threshold"      , 4 , READ_WRITE , EEPROM};
  item[11] = {31 , "Temperature Limit"     , 1 , READ_WRITE , EEPROM};
  item[12] = {32 , "Max Voltage Limit"     , 2 , READ_WRITE , EEPROM};
  item[13] = {34 , "Min Voltage Limit"     , 2 , READ_WRITE , EEPROM};
  item[14] = {36 , "PWM Limit"             , 2 , READ_WRITE , EEPROM};
  item[15] = {38 , "Current Limit"         , 2 , READ_WRITE , EEPROM};
  item[16] = {40 , "Acceleration Limit"    , 4 , READ_WRITE , EEPROM};
  item[17] = {44 , "Velocity Limit"        , 4 , READ_WRITE , EEPROM};
  item[18] = {48 , "Max Position Limit"    , 4 , READ_WRITE , EEPROM};
  item[19] = {52 , "Min Position Limit"    , 4 , READ_WRITE , EEPROM};
  item[20] = {63 , "Shutdown"              , 1 , READ_WRITE , EEPROM};

  item[21] = {64 , "Torque Enable"         , 1 , READ_WRITE , RAM};
  item[22] = {65 , "LED"                   , 1 , READ_WRITE , RAM};
  item[23] = {68 , "Status Return Level"   , 1 , READ_WRITE , RAM};
  item[24] = {69 , "Registered Instruction", 1 , READ       , RAM};
  item[25] = {70 , "Hardware Error Status" , 1 , READ       , RAM};
  item[26] = {76 , "Velocity I Gain"       , 2 , READ_WRITE , RAM};
  item[27] = {78 , "Velocity P Gain"       , 2 , READ_WRITE , RAM};
  item[28] = {80 , "Position D Gain"       , 2 , READ_WRITE , RAM};
  item[29] = {82 , "Position I Gain"       , 2 , READ_WRITE , RAM};
  item[30] = {84 , "Position P Gain"       , 2 , READ_WRITE , RAM};
  item[31] = {88 , "Feedforward 2nd Gain"  , 2 , READ_WRITE , RAM};
  item[32] = {90 , "Feedforward 1st Gain"  , 2 , READ_WRITE , RAM};
  item[33] = {98 , "Bus Watchdog"          , 1 , READ_WRITE , RAM};
  item[34] = {100, "Goal PWM"              , 2 , READ_WRITE , RAM};
  item[35] = {102, "Goal Current"          , 2 , READ_WRITE , RAM};
  item[36] = {104, "Goal Velocity"         , 4 , READ_WRITE , RAM};
  item[37] = {108, "Profile Acceleration"  , 4 , READ_WRITE , RAM};
  item[38] = {112, "Profile Velocity"      , 4 , READ_WRITE , RAM};
  item[39] = {116, "Goal Position"         , 4 , READ_WRITE , RAM};
  item[40] = {120, "Realtime Tick"         , 2 , READ       , RAM};
  item[41] = {122, "Moving"                , 1 , READ       , RAM};
  item[42] = {123, "Moving Status"         , 1 , READ       , RAM};
  item[43] = {124, "Present PWM"           , 2 , READ       , RAM};
  item[44] = {126, "Present Current"       , 2 , READ       , RAM};
  item[45] = {128, "Present Velocity"      , 4 , READ       , RAM};
  item[46] = {132, "Present Position"      , 4 , READ       , RAM};
  item[47] = {136, "Velocity Trajectory"   , 4 , READ       , RAM};
  item[48] = {140, "Position Trajectory"   , 4 , READ       , RAM};
  item[49] = {144, "Present Input Voltage" , 2 , READ       , RAM};
  item[50] = {146, "Present Temperature"   , 1 , READ       , RAM};

  control_table_size = 51;
#endif  
}

void setXLInfo()
{
  model_info.velocity_to_value_ratio         = 41.71;

  model_info.value_of_0_radian_position      = 2048;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 4095;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

void setXMItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1 , READ_WRITE , EEPROM};
  item[1]  = {8  , "Baud Rate"             , 1 , READ_WRITE , EEPROM};
  item[2]  = {11 , "Operating Mode"        , 1 , READ_WRITE , EEPROM};

  item[3]  = {64 , "Torque Enable"         , 1 , READ_WRITE , RAM};
  item[4]  = {65 , "LED"                   , 1 , READ_WRITE , RAM};
  item[5]  = {100, "Goal PWM"              , 2 , READ_WRITE , RAM};
  item[6]  = {102, "Goal Current"          , 2 , READ_WRITE , RAM};
  item[7]  = {104, "Goal Velocity"         , 4 , READ_WRITE , RAM};
  item[8]  = {108, "Profile Acceleration"  , 4 , READ_WRITE , RAM};
  item[9]  = {112, "Profile Velocity"      , 4 , READ_WRITE , RAM};
  item[10] = {116, "Goal Position"         , 4 , READ_WRITE , RAM};
  item[11] = {120, "Realtime Tick"         , 2 , READ       , RAM};
  item[12] = {122, "Moving"                , 1 , READ       , RAM};
  item[13] = {123, "Moving Status"         , 1 , READ       , RAM};
  item[14] = {124, "Present PWM"           , 2 , READ       , RAM};
  item[15] = {126, "Present Current"       , 2 , READ       , RAM};
  item[16] = {128, "Present Velocity"      , 4 , READ       , RAM};
  item[17] = {132, "Present Position"      , 4 , READ       , RAM};
  item[18] = {136, "Velocity Trajectory"   , 4 , READ       , RAM};
  item[19] = {140, "Position Trajectory"   , 4 , READ       , RAM};
  item[20] = {144, "Present Input Voltage" , 2 , READ       , RAM};
  item[21] = {146, "Present Temperature"   , 1 , READ       , RAM};

  control_table_size = 22;
#else
  item[0]  = {0  , "Model Number"          , 2 , READ       , EEPROM};
  item[1]  = {6  , "Version of Firmware"   , 1 , READ       , EEPROM};
  item[2]  = {7  , "ID"                    , 1 , READ_WRITE , EEPROM};
  item[3]  = {8  , "Baud Rate"             , 1 , READ_WRITE , EEPROM};
  item[4]  = {9  , "Return Delay Time"     , 1 , READ_WRITE , EEPROM};
  item[5]  = {10 , "Drive Mode"            , 1 , READ_WRITE , EEPROM};
  item[6]  = {11 , "Operating Mode"        , 1 , READ_WRITE , EEPROM};
  item[7]  = {12 , "Secondary ID"          , 1 , READ_WRITE , EEPROM};
  item[8]  = {13 , "Protocol Version"      , 1 , READ_WRITE , EEPROM};
  item[9]  = {20 , "Homing Offset"         , 4 , READ_WRITE , EEPROM};
  item[10] = {24 , "Moving Threshold"      , 4 , READ_WRITE , EEPROM};
  item[11] = {31 , "Temperature Limit"     , 1 , READ_WRITE , EEPROM};
  item[12] = {32 , "Max Voltage Limit"     , 2 , READ_WRITE , EEPROM};
  item[13] = {34 , "Min Voltage Limit"     , 2 , READ_WRITE , EEPROM};
  item[14] = {36 , "PWM Limit"             , 2 , READ_WRITE , EEPROM};
  item[15] = {38 , "Current Limit"         , 2 , READ_WRITE , EEPROM};
  item[16] = {40 , "Acceleration Limit"    , 4 , READ_WRITE , EEPROM};
  item[17] = {44 , "Velocity Limit"        , 4 , READ_WRITE , EEPROM};
  item[18] = {48 , "Max Position Limit"    , 4 , READ_WRITE , EEPROM};
  item[19] = {52 , "Min Position Limit"    , 4 , READ_WRITE , EEPROM};
  item[20] = {63 , "Shutdown"              , 1 , READ_WRITE , EEPROM};

  item[21] = {64 , "Torque Enable"         , 1 , READ_WRITE , RAM};
  item[22] = {65 , "LED"                   , 1 , READ_WRITE , RAM};
  item[23] = {68 , "Status Return Level"   , 1 , READ_WRITE , RAM};
  item[24] = {69 , "Registered Instruction", 1 , READ       , RAM};
  item[25] = {70 , "Hardware Error Status" , 1 , READ       , RAM};
  item[26] = {76 , "Velocity I Gain"       , 2 , READ_WRITE , RAM};
  item[27] = {78 , "Velocity P Gain"       , 2 , READ_WRITE , RAM};
  item[28] = {80 , "Position D Gain"       , 2 , READ_WRITE , RAM};
  item[29] = {82 , "Position I Gain"       , 2 , READ_WRITE , RAM};
  item[30] = {84 , "Position P Gain"       , 2 , READ_WRITE , RAM};
  item[31] = {88 , "Feedforward 2nd Gain"  , 2 , READ_WRITE , RAM};
  item[32] = {90 , "Feedforward 1st Gain"  , 2 , READ_WRITE , RAM};
  item[33] = {98 , "Bus Watchdog"          , 1 , READ_WRITE , RAM};
  item[34] = {100, "Goal PWM"              , 2 , READ_WRITE , RAM};
  item[35] = {102, "Goal Current"          , 2 , READ_WRITE , RAM};
  item[36] = {104, "Goal Velocity"         , 4 , READ_WRITE , RAM};
  item[37] = {108, "Profile Acceleration"  , 4 , READ_WRITE , RAM};
  item[38] = {112, "Profile Velocity"      , 4 , READ_WRITE , RAM};
  item[39] = {116, "Goal Position"         , 4 , READ_WRITE , RAM};
  item[40] = {120, "Realtime Tick"         , 2 , READ       , RAM};
  item[41] = {122, "Moving"                , 1 , READ       , RAM};
  item[42] = {123, "Moving Status"         , 1 , READ       , RAM};
  item[43] = {124, "Present PWM"           , 2 , READ       , RAM};
  item[44] = {126, "Present Current"       , 2 , READ       , RAM};
  item[45] = {128, "Present Velocity"      , 4 , READ       , RAM};
  item[46] = {132, "Present Position"      , 4 , READ       , RAM};
  item[47] = {136, "Velocity Trajectory"   , 4 , READ       , RAM};
  item[48] = {140, "Position Trajectory"   , 4 , READ       , RAM};
  item[49] = {144, "Present Input Voltage" , 2 , READ       , RAM};
  item[50] = {146, "Present Temperature"   , 1 , READ       , RAM};

  control_table_size = 51;
#endif  
}

void setXMInfo()
{
  model_info.velocity_to_value_ratio       = 41.71;
  model_info.torque_to_current_value_ratio = 149.795386991;

  model_info.value_of_min_radian_position  = 0;
  model_info.value_of_0_radian_position    = 2048;
  model_info.value_of_max_radian_position  = 4095;

  model_info.min_radian = -3.14159265;
  model_info.max_radian = 3.14159265;
}

void setXHItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1 , READ_WRITE , EEPROM};
  item[1]  = {8  , "Baud Rate"             , 1 , READ_WRITE , EEPROM};
  item[2]  = {11 , "Operating Mode"        , 1 , READ_WRITE , EEPROM};

  item[3]  = {64 , "Torque Enable"         , 1 , READ_WRITE , RAM};
  item[4]  = {65 , "LED"                   , 1 , READ_WRITE , RAM};
  item[5]  = {100, "Goal PWM"              , 2 , READ_WRITE , RAM};
  item[6]  = {102, "Goal Current"          , 2 , READ_WRITE , RAM};
  item[7]  = {104, "Goal Velocity"         , 4 , READ_WRITE , RAM};
  item[8]  = {108, "Profile Acceleration"  , 4 , READ_WRITE , RAM};
  item[9]  = {112, "Profile Velocity"      , 4 , READ_WRITE , RAM};
  item[10] = {116, "Goal Position"         , 4 , READ_WRITE , RAM};
  item[11] = {120, "Realtime Tick"         , 2 , READ       , RAM};
  item[12] = {122, "Moving"                , 1 , READ       , RAM};
  item[13] = {123, "Moving Status"         , 1 , READ       , RAM};
  item[14] = {124, "Present PWM"           , 2 , READ       , RAM};
  item[15] = {126, "Present Current"       , 2 , READ       , RAM};
  item[16] = {128, "Present Velocity"      , 4 , READ       , RAM};
  item[17] = {132, "Present Position"      , 4 , READ       , RAM};
  item[18] = {136, "Velocity Trajectory"   , 4 , READ       , RAM};
  item[19] = {140, "Position Trajectory"   , 4 , READ       , RAM};
  item[20] = {144, "Present Input Voltage" , 2 , READ       , RAM};
  item[21] = {146, "Present Temperature"   , 1 , READ       , RAM};

  control_table_size = 22;
#else
  item[0]  = {0  , "Model Number"          , 2 , READ       , EEPROM};
  item[1]  = {6  , "Version of Firmware"   , 1 , READ       , EEPROM};
  item[2]  = {7  , "ID"                    , 1 , READ_WRITE , EEPROM};
  item[3]  = {8  , "Baud Rate"             , 1 , READ_WRITE , EEPROM};
  item[4]  = {9  , "Return Delay Time"     , 1 , READ_WRITE , EEPROM};
  item[5]  = {10 , "Drive Mode"            , 1 , READ_WRITE , EEPROM};
  item[6]  = {11 , "Operating Mode"        , 1 , READ_WRITE , EEPROM};
  item[7]  = {12 , "Secondary ID"          , 1 , READ_WRITE , EEPROM};
  item[8]  = {13 , "Protocol Version"      , 1 , READ_WRITE , EEPROM};
  item[9]  = {20 , "Homing Offset"         , 4 , READ_WRITE , EEPROM};
  item[10] = {24 , "Moving Threshold"      , 4 , READ_WRITE , EEPROM};
  item[11] = {31 , "Temperature Limit"     , 1 , READ_WRITE , EEPROM};
  item[12] = {32 , "Max Voltage Limit"     , 2 , READ_WRITE , EEPROM};
  item[13] = {34 , "Min Voltage Limit"     , 2 , READ_WRITE , EEPROM};
  item[14] = {36 , "PWM Limit"             , 2 , READ_WRITE , EEPROM};
  item[15] = {38 , "Current Limit"         , 2 , READ_WRITE , EEPROM};
  item[16] = {40 , "Acceleration Limit"    , 4 , READ_WRITE , EEPROM};
  item[17] = {44 , "Velocity Limit"        , 4 , READ_WRITE , EEPROM};
  item[18] = {48 , "Max Position Limit"    , 4 , READ_WRITE , EEPROM};
  item[19] = {52 , "Min Position Limit"    , 4 , READ_WRITE , EEPROM};
  item[20] = {63 , "Shutdown"              , 1 , READ_WRITE , EEPROM};

  item[21] = {64 , "Torque Enable"         , 1 , READ_WRITE , RAM};
  item[22] = {65 , "LED"                   , 1 , READ_WRITE , RAM};
  item[23] = {68 , "Status Return Level"   , 1 , READ_WRITE , RAM};
  item[24] = {69 , "Registered Instruction", 1 , READ       , RAM};
  item[25] = {70 , "Hardware Error Status" , 1 , READ       , RAM};
  item[26] = {76 , "Velocity I Gain"       , 2 , READ_WRITE , RAM};
  item[27] = {78 , "Velocity P Gain"       , 2 , READ_WRITE , RAM};
  item[28] = {80 , "Position D Gain"       , 2 , READ_WRITE , RAM};
  item[29] = {82 , "Position I Gain"       , 2 , READ_WRITE , RAM};
  item[30] = {84 , "Position P Gain"       , 2 , READ_WRITE , RAM};
  item[31] = {88 , "Feedforward 2nd Gain"  , 2 , READ_WRITE , RAM};
  item[32] = {90 , "Feedforward 1st Gain"  , 2 , READ_WRITE , RAM};
  item[33] = {98 , "Bus Watchdog"          , 1 , READ_WRITE , RAM};
  item[34] = {100, "Goal PWM"              , 2 , READ_WRITE , RAM};
  item[35] = {102, "Goal Current"          , 2 , READ_WRITE , RAM};
  item[36] = {104, "Goal Velocity"         , 4 , READ_WRITE , RAM};
  item[37] = {108, "Profile Acceleration"  , 4 , READ_WRITE , RAM};
  item[38] = {112, "Profile Velocity"      , 4 , READ_WRITE , RAM};
  item[39] = {116, "Goal Position"         , 4 , READ_WRITE , RAM};
  item[40] = {120, "Realtime Tick"         , 2 , READ       , RAM};
  item[41] = {122, "Moving"                , 1 , READ       , RAM};
  item[42] = {123, "Moving Status"         , 1 , READ       , RAM};
  item[43] = {124, "Present PWM"           , 2 , READ       , RAM};
  item[44] = {126, "Present Current"       , 2 , READ       , RAM};
  item[45] = {128, "Present Velocity"      , 4 , READ       , RAM};
  item[46] = {132, "Present Position"      , 4 , READ       , RAM};
  item[47] = {136, "Velocity Trajectory"   , 4 , READ       , RAM};
  item[48] = {140, "Position Trajectory"   , 4 , READ       , RAM};
  item[49] = {144, "Present Input Voltage" , 2 , READ       , RAM};
  item[50] = {146, "Present Temperature"   , 1 , READ       , RAM};

  control_table_size = 51;
#endif  
}

void setXHInfo()
{
  model_info.velocity_to_value_ratio       = 41.71;

  model_info.value_of_min_radian_position  = 0;
  model_info.value_of_0_radian_position    = 2048;
  model_info.value_of_max_radian_position  = 4095;

  model_info.min_radian = -3.14159265;
  model_info.max_radian = 3.14159265;
}

static void setPROItem()
{
  #if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1 , READ_WRITE , EEPROM};
  item[1]  = {8  , "Baud Rate"             , 1 , READ_WRITE , EEPROM};
  item[2]  = {11 , "Operating Mode"        , 1 , READ_WRITE , EEPROM};

  item[3]  = {562, "Torque Enable"         , 1 , READ_WRITE , RAM};
  item[4]  = {563, "LED RED"               , 1 , READ_WRITE , RAM};
  item[5]  = {564, "LED GREEN"             , 1 , READ_WRITE , RAM};
  item[6]  = {565, "LED BLUE"              , 1 , READ_WRITE , RAM};
  item[7]  = {586, "Velocity I Gain"       , 2 , READ_WRITE , RAM};
  item[8]  = {588, "Velocity P Gain"       , 2 , READ_WRITE , RAM};
  item[9]  = {594, "Position P Gain"       , 2 , READ_WRITE , RAM};
  item[10] = {596, "Goal Position"         , 4 , READ_WRITE , RAM};
  item[11] = {600, "Goal Velocity"         , 4 , READ_WRITE , RAM};
  item[12] = {604, "Goal Torque"           , 2 , READ_WRITE , RAM};
  item[13] = {606, "Goal Acceleration"     , 4 , READ_WRITE , RAM};
  item[14] = {610, "Moving"                , 1 , READ       , RAM};
  item[15] = {611, "Present Position"      , 4 , READ       , RAM};
  item[16] = {615, "Present Velocity"      , 4 , READ       , RAM};
  item[17] = {621, "Present Current"       , 2 , READ       , RAM};
  item[18] = {623, "Present Input Voltage" , 2 , READ       , RAM};
  item[19] = {625, "Present Temperature"   , 1 , READ       , RAM};
  item[20] = {890, "Registered Instruction", 1 , READ       , RAM};
  item[21] = {891, "Status Return Level"   , 1 , READ_WRITE , RAM};
  item[22] = {892, "Hardware Error Status" , 1 , READ       , RAM};

  control_table_size = 23;
#else
  item[0]  = {0  , "Model Number"          , 2 , READ       , EEPROM};
  item[1]  = {6  , "Version of Firmware"   , 1 , READ       , EEPROM};
  item[2]  = {7  , "ID"                    , 1 , READ_WRITE , EEPROM};
  item[3]  = {8  , "Baud Rate"             , 1 , READ_WRITE , EEPROM};
  item[4]  = {9  , "Return Delay Time"     , 1 , READ_WRITE , EEPROM};
  item[5]  = {11 , "Operating Mode"        , 1 , READ_WRITE , EEPROM};
  item[6]  = {13 , "Homing Offset"         , 4 , READ_WRITE , EEPROM};
  item[7]  = {17 , "Moving Threshold"      , 4 , READ_WRITE , EEPROM};
  item[8]  = {21 , "Temperature Limit"     , 1 , READ_WRITE , EEPROM};
  item[9]  = {22 , "Max Voltage Limit"     , 2 , READ_WRITE , EEPROM};
  item[10] = {24 , "Min Voltage Limit"     , 2 , READ_WRITE , EEPROM};
  item[11] = {26 , "Acceleration Limit"    , 4 , READ_WRITE , EEPROM};
  item[12] = {30 , "Torque Limit"          , 2 , READ_WRITE , EEPROM};
  item[13] = {32 , "Velocity Limit"        , 4 , READ_WRITE , EEPROM};
  item[14] = {36 , "Max Position Limit"    , 4 , READ_WRITE , EEPROM};
  item[15] = {40 , "Min Position Limit"    , 4 , READ_WRITE , EEPROM};
  item[16] = {48 , "Shutdown"              , 1 , READ_WRITE , EEPROM};

  item[17] = {562, "Torque Enable"         , 1 , READ_WRITE , RAM};
  item[18] = {563, "LED RED"               , 1 , READ_WRITE , RAM};
  item[19] = {564, "LED GREEN"             , 1 , READ_WRITE , RAM};
  item[20] = {565, "LED BLUE"              , 1 , READ_WRITE , RAM};
  item[21] = {586, "Velocity I Gain"       , 2 , READ_WRITE , RAM};
  item[22] = {588, "Velocity P Gain"       , 2 , READ_WRITE , RAM};
  item[23] = {594, "Position P Gain"       , 2 , READ_WRITE , RAM};
  item[24] = {596, "Goal Position"         , 4 , READ_WRITE , RAM};
  item[25] = {600, "Goal Velocity"         , 4 , READ_WRITE , RAM};
  item[26] = {604, "Goal Torque"           , 2 , READ_WRITE , RAM};
  item[27] = {606, "Goal Acceleration"     , 4 , READ_WRITE , RAM};
  item[28] = {610, "Moving"                , 1 , READ       , RAM};
  item[29] = {611, "Present Position"      , 4 , READ       , RAM};
  item[30] = {615, "Present Velocity"      , 4 , READ       , RAM};
  item[31] = {621, "Present Current"       , 2 , READ       , RAM};
  item[32] = {623, "Present Input Voltage" , 2 , READ       , RAM};
  item[33] = {625, "Present Temperature"   , 1 , READ       , RAM};
  item[34] = {890, "Registered Instruction", 1 , READ       , RAM};
  item[35] = {891, "Status Return Level"   , 1 , READ_WRITE , RAM};
  item[36] = {892, "Hardware Error Status" , 1 , READ       , RAM};

  control_table_size = 37;
#endif  
}

static void setPROInfo()
{
  model_info.velocity_to_value_ratio         = 4792.8;
  
  model_info.value_of_0_radian_position      = 0;
  model_info.value_of_min_radian_position    = -250950;
  model_info.value_of_max_radian_position    =  250950;
  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

ControlTableItem* getItem(uint16_t num)
{
  if (num == AX_12A || num == AX_12W || num == AX_18A)
  {
    setAXItem();
  }
  else if (num == RX_24F || num == RX_28 || num == RX_64)
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
  else if (num == XH430_V210 || num == XH430_V350 || num == XH430_W210 || num == XH430_W350)
  {
    setXHItem();
  }
  else if (num == PRO_L42_10_S300_R  || num == PRO_L54_30_S400_R || num == PRO_L54_30_S500_R || num == PRO_L54_50_S290_R || num == PRO_L54_50_S500_R  ||
           num == PRO_M42_10_S260_R  || num == PRO_M54_40_S250_R || num == PRO_M54_60_S250_R || num == PRO_H42_20_S300_R || num == PRO_H54_100_S500_R ||
           num == PRO_H54_200_S500_R)
  {
    setPROItem();
  }
  else
  {
    setXMItem();
  }

  return item;
}

ModelInfo* getInfo(uint16_t num)
{  
  if (num == AX_12A || num == AX_12W || num == AX_18A)
  {
    setAXInfo();
  }
  else if (num == RX_24F || num == RX_28 || num == RX_64)
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
  else if (num == XH430_V210 || num == XH430_V350 || num == XH430_W210 || num == XH430_W350)
  {
    setXHInfo();
  }
  else if (num == PRO_L42_10_S300_R  || num == PRO_L54_30_S400_R || num == PRO_L54_30_S500_R || num == PRO_L54_50_S290_R || num == PRO_L54_50_S500_R  ||
           num == PRO_M42_10_S260_R  || num == PRO_M54_40_S250_R || num == PRO_M54_60_S250_R || num == PRO_H42_20_S300_R || num == PRO_H54_100_S500_R ||
           num == PRO_H54_200_S500_R)
  {
    setPROInfo();
  }
  else
  {
    setXMInfo();
  }

  return &model_info;
}

uint8_t getSize()
{
  return control_table_size;
}