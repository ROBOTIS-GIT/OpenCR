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

#ifndef DYNAMIXEL_2_ARDUINO_FOR_TB3_H_
#define DYNAMIXEL_2_ARDUINO_FOR_TB3_H_


#include "utility/master.h"
#include "utility/slave.h"
#include "actuator.h"

enum OperatingMode{
  OP_POSITION = 0,
  OP_EXTENDED_POSITION,
  OP_CURRENT_BASED_POSITION,
  OP_VELOCITY,
  OP_PWM,
  OP_CURRENT,

  UNKNOWN_OP
};

enum ParamUnit{
  UNIT_RAW = 0,
  UNIT_PERCENT,
  UNIT_RPM,
  UNIT_DEGREE,
  UNIT_MILLI_AMPERE
};


class Dynamixel2Arduino : public DYNAMIXEL::Master
{
  public:
    /**
     * @brief The constructor.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * @endcode
     * @param port The HardwareSerial port you want to use on the board to communicate with DYNAMIXELs.
     *          It is automatically initialized baudrate to 57600 by calling the begin () function.
     * @param dir_pin Directional pins for using half-duplex communication. -1 uses full duplex. (default : -1)
     */   
    Dynamixel2Arduino(HardwareSerial& port, int dir_pin = -1);
    
    /**
     * @brief Initialization function to start communication with DYNAMIXEL.
     *      Or change baudrate of baud serial port.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.begin(57600);
     * @endcode
     * @param baud The port speed you want on the board (the speed to communicate with DYNAMIXEL) (default : 57600)
     * @return It returns true(1) on success, false(0) on failure.
     */    
    void begin(unsigned long baud = 57600);

    /**
     * @brief It is API for getting serial baudrate of board port.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * Serial.print(dxl.getPortBaud());
     * @endcode
     * @return It returns serial baudrate of board port.
     */   
    unsigned long getPortBaud() const;
    
    /**
     * @brief It is API for To checking the communication connection status of DYNAMIXEL.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.ping(1);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID or BROADCAST ID (0xFE). (default : 0xFE)
     * @return It returns true(1) on success, false(0) on failure.
     */    
    bool ping(uint8_t id = DXL_BROADCAST_ID);

    /**
     * @brief It is same as using the broadcast parameter(0xFE) of the ping function.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.scan();
     * @endcode
     * @return If a dynamixel succeeds in pinging, it returns 1, and returns 0 if none succeeds.
     */    
    bool scan();

    /**
     * @brief It is API for getting model number of DYNAMIXEL.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * Serial.print(dxl.getModelNumber(1));
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @return It returns the data read from DXL control table item.
     * If the read fails, 0xFFFF is returned.
     */      
    uint16_t getModelNumber(uint8_t id);

    /**
     * @brief It is API for changing ID of DYNAMIXEL.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.setID(1, 2);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param new_id DYNAMIXEL Actuator's ID you want.
     * @return It returns true(1) on success, false(0) on failure.
     */
    bool setID(uint8_t id, uint8_t new_id);

    /**
     * @brief It is API for changing protocol of DYNAMIXEL.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.setProtocol(1, 1.0);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param version DYNAMIXEL Actuator's protocol version you want. (1.0 or 2.0)
     * @return It returns true(1) on success, false(0) on failure.
     */    
    bool setProtocol(uint8_t id, float version);

    /**
     * @brief It is API for changing baudrate of DYNAMIXEL.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.setBaudrate(1, 57600);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param baudrate DYNAMIXEL Actuator's baudrate value. (Refer to the control table of each actuator for supported values (e-manual))
     * @return It returns true(1) on success, false(0) on failure.
     */    
    bool setBaudrate(uint8_t id, uint32_t baudrate);

    /**
     * @brief It is API for controlling torque on/off(turn on) of DYNAMIXEL.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.torqueOn(1);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @return It returns true(1) on success, false(0) on failure.
     */
    bool torqueOn(uint8_t id);

    /**
     * @brief It is API for controlling torque on/off(turn on) of DYNAMIXEL.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.torqueOff(1);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @return It returns true(1) on success, false(0) on failure.
     */    
    bool torqueOff(uint8_t id);

    /**
     * @brief It is API for controlling LED(turn on) of DYNAMIXEL.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.ledOn(1);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @return It returns true(1) on success, false(0) on failure.
     */
    bool ledOn(uint8_t id);

     /**
     * @brief It is API for controlling LED(turn off) of DYNAMIXEL.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.ledOff(1);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @return It returns true(1) on success, false(0) on failure.
     */
    bool ledOff(uint8_t id);

    /**
     * @brief It is API for controlling operating mode of DYNAMIXEL.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.setOperatingMode(1, OP_POSITION);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param mode The operating mode you want.
     * @return It returns true(1) on success, false(0) on failure.
     */
    bool setOperatingMode(uint8_t id, uint8_t mode);

    /**
     * @brief It is API for controlling position(joint) of DYNAMIXEL.
     * You can use the @unit parameter to specify the unit of @value.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.setGoalPosition(1, 512);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param value The value you want to set.
     * @param unit The unit of the value you entered. (default : UNIT_RAW)
     *    Only support UNIT_RAW, UNIT_DEGREE.
     * @return It returns true(1) on success, false(0) on failure.
     */
    bool setGoalPosition(uint8_t id, float value, uint8_t unit = UNIT_RAW);

    /**
     * @brief It is API for getting present position (joint) of DYNAMIXEL.
     * You can use the @unit parameter to specify the unit of @value.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * Serial.print(dxl.getPresentPosition(1));
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param unit The unit you want to return (the function converts the raw value to the unit you specified and returns it) (default : UNIT_RAW)
     *    Only support UNIT_RAW, UNIT_DEGREE.
     * @return It returns the data read from DXL control table item.(Returns the value appropriate for @unit.)
     * If the read fails, 0 is returned. Whether or not this is an actual value can be confirmed with @getLastErrorCode().
     */    
    float getPresentPosition(uint8_t id, uint8_t unit = UNIT_RAW);

    /**
     * @brief It is API for controlling velocity(wheel mode speed) of DYNAMIXEL.
     * You can use the @unit parameter to specify the unit of @value.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.setGoalVelocity(1, 512);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param value The value you want to set.
     * @param unit The unit of the value you entered. (default : UNIT_RAW)
     *    Only support UNIT_RAW, UNIT_PERCENT, UNIT_RPM.
     * @return It returns true(1) on success, false(0) on failure.
     */
    bool setGoalVelocity(uint8_t id, float value, uint8_t unit = UNIT_RAW);

    /**
     * @brief It is API for getting present velocity(wheel mode speed) of DYNAMIXEL.
     * You can use the @unit parameter to specify the unit of @value.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * Serial.print(dxl.getPresentVelocity(1));
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param unit The unit you want to return (the function converts the raw value to the unit you specified and returns it) (default : UNIT_RAW)
     *    Only support UNIT_RAW, UNIT_PERCENT, UNIT_RPM.
     * @return It returns the data read from DXL control table item.(Returns the value appropriate for @unit.)
     * If the read fails, 0 is returned. Whether or not this is an actual value can be confirmed with @getLastErrorCode().
     */    
    float getPresentVelocity(uint8_t id, uint8_t unit = UNIT_RAW);

    /**
     * @brief It is API for controlling PWM of DYNAMIXEL.
     * You can use the @unit parameter to specify the unit of @value.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.setGoalPWM(1, 50.0, UNIT_PERCENT);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param value The value you want to set.
     * @param unit The unit of the value you entered.  (default : UNIT_RAW)
     *    Only support UNIT_RAW, UNIT_PERCENT.
     * @return It returns true(1) on success, false(0) on failure.
     */
    bool setGoalPWM(uint8_t id, float value, uint8_t unit = UNIT_RAW);

    /**
     * @brief It is API for getting present PWM of DYNAMIXEL.
     * You can use the @unit parameter to specify the unit of @value.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * Serial.print(dxl.getPresentPWM(1, UNIT_PERCENT));
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param unit The unit you want to return (the function converts the raw value to the unit you specified and returns it) (default : UNIT_RAW)
     *    Only support UNIT_RAW, UNIT_PERCENT.
     * @return It returns the data read from DXL control table item.(Returns the value appropriate for @unit.)
     * If the read fails, 0 is returned. Whether or not this is an actual value can be confirmed with @getLastErrorCode().
     */    
    float getPresentPWM(uint8_t id, uint8_t unit = UNIT_RAW);

    /**
     * @brief It is API for controlling current of DYNAMIXEL.
     * You can use the @unit parameter to specify the unit of @value.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.setGoalCurrent(1, 512);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param value The value you want to set.
     * @param unit The unit of the value you entered.  (default : UNIT_RAW)
     *    Only support UNIT_RAW, UNIT_PERCENT, UNIT_MILLI_AMPERE.
     * @return It returns true(1) on success, false(0) on failure.
     */
    bool setGoalCurrent(uint8_t id, float value, uint8_t unit = UNIT_RAW);

    /**
     * @brief It is API for getting present current of DYNAMIXEL.
     * You can use the @unit parameter to specify the unit of @value.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * Serial.print(dxl.getPresentCurrent(1));
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param unit The unit you want to return (the function converts the raw value to the unit you specified and returns it) (default : UNIT_RAW)
     *    Only support UNIT_RAW, UNIT_PERCENT, UNIT_MILLI_AMPERE.
     * @return It returns the data read from DXL control table item.(Returns the value appropriate for @unit.)
     * If the read fails, 0 is returned. Whether or not this is an actual value can be confirmed with @getLastErrorCode().
     */  
    float getPresentCurrent(uint8_t id, uint8_t unit = UNIT_RAW);    

    /**
     * @brief It is API for getting data of a DYNAMIXEL control table item.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * Serial.print(dxl.readControlTableItem(PRESENT_VOLTAGE, 1));
     * @endcode
     * @param item_idx DYNAMIXEL Actuator's control table item's index.
     *    For each index, replace the name of the control table item in the e-manual with capital letters and replace the space with an underscore (_).
     *    For the list of indexes, please refer to the link below.
     *    TODO: add link (It is defined as 'enum ControlTableItem' in actuator.h)
     * @param id DYNAMIXEL Actuator's ID.
     * @param timeout A timeout waiting for a response to a data transfer.
     * @return It returns the data read from DXL control table item.(Returns the value appropriate for @unit.)
     * If the read fails, 0 is returned. Whether or not this is an actual value can be confirmed with @getLastErrorCode().
     */  
    int32_t readControlTableItem(uint8_t item_idx, 
      uint8_t id, uint32_t timeout = 3);

    /**
     * @brief It is API for controlling data of a DYNAMIXEL control table item.
     * @code
     * const int DXL_DIR_PIN = 2;
     * Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
     * dxl.writeControlTableItem(TORQUE_ENABLE, 1, 1);
     * @endcode
     * @param item_idx DYNAMIXEL Actuator's control table item's index.
     *    For each index, replace the name of the control table item in the e-manual with capital letters and replace the space with an underscore (_).
     *    For the list of indexes, please refer to the link below.
     *    TODO: add link (It is defined as 'enum ControlTableItem' in actuator.h)
     * @param id DYNAMIXEL Actuator's ID.
     * @param data The data you want to write. Only the data size allowed by the address is applied.
     * @param timeout A timeout waiting for a response to a data transfer.
     * @return It returns true(1) on success, false(0) on failure.
     */
    bool writeControlTableItem(uint8_t item_idx, 
      uint8_t id, int32_t data, uint32_t timeout = 3);


#if 0 //TODO
    bool setDirectionToNormal(uint8_t id);
    bool setDirectionToReverse(uint8_t id);
    bool setDirection(uint8_t id, bool dir);

    bool setProfileBase(uint8_t id, uint8_t base);
    bool setProfileToVelocityBased(uint8_t id);
    bool setProfileToTimeBased(uint8_t id);

    bool setProfileVelocity(uint8_t id, float value, uint8_t unit = UNIT_RAW);
    bool setProfileAcceleration(uint8_t id, float value, uint8_t unit = UNIT_RAW);

    bool setPositionPIDGain(uint8_t id, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain);
    bool setVelocityPIGain(uint8_t id, uint16_t p_gain, uint16_t i_gain);
    bool setFeedForwardGain(uint8_t id, uint16_t fisrt_gain, uint16_t second_gain);

    uint8_t getHardwareError(uint8_t id);
#endif     

  private:
    typedef struct IdAndModelNum{
      uint16_t model_num;
      uint8_t id;
    } IdAndModelNum_t;

    DYNAMIXEL::SerialPortHandler dxl_port_;
    
    IdAndModelNum_t registered_dxl_[DXL_MAX_NODE];
    uint8_t         registered_dxl_cnt_;
    uint32_t        err_code_;

    uint16_t getModelNumberFromTable(uint8_t id);

    bool setTorqueEnable(uint8_t id, bool enable);
    bool setLedState(uint8_t id, bool state);

    float readForRangeDependencyFunc(uint8_t func_idx, uint8_t id, uint8_t unit);
    bool writeForRangeDependencyFunc(uint8_t func_idx, uint8_t id, float value, uint8_t unit);

    int32_t readControlTableItem(uint16_t model_num,
      uint8_t item_idx, uint8_t id, uint32_t timeout = 3);
    
    bool writeControlTableItem(uint16_t model_num, 
      uint8_t item_idx, uint8_t id, int32_t data, uint32_t timeout = 3);  
};




#endif /* DYNAMIXEL_2_ARDUINO_FOR_TB3_H_ */