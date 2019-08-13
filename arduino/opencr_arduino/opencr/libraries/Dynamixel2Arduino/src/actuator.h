#ifndef DYNAMIXEL_ACTUATOR_H_
#define DYNAMIXEL_ACTUATOR_H_

#include "stdint.h"
#include "utility/config.h"


#define AX12A               (uint16_t)12
#define AX12W               (uint16_t)300
#define AX18A               (uint16_t)18

#define RX10                (uint16_t)10
#define RX24F               (uint16_t)24
#define RX28                (uint16_t)28
#define RX64                (uint16_t)64

#define DX113               (uint16_t)113
#define DX116               (uint16_t)116
#define DX117               (uint16_t)117

#define EX106               (uint16_t)107

#define MX12W               (uint16_t)104
#define MX28                (uint16_t)29
#define MX64                (uint16_t)310
#define MX106               (uint16_t)320

#define MX28_2              (uint16_t)30
#define MX64_2              (uint16_t)311
#define MX106_2             (uint16_t)321

#define XL320               (uint16_t)350

#define XC430_W150          (uint16_t)1070
#define XC430_W240          (uint16_t)1080
#define XXC430_W250         (uint16_t)1160

#define XL430_W250          (uint16_t)1060

#define XXL430_W250         (uint16_t)1090

#define XM430_W210          (uint16_t)1030
#define XM430_W350          (uint16_t)1020

#define XM540_W150          (uint16_t)1130
#define XM540_W270          (uint16_t)1120

#define XH430_V210          (uint16_t)1050
#define XH430_V350          (uint16_t)1040
#define XH430_W210          (uint16_t)1010
#define XH430_W350          (uint16_t)1000

#define XH540_W150          (uint16_t)1110
#define XH540_W270          (uint16_t)1100
#define XH540_V150          (uint16_t)1150
#define XH540_V270          (uint16_t)1140

#define PRO_L42_10_S300_R   (uint16_t)35072
#define PRO_L54_30_S400_R   (uint16_t)37928
#define PRO_L54_30_S500_R   (uint16_t)37896
#define PRO_L54_50_S290_R   (uint16_t)38176
#define PRO_L54_50_S500_R   (uint16_t)38152

#define PRO_M42_10_S260_R   (uint16_t)43288
#define PRO_M54_40_S250_R   (uint16_t)46096
#define PRO_M54_60_S250_R   (uint16_t)46352

#define PRO_H42_20_S300_R   (uint16_t)51200
#define PRO_H54_100_S500_R  (uint16_t)53768
#define PRO_H54_200_S500_R  (uint16_t)54024

#define PRO_M42_10_S260_RA  (uint16_t)43289
#define PRO_M54_40_S250_RA  (uint16_t)46097
#define PRO_M54_60_S250_RA  (uint16_t)46353

#define PRO_H42_20_S300_RA  (uint16_t)51201
#define PRO_H54_100_S500_RA (uint16_t)53761
#define PRO_H54_200_S500_RA (uint16_t)54025

#define PRO_H42P_020_S300_R (uint16_t)2000
#define PRO_H54P_100_S500_R (uint16_t)2010
#define PRO_H54P_200_S500_R (uint16_t)2020

#define PRO_M42P_010_S260_R (uint16_t)2100
#define PRO_M54P_040_S250_R (uint16_t)2110
#define PRO_M54P_060_S250_R (uint16_t)2120


enum ControlTableItem{
  MODEL_NUMBER = 0,
  MODEL_INFORMATION,
  FIRMWARE_VERSION,
  PROTOCOL_VERSION,
  ID,
  SECONDARY_ID,
  BAUD_RATE,
  DRIVE_MODE,
  CONTROL_MODE,
  OPERATING_MODE,
  CW_ANGLE_LIMIT,
  CCW_ANGLE_LIMIT,
  TEMPERATURE_LIMIT,
  MIN_VOLTAGE_LIMIT,
  MAX_VOLTAGE_LIMIT,
  PWM_LIMIT,
  CURRENT_LIMIT,
  VELOCITY_LIMIT,
  MAX_POSITION_LIMIT,
  MIN_POSITION_LIMIT,
  ACCELERATION_LIMIT,
  MAX_TORQUE,
  HOMING_OFFSET,
  MOVING_THRESHOLD,
  MULTI_TURN_OFFSET,
  RESOLUTION_DIVIDER,
  EXTERNAL_PORT_MODE_1,
  EXTERNAL_PORT_MODE_2,
  EXTERNAL_PORT_MODE_3,
  EXTERNAL_PORT_MODE_4,
  STATUS_RETURN_LEVEL,
  RETURN_DELAY_TIME,
  ALARM_LED,
  SHUTDOWN,

  TORQUE_ENABLE,
  LED,
  LED_RED,
  LED_GREEN,
  LED_BLUE,
  REGISTERED_INSTRUCTION,
  HARDWARE_ERROR_STATUS,
  VELOCITY_P_GAIN,
  VELOCITY_I_GAIN,
  POSITION_P_GAIN,
  POSITION_I_GAIN,
  POSITION_D_GAIN,
  FEEDFORWARD_1ST_GAIN,
  FEEDFORWARD_2ND_GAIN,
  P_GAIN,
  I_GAIN,
  D_GAIN,
  CW_COMPLIANCE_MARGIN,
  CCW_COMPLIANCE_MARGIN,
  CW_COMPLIANCE_SLOPE,
  CCW_COMPLIANCE_SLOPE,
  GOAL_PWM,
  GOAL_TORQUE,
  GOAL_CURRENT,
  GOAL_POSITION,
  GOAL_VELOCITY,
  GOAL_ACCELERATION,
  MOVING_SPEED,
  PRESENT_PWM,
  PRESENT_LOAD,
  PRESENT_SPEED,
  PRESENT_CURRENT,
  PRESENT_POSITION,
  PRESENT_VELOCITY,
  PRESENT_VOLTAGE,
  PRESENT_TEMPERATURE,
  TORQUE_LIMIT,
  REGISTERED,
  MOVING,
  LOCK,
  PUNCH,
  CURRENT,
  SENSED_CURRENT,
  REALTIME_TICK,
  TORQUE_CTRL_MODE_ENABLE,
  BUS_WATCHDOG,
  PROFILE_ACCELERATION,
  PROFILE_VELOCITY,
  MOVING_STATUS,
  VELOCITY_TRAJECTORY,
  POSITION_TRAJECTORY,
  PRESENT_INPUT_VOLTAGE,
  EXTERNAL_PORT_DATA_1,
  EXTERNAL_PORT_DATA_2,
  EXTERNAL_PORT_DATA_3,
  EXTERNAL_PORT_DATA_4,

  LAST_DUMMY_ITEM = 0xFF
};

namespace DYNAMIXEL{

typedef struct ControlTableItemInfo{
  uint16_t addr;
  uint8_t addr_length;
} ControlTableItemInfo_t;

ControlTableItemInfo_t getControlTableItemInfo(uint16_t model_num, uint8_t control_item);

} // namespace DYNAMIXEL

#endif /* DYNAMIXEL_ACTUATOR_H_ */