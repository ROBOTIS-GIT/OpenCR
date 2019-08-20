#ifndef DYNAMIXEL_CONFIG_H_
#define DYNAMIXEL_CONFIG_H_

#define ENABLE_ACTUATOR_AX              1
#define ENABLE_ACTUATOR_DX              0
#define ENABLE_ACTUATOR_RX              0
#define ENABLE_ACTUATOR_EX              0
#define ENABLE_ACTUATOR_MX12W           1
#define ENABLE_ACTUATOR_MX28            1
#define ENABLE_ACTUATOR_MX64            1
#define ENABLE_ACTUATOR_MX106           1

#define ENABLE_ACTUATOR_MX28_PROTOCOL2  1
#define ENABLE_ACTUATOR_MX64_PROTOCOL2  1
#define ENABLE_ACTUATOR_MX106_PROTOCOL2 1

#define ENABLE_ACTUATOR_XL320           1
#define ENABLE_ACTUATOR_XL430           1
#define ENABLE_ACTUATOR_XM430           1
#define ENABLE_ACTUATOR_XH430           1
#define ENABLE_ACTUATOR_XM540           1
#define ENABLE_ACTUATOR_XH540           1

#define ENABLE_ACTUATOR_PRO_R           1
#define ENABLE_ACTUATOR_PRO_RA          1
#define ENABLE_ACTUATOR_PRO_PLUS        1


#if defined (ARDUINO_AVR_UNO) || defined (ARDUINO_AVR_YUN) \
  || defined (ARDUINO_AVR_LEONARDO) || defined (ARDUINO_AVR_INDUSTRIAL101)
#define DXL_MAX_NODE                   16
#define DXL_MAX_NODE_BUFFER_SIZE        8
#elif defined(OpenCR)
#define DXL_MAX_NODE                  253 // Max number of XEL on DYNAMIXEL protocol
#define DXL_MAX_NODE_BUFFER_SIZE       32
#else
#define DXL_MAX_NODE                   16
#define DXL_MAX_NODE_BUFFER_SIZE       16*2
#endif

#define DXL_BUF_LENGTH (DXL_MAX_NODE*DXL_MAX_NODE_BUFFER_SIZE + 11) // 11 = Header(3)+Reserved(1)+ID(1)+Length(2)+Instruction(1)+Error(1)+crc(2)



#if (DXL_MAX_NODE > 253)
#error "\r\nError : DXL_MAX_NODE is OVERFLOW! This should be less or equal than 253 by the protocol."
#endif
#if (DXL_MAX_NODE_BUFFER_SIZE > 0xFF)
#error "\r\nError : DXL_MAX_NODE_BUFFER_SIZE is OVERFLOW! This must be a 8 bit range."
#endif
#if (DXL_BUF_LENGTH > 0xFFFF)
#error "\r\nError : DXL_BUF_LENGTH is OVERFLOW! This must be a 16 bit range."
#endif




#endif /* DYNAMIXEL_CONFIG_H_ */