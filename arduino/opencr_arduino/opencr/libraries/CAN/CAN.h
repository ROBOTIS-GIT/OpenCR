#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#include <chip.h>
#include "variant.h"
#include "drv_can.h"
#include "def.h"

#define CAN_STD_FORMAT            _DEF_CAN_STD
#define CAN_EXT_FORMAT            _DEF_CAN_EXT

#define CAN_BAUD_125K       _DEF_CAN_BAUD_125K
#define CAN_BAUD_250K       _DEF_CAN_BAUD_250K
#define CAN_BAUD_500K       _DEF_CAN_BAUD_500K
#define CAN_BAUD_1000K      _DEF_CAN_BAUD_1000K

typedef struct can_message
{
    uint32_t id;
    uint32_t length;
    uint8_t  data[8];
    uint8_t  format;
}can_message_t;

class CANClass
{
public:
    CANClass();
    bool begin();
    bool begin(uint32_t baudrate);
    bool begin(uint32_t baudrate, uint8_t format);
    void end(void);
    bool configFilter(uint32_t id, uint32_t mask);
    bool configFilter(uint32_t id, uint32_t mask, uint8_t format);
    uint32_t write(uint32_t id, uint8_t *p_data, uint32_t length);  //write data
    uint32_t write(uint32_t id, uint8_t *p_data, uint32_t length, uint8_t format);
    uint8_t read(void); //read one byte
    uint32_t  avaliable(void);
    uint32_t writeMessage(can_message_t *p_msg);
    bool readMessage(can_message_t *p_msg);
    uint32_t avaliableMessage(void);

    uint8_t getErrCount(void);
    uint32_t getError(void);
    uint32_t getState(void);

    void attachRxInterrupt(void (*handler)(can_message_t *arg));
    void detachRxInterrupt(void);

private:
    uint8_t format_;    
};

extern CANClass CanBus;
#endif
