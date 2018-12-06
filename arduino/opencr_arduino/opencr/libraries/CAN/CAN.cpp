#include "CAN.h"
#include "drv_can.h"

CANClass CanBus;

#define OPENCR_CAN_CHANNEL 0 //CAN2

CANClass::CANClass()
{
    format_ = CAN_STD_FORMAT;
}

bool CANClass::begin()
{
    return drvCanOpen(OPENCR_CAN_CHANNEL, CAN_BAUD_125K, format_);
}

bool CANClass::begin(uint32_t baudrate)
{
    return drvCanOpen(OPENCR_CAN_CHANNEL, baudrate, format_);
}

bool CANClass::begin(uint32_t baudrate, uint8_t format)
{
    return drvCanOpen(OPENCR_CAN_CHANNEL, baudrate, format);
}

void CANClass::end(void)
{
    drvCanClose(OPENCR_CAN_CHANNEL);
}

 bool CANClass::configFilter(uint32_t id, uint32_t mask)
 {
    return drvCanConfigFilter(0, id, mask, format_);
 }

 bool CANClass::configFilter(uint32_t id, uint32_t mask, uint8_t format)
 {
    return drvCanConfigFilter(0, id, mask, format);
 }

uint32_t CANClass::write(uint32_t id, uint8_t *p_data, uint32_t length)
{
    return drvCanWrite(OPENCR_CAN_CHANNEL, id, p_data, length, format_);
}

uint32_t CANClass::write(uint32_t id, uint8_t *p_data, uint32_t length, uint8_t format)
{
    return drvCanWrite(OPENCR_CAN_CHANNEL, id, p_data, length, format);
}

uint8_t CANClass::read(void)  //read one byte
{
    return drvCanRead(OPENCR_CAN_CHANNEL);
}

uint32_t  CANClass::avaliable(void)
{
    return drvCanAvailable(OPENCR_CAN_CHANNEL); 
}

uint32_t CANClass::writeMessage(can_message_t *p_msg)
{
    drv_can_msg_t msg;
    msg.id = p_msg->id;
    msg.format = p_msg->format;
    msg.length = p_msg->length;
    memcpy(msg.data, p_msg->data, 8);

    return drvCanWriteMsg(OPENCR_CAN_CHANNEL, &msg);
}

bool CANClass::readMessage(can_message_t *p_msg)
{
    bool ret = false;
    drv_can_msg_t *rx_msg;

    rx_msg = drvCanReadMsg(OPENCR_CAN_CHANNEL);

    if(rx_msg != NULL)
    {
        p_msg->id = rx_msg->id;
        p_msg->format = rx_msg->format;
        p_msg->length = rx_msg->length;

        memcpy(p_msg->data, rx_msg->data, p_msg->length);

        ret = true;
    }

    return ret;
}

uint32_t CANClass::avaliableMessage(void)
{
     return drvCanAvailableMsg(OPENCR_CAN_CHANNEL);
}

uint8_t CANClass::getErrCount(void)
{
    return drvCanGetErrCount(OPENCR_CAN_CHANNEL);
}
uint32_t CANClass::getError(void)
{
    return drvCanGetError(OPENCR_CAN_CHANNEL);
}

uint32_t CANClass::getState(void)
{
     return drvCanGetState(OPENCR_CAN_CHANNEL);
}

void CANClass::attachRxInterrupt(void (*handler)(can_message_t *arg))
{
    drvCanAttachRxInterrupt(OPENCR_CAN_CHANNEL, (void(*)(void *arg)) handler);
}

void CANClass::detachRxInterrupt(void)
{
    drvCanDetachRxInterrupt(OPENCR_CAN_CHANNEL);
}
