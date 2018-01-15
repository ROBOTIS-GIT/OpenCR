#ifndef __XPT2046_H
#define __XPT2046_H

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <SPI.h>


#define BIT(__N)  (uint8_t)(1 << (__N))


#if  defined(__AVR_ATmega32U4__)

#define __XPT2046_CS_OUT()      DDRD |= BIT(4)
#define __XPT2046_CS_SET()      PORTD |=  BIT(4)
#define __XPT2046_CS_CLR()      PORTD &=~ BIT(4)

#define __XPT2046_IRQ_IN()      DDRD &=~ BIT(0);PORTD |=  BIT(0)
#define __XPT2046_IRQ_READ()    (PIND & BIT(0))

#define __XPT2046_WRITE_BYTE(__DATA)       SPI.transfer(__DATA)
          
#define __SD_CS_DISABLE()       DDRD |= BIT(6); PORTD |=  BIT(6)

#define __XPT2046_CS_DISABLE()    DDRD |= 0x10; PORTD |=  0x10

#elif defined(__AVR_ATmega328__)

#define __XPT2046_CS_OUT()      DDRD |= BIT(4)
#define __XPT2046_CS_SET()      PORTD |=  BIT(4)
#define __XPT2046_CS_CLR()      PORTD &=~ BIT(4)

#define __XPT2046_IRQ_IN()      DDRD &=~ BIT(3);PORTD |=  BIT(3)
#define __XPT2046_IRQ_READ()    (PIND & BIT(3))

#define __XPT2046_WRITE_BYTE(__DATA)       SPI.transfer(__DATA)
          
#define __SD_CS_DISABLE()       DDRD |= BIT(5); PORTD |=  BIT(5)

#define __XPT2046_CS_DISABLE()    DDRD |= 0x10; PORTD |=  0x10

#else

#define TP_IRQ_PIN         3
#define TP_CS_PIN          4
#define SD_CS_PIN          5

#define __XPT2046_CS_OUT()      pinMode(TP_CS_PIN, OUTPUT)
#define __XPT2046_CS_SET()      digitalWrite(TP_CS_PIN, HIGH)
#define __XPT2046_CS_CLR()      digitalWrite(TP_CS_PIN, LOW)

#define __XPT2046_IRQ_IN()      do { \
                                    pinMode(TP_IRQ_PIN, INPUT); \
                                    digitalWrite(TP_IRQ_PIN, HIGH); \
                                } while (0)
#define __XPT2046_IRQ_READ()    digitalRead(TP_IRQ_PIN)
#define __XPT2046_WRITE_BYTE(__DATA)       SPI.transfer(__DATA)
#define __SD_CS_DISABLE()       do { \
                                    pinMode(SD_CS_PIN, OUTPUT); \
                                    digitalWrite(SD_CS_PIN, HIGH); \
                                } while (0)
#define __XPT2046_CS_DISABLE()  __XPT2046_CS_SET()

#endif






class XPT
{

public:
    
    inline uint8_t xpt2046_write_byte(uint8_t chData)
    {
        return __XPT2046_WRITE_BYTE(chData);
    }

    uint16_t xpt2046_read_ad_value(uint8_t chCmd)
    {
        uint16_t hwData = 0;
    
        __XPT2046_CS_CLR();
        xpt2046_write_byte(chCmd);
        hwData = xpt2046_write_byte(0x00);
        hwData <<= 8;
        hwData |= xpt2046_write_byte(0x00);
        hwData >>= 4;
       __XPT2046_CS_SET();
        
        return hwData;
    }
    
#define READ_TIMES  5
#define LOST_NUM    1
    uint16_t xpt2046_read_average(uint8_t chCmd)
    {
        uint8_t i, j;
        uint16_t hwbuffer[READ_TIMES], hwSum = 0, hwTemp;

        for (i = 0; i < READ_TIMES; i ++) {
            hwbuffer[i] = xpt2046_read_ad_value(chCmd);
        }
        for (i = 0; i < READ_TIMES - 1; i ++) {
            for (j = i + 1; j < READ_TIMES; j ++) {
                if (hwbuffer[i] > hwbuffer[j]) {
                    hwTemp = hwbuffer[i];
                    hwbuffer[i] = hwbuffer[j];
                    hwbuffer[j] = hwTemp;
                }
            }
        }
        for (i = LOST_NUM; i < READ_TIMES - LOST_NUM; i ++) {
            hwSum += hwbuffer[i];
        }
        hwTemp = hwSum / (READ_TIMES - 2 * LOST_NUM);

        return hwTemp;
    }
    
    void xpt2046_init(void);
    void xpt2046_read_xy(uint16_t *phwXpos, uint16_t *phwYpos);
    bool xpt2046_twice_read_xy(uint16_t *phwXpos, uint16_t *phwYpos);
};

extern uint8_t g_chXcmd, g_chYcmd;
extern XPT Xpt;

#endif



