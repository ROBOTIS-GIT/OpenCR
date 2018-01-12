#ifndef __TOUCH_H_
#define __TOUCH_H_

#include <Arduino.h>
#include <avr/pgmspace.h>


#define TP_PRESS_DOWN           0x80
#define TP_PRESSED              0x40

typedef struct {
	uint16_t hwXpos0;
	uint16_t hwYpos0;
	uint16_t hwXpos;
	uint16_t hwYpos;
	uint8_t chStatus;
	uint8_t chType;
	short iXoff;
	short iYoff;
	float fXfac;
	float fYfac;
} tp_dev_t;

class TP
{
private :
	tp_dev_t s_tTouch;
	uint8_t tp_scan(uint8_t chCoordType);
	
	
	void tp_show_info(uint16_t hwXpos0, uint16_t hwYpos0,
						 uint16_t hwXpos1, uint16_t hwYpos1,
						 uint16_t hwXpos2, uint16_t hwYpos2,
						 uint16_t hwXpos3, uint16_t hwYpos3, uint16_t hwFac);
	

public:
	
	void tp_show_single_info(uint16_t* pos);
	void tp_init(void);
	void tp_adjust(void);
	void tp_dialog(void);
	void tp_draw_board(void);
	void tp_draw_big_point(uint16_t hwXpos, uint16_t hwYpos, uint16_t hwColor);
	void tp_draw_touch_point(uint16_t hwXpos, uint16_t hwYpos, uint16_t hwColor);
};

extern TP Tp;

#endif



