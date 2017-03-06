/*
 * OLLO.h For OpenCR
 *
 *  Created on: 2013. 5. 30.
 *      Author: ROBOTIS CO,.LTD.
 */

#ifndef OLLO_H_
#define OLLO_H_
#include <Arduino.h>





typedef enum OLLO_DEVICE_INDEX {
    IR_SENSOR,
    TOUCH_SENSOR,
    GYRO_SENOSR,
    DMS_SENSOR,
    PIR_SENSOR,
    MAGNETIC_SENSOR,
    COLOR_SENSOR,
    ULTRASONIC_SENSOR,
    LED_DISPLAY,
    TEMPERATURE_SENSOR
}OlloDeviceIndex;

typedef enum COLOR_INDEX {
    RED =1 ,
    GREEN,
    BLUE,
    YELLOW,
    ORANGE,
    BLACK,
    WHITE
}ColorIndex;


#define COLOR_RED   1
#define COLOR_GREEN 2
#define COLOR_BLUE  3


#define PORT1_SIG1  40
#define PORT1_SIG2  41
#define PORT1_ADC   42

#define PORT2_SIG1  43
#define PORT2_SIG2  44
#define PORT2_ADC   45

#define PORT3_SIG1  70
#define PORT3_SIG2  71
#define PORT3_ADC   72

#define PORT4_SIG1  73
#define PORT4_SIG2  74
#define PORT4_ADC   75

#define OLLO_SLEEP  46


class OLLO {
private:
	uint8_t mportNumber;
	uint8_t mMot_plus;
	uint8_t mMot_minus;
	int detectColor(uint8_t port);
		//int color_chk();
	void setColor(ColorIndex colorIndex);
	int read(int devNum, OlloDeviceIndex device_index, ColorIndex sub_index);
public:
	OLLO();
	virtual ~OLLO();
	//General 3PIN sensors
	void begin(int devNum);
	void begin(int devNum, OlloDeviceIndex device_index);
	void begin(int devNum, OlloDeviceIndex device_index, voidFuncPtr handler);

	int read(int devNum);
	int read(int devNum, OlloDeviceIndex device_index);

//	uint8_t isGreen(uint8_t port);
//	uint8_t isWhite(uint8_t port);
//	uint8_t isBlue(uint8_t port);
//	uint8_t isBlack(uint8_t port);
//	uint8_t isRed(uint8_t port);
//	uint8_t isYellow(uint8_t port);
//	uint8_t detectColor(uint8_t port);

	void write(int devNum, uint8_t leftVal,  uint8_t rightVal);
	void write(int devNum, uint8_t leftVal, uint8_t centerVal, uint8_t rightVal);

	//IR Sensor Module
	//void beginIR(int devNum);
	//LED Module
	void writeLED(int devNum,uint8_t leftVal, uint8_t rightVal );
	//Button Module
	//void beginButton(int devNum,voidFuncPtr handler);
	//int readColor(int devNum, int colorIndex);


};

#endif /* OLLO_H_ */
