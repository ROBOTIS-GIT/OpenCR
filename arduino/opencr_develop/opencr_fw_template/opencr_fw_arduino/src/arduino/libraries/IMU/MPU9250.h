//----------------------------------------------------------------------------
//    프로그램명 	: MPU6050
//
//    만든이     	: Baram ( chcbaram@paran.com )
//
//    날  짜     :
//
//    최종 수정  	:
//
//    MPU_Type	:
//
//    파일명     	: MPU6050.h
//----------------------------------------------------------------------------
#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <inttypes.h>
#include <Arduino.h>

#include "Define.h"
#include "MPU9250_REGS.h"


#define ACC_1G     512
#define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0) //16.4 LSB = 1 deg/s


#define MPU_SPI   SPI_IMU


void read_regs( uint8_t addr, uint8_t reg, uint8_t *p_data, uint32_t length );


class cMPU9250
{

public:
  bool     bConnected;

	int16_t  gyroADC[3];
	int16_t  gyroRAW[3];
	int16_t  gyroZero[3];

	int16_t  accADC[3];
	int16_t  accRAW[3];
	int16_t  accZero[3];

  int16_t  magADC[3];
	int16_t  magRAW[3];
	int16_t  magZero[3];

	int16_t  gyroData[3];
	int16_t  accSmooth[3];

	uint16_t calibratingG;
	uint16_t calibratingA;
  uint16_t calibratingM;


  int16_t AK8963_ASA[3];


public:
	cMPU9250();

  bool begin( void );
  void init( void );
	void gyro_init( void );
	void gyro_get_adc( void );
	void gyro_common();
	void gyro_cali_start();
	bool gyro_cali_get_done();

	void acc_init( void );
	void acc_get_adc( void );
	void acc_common();
	void acc_cali_start();
	bool acc_cali_get_done();

  void mag_init( void );
	void mag_get_adc( void );
	void mag_common();
	void mag_cali_start();
	bool mag_cali_get_done();

};


#endif
