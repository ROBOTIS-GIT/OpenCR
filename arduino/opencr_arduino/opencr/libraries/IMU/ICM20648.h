#ifndef _ICM20648_H_
#define _ICM20648_H_

#include <inttypes.h>
#include <Arduino.h>

#include "Define.h"
#include "ICM20648_REGS.h"




#define MPU_SPI   SPI_IMU



class cICM20648
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
	cICM20648();

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


private:
  void selectBank(uint8_t bank);
  void spiRead(uint16_t addr, uint8_t *p_data, uint32_t length);
  uint8_t spiReadByte(uint16_t addr);
  void spiWriteByte(uint16_t addr, uint8_t data);

};


#endif
