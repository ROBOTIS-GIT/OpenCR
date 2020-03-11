#ifndef __IMU_SELECTOR_H__
#define __IMU_SELECTOR_H__


#include "ICM20648.h"
#include "MPU9250.h"


class cIMUDevice
{
 public:  
  bool begin( void );
  void init( void );
  void gyro_init( void );
  void gyro_get_adc( void );
  void gyro_common();
  void gyro_cali_start();
  bool gyro_cali_get_done();
  void acc_init( void );
  void acc_get_adc( void );
  void acc_common( void );
  void acc_cali_start();
  bool acc_cali_get_done();
  void mag_init( void );
  void mag_get_adc( void );
  void mag_common();
  void mag_cali_start();
  bool mag_cali_get_done();

 public : 
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

 private:
  cMPU9250 DEV1;
  cICM20648 DEV2;

  uint8_t device_model;

  enum DeviceModel
  {
    MPU9250=1,
    ICM20468
  };
};

#endif

