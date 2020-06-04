#include "imu_selector.h"

bool cIMUDevice::begin( void )
{
  bool result = false;

  if(DEV1.begin() == true)
  {
    device_model = MPU9250;
    result = true;
  }
  else if (DEV2.begin() == true)
  {
    device_model = ICM20468;
    result = true;
  }
  
  return result;
}

void cIMUDevice::init( void )
{
  switch(device_model)
  {
    case MPU9250:
      DEV1.init();
      break;
    case ICM20468:
      DEV2.init();
      break;
    default : break;
  }
}

void cIMUDevice::gyro_init( void )
{
  switch(device_model)
  {
    case MPU9250:
      DEV1.gyro_init();
      break;
    case ICM20468:
      DEV2.gyro_init();
      break;
    default : break;
  }
}

void cIMUDevice::gyro_get_adc( void )
{
  switch(device_model)
  {
    case MPU9250:
      DEV1.gyro_get_adc();
      memcpy(gyroRAW,DEV1.gyroRAW,3*sizeof(int16_t));
      memcpy(gyroADC,DEV1.gyroADC,3*sizeof(int16_t));
      break;
    case ICM20468:
      DEV2.gyro_get_adc();
      memcpy(gyroRAW,DEV2.gyroRAW,3*sizeof(int16_t));
      memcpy(gyroADC,DEV2.gyroADC,3*sizeof(int16_t));
      break;
    default : break;
  }
}

void cIMUDevice::gyro_common()
{
  switch(device_model)
  {
    case MPU9250:
      DEV1.gyro_common();
      break;
    case ICM20468:
      DEV2.gyro_common();
      break;
    default : break;
  }
}

void cIMUDevice::gyro_cali_start()
{
  switch(device_model)
  {
    case MPU9250:
      DEV1.gyro_cali_start();
      break;
    case ICM20468:
      DEV2.gyro_cali_start();
      break;
    default : break; 
  }
}

bool cIMUDevice::gyro_cali_get_done()
{
  bool result = false;

  switch(device_model)
  {
    case MPU9250:
      result = DEV1.gyro_cali_get_done();
      break;
    case ICM20468:
      result = DEV2.gyro_cali_get_done();
      break;
    default : break;
  }

  return result;
}

void cIMUDevice::acc_init( void )
{
  switch(device_model)
  {
    case MPU9250:
      DEV1.acc_init();
      break;
    case ICM20468:
      DEV2.acc_init();
      break;
    default : break;
  }
}

void cIMUDevice::acc_get_adc( void )
{
  switch(device_model)
  {
    case MPU9250:
      DEV1.acc_get_adc();
      memcpy(accRAW,DEV1.accRAW,3*sizeof(int16_t));
      memcpy(accADC,DEV1.accADC,3*sizeof(int16_t));
      break;
    case ICM20468:
      DEV2.acc_get_adc();
      memcpy(accRAW,DEV2.accRAW,3*sizeof(int16_t));
      memcpy(accADC,DEV2.accADC,3*sizeof(int16_t));
      break;
    default : break;
  }
}

void cIMUDevice::acc_common( void )
{
  switch(device_model)
  {
    case MPU9250:
      DEV1.acc_common();
      break;
    case ICM20468:
      DEV2.acc_common();
      break;
    default : break;
  }
}

void cIMUDevice::acc_cali_start()
{
  switch(device_model)
  {
    case MPU9250:
      DEV1.acc_cali_start();
      break;
    case ICM20468:
      DEV2.acc_cali_start();
      break;
    default : break;
  }
}

bool cIMUDevice::acc_cali_get_done()
{
  bool result = false;

  switch(device_model)
  {
    case MPU9250:
    result = DEV1.acc_cali_get_done();
    break;
    case ICM20468:
    result = DEV2.acc_cali_get_done();
    break;
    default : break;
  }
  return result;
}

void cIMUDevice::mag_init( void )
{
  switch(device_model)
  {
    case MPU9250:
      DEV1.mag_init();
      break;
    case ICM20468:
      DEV2.mag_init();
      break;
    default : break;
  }
}

void cIMUDevice::mag_get_adc( void )
{
  switch(device_model)
  {
    case MPU9250:
      DEV1.mag_get_adc();
      memcpy(magRAW,DEV1.magRAW,3*sizeof(int16_t));
      memcpy(magADC,DEV1.magADC,3*sizeof(int16_t));
      break;
    case ICM20468:
      DEV2.mag_get_adc();
      memcpy(magRAW,DEV2.magRAW,3*sizeof(int16_t));
      memcpy(magADC,DEV2.magADC,3*sizeof(int16_t));
      break;
    default : break;
  }
}
void cIMUDevice::mag_common()
{
  switch(device_model)
  {
    case MPU9250:
      DEV1.mag_common();
      break;
    case ICM20468:
      DEV2.mag_common();
      break;
    default : break;
  }
}
void cIMUDevice::mag_cali_start()
{
  switch(device_model)
  {
    case MPU9250:
      DEV1.mag_cali_start();
      break;
    case ICM20468:
      DEV2.mag_cali_start();
      break;
    default : break;
  }
}

bool cIMUDevice::mag_cali_get_done()
{
  bool result = false;

  switch(device_model)
  {
    case MPU9250:
      result = DEV1.mag_cali_get_done();
      break;
    case ICM20468:
      result = DEV2.mag_cali_get_done();
      break;
    default : break;
  }

  return result;
}