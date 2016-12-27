//----------------------------------------------------------------------------
//    프로그램명 	:
//
//    만든이     	: Made by Baram ( chcbaram@paran.com )
//
//    날  짜     :
//
//    최종 수정  	:
//
//    MPU_Type	:
//
//    파일명     	: IMU.ino
//----------------------------------------------------------------------------
#include <Arduino.h>
#include "IMU.h"







/*---------------------------------------------------------------------------
     TITLE   : BLE
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
cIMU::cIMU()
{
  uint8_t i;


  for( i=0; i<3; i++ )
  {
    rpy[i] = 0.;

  }

	bConnected = false;
}




/*---------------------------------------------------------------------------
     TITLE   : begin
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
uint8_t cIMU::begin( uint32_t hz )
{
	uint8_t err_code = IMU_OK;


  update_hz = hz;
  update_us = 1000000/hz;

  aRes = 8.0/32768.0;      // 8g
  gRes = 2000.0/32768.0;   // 2000dps
  //mRes = 10.*4912./8190.;  // 14BIT
  mRes = 10.*4912./32760.; // 16BIT

  bConnected = SEN.begin();

  if( bConnected == true )
  {
    filter.begin(update_hz);
  }


	return err_code;
}





/*---------------------------------------------------------------------------
     TITLE   : update
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
uint16_t cIMU::update( uint32_t option )
{
	uint16_t ret_time = 0;

	static uint32_t tTime;


	if( (micros()-tTime) >= update_us )
	{
		ret_time = micros()-tTime;
    tTime = micros();

		computeIMU();

		gyroData[0] = SEN.gyroData[0];
		gyroData[1] = SEN.gyroData[1];
		gyroData[2] = SEN.gyroData[2];
	}

	return ret_time;
}



/*---------------------------------------------------------------------------
     TITLE   : compute
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cIMU::computeIMU( void )
{
  static uint32_t prev_process_time = micros();
  static uint32_t cur_process_time = 0;
  static uint32_t process_time = 0;
  uint32_t i;


	SEN.acc_get_adc();
	SEN.gyro_get_adc();
  SEN.mag_get_adc();

  for( i=0; i<3; i++ )
  {
    accRaw[i]   = SEN.accRAW[i];
    accData[i]  = SEN.accADC[i];
    gyroRaw[i]  = SEN.accRAW[i];
    gyroData[i] = SEN.accADC[i];
    magRaw[i]   = SEN.magRAW[i];
    magData[i]  = SEN.magADC[i];
  }

  ax = (float)SEN.accADC[0]*aRes;
  ay = (float)SEN.accADC[1]*aRes;
  az = (float)SEN.accADC[2]*aRes;

  gx = (float)SEN.gyroADC[0]*gRes;
  gy = (float)SEN.gyroADC[1]*gRes;
  gz = (float)SEN.gyroADC[2]*gRes;

  mx = (float)SEN.magADC[0]*mRes;
  my = (float)SEN.magADC[1]*mRes;
  mz = (float)SEN.magADC[2]*mRes;


  cur_process_time  = micros();
  process_time      = cur_process_time-prev_process_time;
  prev_process_time = cur_process_time;

  filter.invSampleFreq = (float)process_time/1000000.0f;
  filter.updateIMU(gx, gy, gz, ax, ay, az);
  //filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);


  rpy[0] = filter.getRoll();
  rpy[1] = filter.getPitch();
  rpy[2] = filter.getYaw();

  quat[0] = filter.q0;
  quat[1] = filter.q1;
  quat[2] = filter.q2;
  quat[3] = filter.q3;

  angle[0] = (int16_t)(rpy[0] * 10.);
  angle[1] = (int16_t)(rpy[1] * 10.);
  angle[2] = (int16_t)(rpy[1] * 1.);

}
