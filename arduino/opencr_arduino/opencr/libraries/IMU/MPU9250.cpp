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
//    파일명     	: MPU6050.cpp
//----------------------------------------------------------------------------



#include <Arduino.h>
#include <SPI.h>
#include "MPU9250.h"
#include "imu_spi.h"


#define MPU_CS_PIN          BDPIN_SPI_CS_IMU
#define MPU9250_ADDRESS     0x68
#define MPU_CALI_COUNT      512


//#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  = -X; accADC[ROLL]  =  Y; accADC[YAW]  =   Z;}
//#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}

#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  =  Y; accADC[ROLL]  =  X; accADC[YAW]  =   Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}







/*---------------------------------------------------------------------------
     TITLE   : cMPU9250
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
cMPU9250::cMPU9250()
{
	calibratingG = 0;
	calibratingA = 0;
  calibratingM = 0;
  bConnected   = false;
}



/*---------------------------------------------------------------------------
     TITLE   : cMPU9250
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cMPU9250::begin()
{

  pinMode( MPU_CS_PIN, OUTPUT );

  MPU_SPI.begin();
  MPU_SPI.setDataMode( SPI_MODE3 );
  MPU_SPI.setBitOrder( MSBFIRST );
  MPU_SPI.setClockDivider( SPI_CLOCK_DIV128 ); // 108Mhz/128 = 0.8MHz
  digitalWrite(MPU_CS_PIN, HIGH);
  delay( 100 );


  if( imu_spi_read(MPU9250_ADDRESS, 0x75) == 0x71 )
  {
    bConnected = true;
    init();
    gyro_init();
    acc_init();
    mag_init();

    MPU_SPI.setClockDivider( SPI_CLOCK_DIV8 ); // 13MHz
  }



  return bConnected;
}


void cMPU9250::init( void )
{
  uint8_t state;
  uint8_t data;
  uint8_t response[3] = {0, 0, 0};


  //MPU9250 Reset
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_1, MPU9250_RESET);
	delay(100);
	//MPU9250 Set Clock Source
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_1,  MPU9250_CLOCK_PLLGYROZ);
	delay(1);


	//MPU9250 Set Interrupt
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_INT_PIN_CFG,  MPU9250_INT_ANYRD_2CLEAR);
	delay(1);
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_INT_ENABLE, ENABLE);
	delay(1);
	//MPU9250 Set Sensors
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_2, MPU9250_XYZ_GYRO & MPU9250_XYZ_ACCEL);
	delay(1);
	//MPU9250 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_SMPLRT_DIV, SMPLRT_DIV);
	delay(1);
	//MPU9250 Set Full Scale Gyro Range
	//Fchoice_b[1:0] = [00] enable DLPF
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_GYRO_CONFIG, (MPU9250_FSR_2000DPS << 3));
	delay(1);
	//MPU9250 Set Full Scale Accel Range PS:2G
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG, (MPU9250_FSR_2G << 3));
	delay(1);
	//MPU9250 Set Accel DLPF
	data = imu_spi_read(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG2);
	data |= MPU9250_ACCEL_DLPF_41HZ;
	delay(1);
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG2, data);
	delay(1);
	//MPU9250 Set Gyro DLPF
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_CONFIG, MPU9250_GYRO_DLPF_41HZ);
	delay(1);



  //MPU9250 Set SPI Mode
	state = imu_spi_read(MPU9250_ADDRESS, MPU9250_USER_CTRL);
	delay(1);
	imu_spi_write(MPU9250_ADDRESS, MPU9250_USER_CTRL, state | MPU9250_I2C_IF_DIS);
	delay(1);
	state = imu_spi_read(MPU9250_ADDRESS, MPU9250_USER_CTRL);
	delay(1);
	imu_spi_write(MPU9250_ADDRESS, MPU9250_USER_CTRL, state | MPU9250_I2C_MST_EN);
	delay(1);
	//////////////////////////////////////////////////////////////////////////
	//AK8963 Setup
	//reset AK8963
	imu_spi_ak8963_write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL2, MPU9250_AK8963_CNTL2_SRST);
	delay(1);

	imu_spi_ak8963_write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	delay(1);
	imu_spi_ak8963_write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);
	delay(1);


  imu_spi_ak8963_reads(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_WIA, 3, response);
	//
	//AK8963 get calibration data
	imu_spi_ak8963_reads(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_ASAX, 3, response);
	AK8963_ASA[0] = (int16_t)(response[0]) + 128;
	AK8963_ASA[1] = (int16_t)(response[1]) + 128;
	AK8963_ASA[2] = (int16_t)(response[2]) + 128;
	delay(1);

	imu_spi_ak8963_write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	delay(1);
	//
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_CTRL, 0x5D);
	delay(1);
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ);
	delay(1);
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_REG, MPU9250_AK8963_ST1);
	delay(1);
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_CTRL, 0x88);
	delay(1);
	//
	imu_spi_ak8963_write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
	delay(1);

	//
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 0x09);
	delay(1);
	//
	imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_DELAY_CTRL, 0x81);
	delay(100);
}


/*---------------------------------------------------------------------------
     TITLE   : gyro_init
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::gyro_init( void )
{
	uint8_t i;


	for( i=0; i<3; i++ )
	{
    gyroADC[i]  = 0;
		gyroZero[i] = 0;
		gyroRAW[i]  = 0;
	}

	calibratingG = MPU_CALI_COUNT;
}



/*---------------------------------------------------------------------------
     TITLE   : gyro_get_adc
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::gyro_get_adc( void )
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;

  uint8_t rawADC[6];

  if( bConnected == true )
  {
    imu_spi_reads( MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, rawADC );

 		x = (((int16_t)rawADC[0]) << 8) | rawADC[1];
  	y = (((int16_t)rawADC[2]) << 8) | rawADC[3];
  	z = (((int16_t)rawADC[4]) << 8) | rawADC[5];

  	gyroRAW[0] = x;
  	gyroRAW[1] = y;
  	gyroRAW[2] = z;

  	GYRO_ORIENTATION( x, y,z );
  }

  gyro_common();
}





/*---------------------------------------------------------------------------
     TITLE   : gyro_cali_start
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::gyro_cali_start()
{
	calibratingG = MPU_CALI_COUNT;
}




/*---------------------------------------------------------------------------
     TITLE   : acc_init
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::acc_init( void )
{
  uint8_t i;


  for( i=0; i<3; i++ )
  {
    accADC[i]   = 0;
		accZero[i]  = 0;
    accRAW[i]   = 0;
  }
}




/*---------------------------------------------------------------------------
     TITLE   : acc_get_adc
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::acc_get_adc( void )
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;

  uint8_t rawADC[6];



  if( bConnected == true )
  {
    imu_spi_reads( MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, rawADC );

    x = (((int16_t)rawADC[0]) << 8) | rawADC[1];
    y = (((int16_t)rawADC[2]) << 8) | rawADC[3];
    z = (((int16_t)rawADC[4]) << 8) | rawADC[5];

    accRAW[0] = x;
    accRAW[1] = y;
    accRAW[2] = z;

		ACC_ORIENTATION( x,	y, z );
	}

	acc_common();
}





/*---------------------------------------------------------------------------
     TITLE   : gyro_common
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::gyro_common()
{
	static int16_t previousGyroADC[3];
	static int32_t g[3];
	uint8_t axis, tilt=0;

	memset(previousGyroADC, 0, 3);

	if (calibratingG>0)
	{
		for (axis = 0; axis < 3; axis++)
		{
			if (calibratingG == MPU_CALI_COUNT)
			{ // Reset g[axis] at start of calibration
				g[axis]=0;
				previousGyroADC[axis] = gyroADC[axis];
			}
			if (calibratingG % 10 == 0)
			{
				//if(abs(gyroADC[axis] - previousGyroADC[axis]) > 8) tilt=1;

				previousGyroADC[axis] = gyroADC[axis];
			}
			g[axis] += gyroADC[axis]; // Sum up 512 readings
			gyroZero[axis]=g[axis]>>9;

			if (calibratingG == 1)
			{
				//SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_ELSE);
			}
		}

		if(tilt)
		{
			calibratingG=1000;
		}
		else
		{
			calibratingG--;
		}
		return;
	}


  for (axis = 0; axis < 3; axis++)
  {
    gyroADC[axis] -= gyroZero[axis];

    //anti gyro glitch, limit the variation between two consecutive readings
    //gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
    previousGyroADC[axis] = gyroADC[axis];
  }
}





/*---------------------------------------------------------------------------
     TITLE   : acc_common
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::acc_common()
{
	static int32_t a[3];

	if (calibratingA>0)
	{
		calibratingA--;
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			if (calibratingA ==(MPU_CALI_COUNT-1)) a[axis]=0;  // Reset a[axis] at start of calibration
			a[axis] += accADC[axis];             // Sum up 512 readings
			accZero[axis] = a[axis]>>9;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant
		}
		if (calibratingA == 0)
		{
			//accZero[YAW] -= ACC_1G;
      accZero[YAW] = 0;
		}
	}

  accADC[ROLL]  -=  accZero[ROLL] ;
  accADC[PITCH] -=  accZero[PITCH];
  accADC[YAW]   -=  accZero[YAW] ;
}





/*---------------------------------------------------------------------------
     TITLE   : acc_init
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::mag_init( void )
{
  uint8_t i;

  for( i=0; i<3; i++ )
  {
    magADC[i]   = 0;
		magZero[i]  = 0;
    magRAW[i]   = 0;
  }
}




/*---------------------------------------------------------------------------
     TITLE   : mag_get_adc
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::mag_get_adc( void )
{
	// int16_t x = 0;
	// int16_t y = 0;
	// int16_t z = 0;

  uint8_t data[8];



  if( bConnected == true )
  {
  	imu_spi_reads(MPU9250_ADDRESS, MPU9250_EXT_SENS_DATA_00, 8, data);

  	if (!(data[0] & MPU9250_AK8963_DATA_READY) || (data[0] & MPU9250_AK8963_DATA_OVERRUN))
    {
  		return;
  	}
  	if (data[7] & MPU9250_AK8963_OVERFLOW)
    {
  		return;
  	}
  	magRAW[0] = (data[2] << 8) | data[1];
  	magRAW[1] = (data[4] << 8) | data[3];
  	magRAW[2] = (data[6] << 8) | data[5];

  	magRAW[0] = ((long)magRAW[0] * AK8963_ASA[0]) >> 8;
  	magRAW[1] = ((long)magRAW[1] * AK8963_ASA[1]) >> 8;
  	magRAW[2] = ((long)magRAW[2] * AK8963_ASA[2]) >> 8;
	}

	mag_common();
}





/*---------------------------------------------------------------------------
     TITLE   : mag_common
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::mag_common()
{
  magADC[0] = magRAW[0];
  magADC[1] = magRAW[1];
  magADC[2] = magRAW[2];
}





/*---------------------------------------------------------------------------
     TITLE   : acc_cali_start
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::acc_cali_start()
{
	calibratingA = MPU_CALI_COUNT;
}





/*---------------------------------------------------------------------------
     TITLE   : acc_cali_get_done
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cMPU9250::acc_cali_get_done()
{
	if( calibratingA == 0 ) return true;
	else                    return false;
}




/*---------------------------------------------------------------------------
     TITLE   : gyro_cali_get_done
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cMPU9250::gyro_cali_get_done()
{
	if( calibratingG == 0 ) return true;
	else                    return false;
}
