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
/*
	Based on Multiwii : https://github.com/multiwii/multiwii-firmware
*/



#include <Arduino.h> 
#include <SPI.h>
#include "MPU6050.h"


#define MPU_CS_PIN    BDPIN_SPI_CS_IMU




#define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
//#define MPU6050_ADDRESS     0x69 // address pin AD0 high (VCC)


#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  = -X; accADC[ROLL]  =  Y; accADC[YAW]  =   Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =  -Z;}



#define GYRO_DLPF_CFG   0     // GYRO_LPF_256HZ




void read_regs( uint8_t addr, uint8_t reg, uint8_t *p_data, uint32_t length )
{
  uint32_t i;
  uint8_t read_value;
  
  digitalWrite( MPU_CS_PIN, LOW);
  MPU_SPI.transfer( reg | 0x80 );  // reg | 0x80 to denote read
  for( i=0; i<length; i++ )
  {    
    p_data[i] = MPU_SPI.transfer( 0x00 );  
  }
  digitalWrite( MPU_CS_PIN, HIGH);
}


uint8_t read_reg( uint8_t addr, uint8_t reg )
{
  digitalWrite( MPU_CS_PIN, LOW);
  MPU_SPI.transfer( reg | 0x80 );                 // reg | 0x80 to denote read
  uint8_t read_value = MPU_SPI.transfer( 0x00 );  // write 8-bits zero
  digitalWrite( MPU_CS_PIN, HIGH);
  return read_value;
}


void write_reg( uint8_t addr, uint8_t reg, uint8_t value )
{
  digitalWrite( MPU_CS_PIN, LOW);
  MPU_SPI.transfer( reg );
  MPU_SPI.transfer( value );
  digitalWrite( MPU_CS_PIN, HIGH);
}




/*---------------------------------------------------------------------------
     TITLE   : cMPU6050
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
cMPU6050::cMPU6050()
{	
	calibratingG = 0;
	calibratingA = 0;
  bConnected   = false;
}



/*---------------------------------------------------------------------------
     TITLE   : cMPU6050
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cMPU6050::begin()
{  

  pinMode( MPU_CS_PIN, OUTPUT );
  
  MPU_SPI.begin();  
  MPU_SPI.setDataMode( SPI_MODE1 );
  MPU_SPI.setBitOrder( MSBFIRST );
  MPU_SPI.setClockDivider( SPI_CLOCK_DIV4 ); // 1MHz
  digitalWrite(MPU_CS_PIN, HIGH);
  delay( 500 );


  if( read_reg(MPU6050_ADDRESS, 0x75) == 0x71 )
  {
    bConnected = true;    

    gyro_init();
    acc_init();    

    //MPU_SPI.setClockDivider( SPI_CLOCK_DIV128 ); // 1MHz
  }



  return bConnected;
}



/*---------------------------------------------------------------------------
     TITLE   : gyro_init
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6050::gyro_init( void )
{
	uint8_t i;


	for( i=0; i<3; i++ )
	{
		gyroZero[i] = 0;
		accZero[i]  = 0;
	}

	write_reg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
	delay(150);
	write_reg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  delay(100);
	write_reg(MPU6050_ADDRESS, 0x1A, GYRO_DLPF_CFG);    //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  delay(100);
	write_reg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  delay(100);
	calibratingG = 512;     
}



/*---------------------------------------------------------------------------
     TITLE   : gyro_get_adc
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6050::gyro_get_adc( void ) 
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;

  uint8_t rawADC[6];

  if( bConnected == true )
  {
  	//I2C.get_six_raw_adc(MPU6050_ADDRESS, 0x43);
    read_regs( MPU6050_ADDRESS, 0x43, rawADC, 6 );
  
 		x = (((int16_t)rawADC[0]) << 8) | rawADC[1];
  	y = (((int16_t)rawADC[2]) << 8) | rawADC[3];
  	z = (((int16_t)rawADC[4]) << 8) | rawADC[5];	
  
  		GYRO_ORIENTATION(   x>>2 , // range: +/- 8192; +/- 2000 deg/sec
  							y>>2 ,
  							z>>2 );
  }
  
  gyro_common();
}





/*---------------------------------------------------------------------------
     TITLE   : gyro_cali_start
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6050::gyro_cali_start() 
{
	calibratingG = 512;
}




/*---------------------------------------------------------------------------
     TITLE   : acc_init
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6050::acc_init( void ) 
{
	write_reg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
	//note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
	//confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480
}




/*---------------------------------------------------------------------------
     TITLE   : acc_get_adc
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6050::acc_get_adc( void ) 
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;

  uint8_t rawADC[6];
  

	//I2C.get_six_raw_adc(MPU6050_ADDRESS, 0x3B);
  if( bConnected == true )
  {
    read_regs( MPU6050_ADDRESS, 0x3B, rawADC, 6 );
    
		x = (((int16_t)rawADC[0]) << 8) | rawADC[1];
		y = (((int16_t)rawADC[2]) << 8) | rawADC[3];
		z = (((int16_t)rawADC[4]) << 8) | rawADC[5];

		ACC_ORIENTATION(    x>>3 ,
							y>>3 ,
							z>>3 );
	}
 
	acc_common();
}





/*---------------------------------------------------------------------------
     TITLE   : gyro_common
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6050::gyro_common() 
{
	static int16_t previousGyroADC[3] = {0,0,0};
	static int32_t g[3];
	uint8_t axis, tilt=0;

	if (calibratingG>0) 
	{
		for (axis = 0; axis < 3; axis++) 
		{
			if (calibratingG == 512) 
			{ // Reset g[axis] at start of calibration
				g[axis]=0;                    
				previousGyroADC[axis] = gyroADC[axis];
			}
			if (calibratingG % 10 == 0) 
			{
				if(abs(gyroADC[axis] - previousGyroADC[axis]) > 8) tilt=1;
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
		gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
		previousGyroADC[axis] = gyroADC[axis];
     }
}





/*---------------------------------------------------------------------------
     TITLE   : acc_common
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6050::acc_common() 
{
	static int32_t a[3];

	if (calibratingA>0) 
	{
		calibratingA--;
		for (uint8_t axis = 0; axis < 3; axis++) 
		{
			if (calibratingA == 511) a[axis]=0;   // Reset a[axis] at start of calibration
			a[axis] += accADC[axis];           // Sum up 512 readings
			accZero[axis] = a[axis]>>9; // Calculate average, only the last itteration where (calibratingA == 0) is relevant
		}
		if (calibratingA == 0) 
		{
			accZero[YAW] -= ACC_1G;   // shift Z down by ACC_1G and store values in EEPROM at end of calibration
		}
	}


	accADC[ROLL]  -=  accZero[ROLL] ;
	accADC[PITCH] -=  accZero[PITCH];
	accADC[YAW]   -=  accZero[YAW] ;
}





/*---------------------------------------------------------------------------
     TITLE   : acc_cali_start
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6050::acc_cali_start() 
{
	calibratingA = 512;
}





/*---------------------------------------------------------------------------
     TITLE   : acc_cali_get_done
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cMPU6050::acc_cali_get_done() 
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
bool cMPU6050::gyro_cali_get_done() 
{
	if( calibratingG == 0 ) return true;
	else                    return false;
}
