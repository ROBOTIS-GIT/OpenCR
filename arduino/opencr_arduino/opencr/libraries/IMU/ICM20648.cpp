#include <Arduino.h>
#include <SPI.h>
#include "ICM20648.h"




//#define SPI_CS_PIN          BDPIN_SPI_CS_IMU
                            
#define ICM20648_ADDRESS    0x68
#define MPU_CALI_COUNT      512


#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  =  Y; accADC[ROLL]  =  X; accADC[YAW]  =   Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}







cICM20648::cICM20648()
{
	calibratingG = 0;
	calibratingA = 0;
  calibratingM = 0;
  bConnected   = false;
}

void cICM20648::selectBank(uint8_t bank)
{
  digitalWrite( BDPIN_SPI_CS_IMU, LOW);

  /* clear R/W bit - write, send the address */
  MPU_SPI.write((uint8_t)ICM20648_REG_BANK_SEL);
  MPU_SPI.write((uint8_t)(bank << 4));
 
  digitalWrite( BDPIN_SPI_CS_IMU, HIGH);
}

void cICM20648::spiRead(uint16_t addr, uint8_t *p_data, uint32_t length)
{
  uint8_t reg_addr;
  uint8_t bank;
 
  reg_addr = (uint8_t) (addr & 0x7F);
  bank = (uint8_t) (addr >> 7);

  selectBank(bank);

  digitalWrite( BDPIN_SPI_CS_IMU, LOW);
 
  MPU_SPI.transfer(0x80 | reg_addr);
  MPU_SPI.transfer(NULL, (void *)p_data, (size_t)length);
 
  digitalWrite( BDPIN_SPI_CS_IMU, HIGH);
}

uint8_t cICM20648::spiReadByte(uint16_t addr)
{
  uint8_t data;

  spiRead(addr, &data, 1);

	return data;
}

void cICM20648::spiWriteByte(uint16_t addr, uint8_t data)
{
  uint8_t reg_addr;
  uint8_t bank;
 
  reg_addr = (uint8_t) (addr & 0x7F);
  bank = (uint8_t) (addr >> 7);
 
  selectBank(bank);
 
  digitalWrite( BDPIN_SPI_CS_IMU, LOW); 

  MPU_SPI.transfer(reg_addr & 0x7F);
  MPU_SPI.transfer(data);
 
  digitalWrite( BDPIN_SPI_CS_IMU, HIGH); 
}

bool cICM20648::begin()
{
  uint8_t data;

  pinMode( BDPIN_SPI_CS_IMU, OUTPUT );

  MPU_SPI.begin();
  MPU_SPI.setDataMode( SPI_MODE3 );
  MPU_SPI.setBitOrder( MSBFIRST );
  MPU_SPI.setClockDivider( SPI_CLOCK_DIV128 ); // 108Mhz/128 = 0.8MHz
  digitalWrite(BDPIN_SPI_CS_IMU, HIGH);
  delay( 100 );

  // Disable I2C interface, use SPI
  spiWriteByte(ICM20648_REG_USER_CTRL, ICM20648_BIT_I2C_IF_DIS);

  data = spiReadByte(ICM20648_REG_WHO_AM_I);

  if(data == ICM20648_DEVICE_ID)
  {
    bConnected = true;
    init();
    gyro_init();
    acc_init();
    mag_init();

    MPU_SPI.setClockDivider( SPI_CLOCK_DIV4 ); // 6.5MHz
  }



  return bConnected;
}

void cICM20648::init( void )
{
  uint8_t state;
  uint8_t data;
  uint8_t response[3] = {0, 0, 0};


  //ICM20648 Reset
  spiWriteByte(ICM20648_REG_PWR_MGMT_1, ICM20648_BIT_H_RESET);
	delay(100);

	//ICM20648 Set Clock Source
  // Auto selects the best available clock source  PLL if ready, else use the Internal oscillator
  spiWriteByte(ICM20648_REG_PWR_MGMT_1, ICM20648_BIT_CLK_PLL); 
  // PLL startup time - maybe it is too long but better be on the safe side, no spec in the datasheet */
  delay(30);  

	//ICM20648 Set Interrupt
  spiWriteByte(ICM20648_REG_INT_PIN_CFG, ICM20648_BIT_INT_ACTL | ICM20648_BIT_INT_OPEN);
	delay(1);

	//ICM20648 Set Sensors
  spiWriteByte(ICM20648_REG_PWR_MGMT_2, 0x00); // Acc/Gyro Enable
	delay(1);

	//ICM20648 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
  spiWriteByte(ICM20648_REG_GYRO_SMPLRT_DIV, 0x00); // 
	delay(1);

	//ICM20648 Gyro 

	// Set Full Scale Gyro Range
  data = ICM20648_GYRO_FULLSCALE_2000DPS;
  // Set Gyro DLPF
  data |= ICM20648_GYRO_BW_51HZ;
  spiWriteByte(ICM20648_REG_GYRO_CONFIG_1, data);
	delay(1);

	//ICM20648 Accel
	
  // Set Full Scale Accel Range
  data = ICM20648_ACCEL_FULLSCALE_2G;
  // Set Accel DLPF
  data |= ICM20648_ACCEL_BW_50HZ;
  spiWriteByte(ICM20648_REG_ACCEL_CONFIG, data);
	delay(1);
}

void cICM20648::gyro_init( void )
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

void cICM20648::gyro_get_adc( void )
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;

  uint8_t rawADC[6];

  if( bConnected == true )
  {
    spiRead(ICM20648_REG_GYRO_XOUT_H_SH, &rawADC[0], 6);

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

void cICM20648::gyro_cali_start()
{
	calibratingG = MPU_CALI_COUNT;
}

void cICM20648::acc_init( void )
{
  uint8_t i;


  for( i=0; i<3; i++ )
  {
    accADC[i]   = 0;
		accZero[i]  = 0;
    accRAW[i]   = 0;
  }
}

void cICM20648::acc_get_adc( void )
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
  uint8_t rawADC[6];

  if( bConnected == true )
  {    
    spiRead(ICM20648_REG_ACCEL_XOUT_H_SH, &rawADC[0], 6);

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

void cICM20648::gyro_common()
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

void cICM20648::acc_common()
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

void cICM20648::mag_init( void )
{
  uint8_t i;

  for( i=0; i<3; i++ )
  {
    magADC[i]   = 0;
		magZero[i]  = 0;
    magRAW[i]   = 0;
  }
}

void cICM20648::mag_get_adc( void )
{
  return;
}

void cICM20648::mag_common()
{
  magADC[0] = magRAW[0];
  magADC[1] = magRAW[1];
  magADC[2] = magRAW[2];
}

void cICM20648::acc_cali_start()
{
	calibratingA = MPU_CALI_COUNT;
}

bool cICM20648::acc_cali_get_done()
{
	if( calibratingA == 0 ) return true;
	else                    return false;
}

bool cICM20648::gyro_cali_get_done()
{
	if( calibratingG == 0 ) return true;
	else                    return false;
}
