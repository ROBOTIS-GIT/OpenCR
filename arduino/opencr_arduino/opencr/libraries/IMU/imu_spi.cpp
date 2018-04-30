#include <Arduino.h>
#include <SPI.h>
#include "imu_spi.h"
#include "MPU9250_REGS.h"





void imu_spi_init(void)
{
  pinMode( BDPIN_SPI_CS_IMU, OUTPUT );

  SPI_IMU.begin();
  SPI_IMU.setDataMode( SPI_MODE0 );
  SPI_IMU.setBitOrder( MSBFIRST );
  SPI_IMU.setClockDivider( SPI_CLOCK_DIV128 ); // 1MHz
  digitalWrite(BDPIN_SPI_CS_IMU, HIGH);
  delay(100);
}

void imu_spi_initFast(void)
{
  SPI_IMU.begin();
  SPI_IMU.setDataMode( SPI_MODE3 );
  SPI_IMU.setBitOrder( MSBFIRST );
  SPI_IMU.setClockDivider( SPI_CLOCK_DIV8 ); // 1MHz
  digitalWrite(BDPIN_SPI_CS_IMU, HIGH);
  delay(200);
}

int  imu_spi_writes(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
  uint32_t i;

	UNUSED(slave_addr);

  digitalWrite( BDPIN_SPI_CS_IMU, LOW);
  SPI_IMU.transfer( reg_addr );

  for( i=0; i<length; i++ )
  {
    SPI_IMU.transfer( data[i] );
  }
  digitalWrite( BDPIN_SPI_CS_IMU, HIGH);
  delay_ms(1);
	return 0;
}

int imu_spi_reads(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
  uint32_t i;
  //uint8_t read_value;

	UNUSED(slave_addr);

  digitalWrite( BDPIN_SPI_CS_IMU, LOW);
  SPI_IMU.transfer( reg_addr | 0x80 );  // reg | 0x80 to denote read
  for( i=0; i<length; i++ )
  {
    data[i] = SPI_IMU.transfer( 0xFF );
  }
  digitalWrite( BDPIN_SPI_CS_IMU, HIGH);
	return 0;
}

int imu_spi_ak8963_reads(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
	uint8_t index = 0;
	uint8_t status = 0;
	uint32_t timeout = 0;
	uint8_t tmp = 0;

	tmp = akm_addr | MPU9250_I2C_READ;
	imu_spi_writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	delay(1);
	while(index < len){
		tmp = reg_addr + index;
		imu_spi_writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
		delay(1);
		tmp = MPU9250_I2C_SLV4_EN;
		imu_spi_writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		delay(1);

		do {
			if (timeout++ > 50){
				return -2;
			}
			imu_spi_reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
			delay_ms(2);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		imu_spi_reads(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data + index);
		delay_ms(1);
		index++;
	}
	return 0;
}

int imu_spi_ak8963_writes(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
  uint32_t timeout = 0;
	uint8_t status = 0;
	uint8_t tmp = 0;
	uint8_t index = 0;

	tmp = akm_addr;
	imu_spi_writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	delay_ms(2);

	while(index < len){
		tmp = reg_addr + index;
		imu_spi_writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
		delay_ms(2);
		imu_spi_writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, data + index);
		delay_ms(2);
		tmp = MPU9250_I2C_SLV4_EN;
		imu_spi_writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		delay_ms(2);

		do {
			if (timeout++ > 50)
				return -2;
			imu_spi_reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
			delay_ms(2);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		if (status & MPU9250_I2C_SLV4_NACK)
			return -3;
		index++;
	}
	return 0;
}

int imu_spi_write(uint8_t addr, uint8_t reg_addr, uint8_t data)
{
	UNUSED(addr);

	digitalWrite( BDPIN_SPI_CS_IMU, LOW);
	SPI_IMU.transfer(reg_addr);
	SPI_IMU.transfer(data);
	digitalWrite( BDPIN_SPI_CS_IMU, HIGH);
	return 0;
}

uint8_t imu_spi_read(uint8_t addr, uint8_t reg_addr)
{
	UNUSED(addr);

	uint8_t dummy = 0;
	uint8_t data = 0;

	digitalWrite( BDPIN_SPI_CS_IMU, LOW);
	SPI_IMU.transfer(0x80 | reg_addr);
	data = SPI_IMU.transfer(dummy);
	digitalWrite( BDPIN_SPI_CS_IMU, HIGH);
	return data;
}

int imu_spi_ak8963_write(uint8_t akm_addr, uint8_t reg_addr, uint8_t data)
{
  uint8_t param[1];

  param[0] = data;

  return imu_spi_ak8963_writes(akm_addr,reg_addr, 1, param);
}
