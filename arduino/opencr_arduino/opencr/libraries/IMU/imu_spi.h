#ifndef _IMU_SPI_H_
#define _IMU_SPI_H_

#if defined(__cplusplus)
extern "C" {
#endif

#include <inttypes.h>

void imu_spi_init(void);
void imu_spi_initFast(void);
int  imu_spi_reads(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);
int  imu_spi_writes(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);
int  imu_spi_ak8963_reads(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data);
int  imu_spi_ak8963_writes(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data);

int imu_spi_ak8963_write(uint8_t akm_addr, uint8_t reg_addr, uint8_t data);

uint8_t imu_spi_read(uint8_t addr, uint8_t reg_addr);
int     imu_spi_write(uint8_t addr, uint8_t reg_addr, uint8_t data);


#if defined(__cplusplus)
}
#endif

#endif
