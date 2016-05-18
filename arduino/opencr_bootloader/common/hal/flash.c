#include "flash.h"


#define HW_FLASH_SECTOR0_START_ADDRESS 	((uint32_t)0x08000000)
#define HW_FLASH_SECTOR1_START_ADDRESS 	((uint32_t)0x08008000)
#define HW_FLASH_SECTOR2_START_ADDRESS 	((uint32_t)0x08010000)
#define HW_FLASH_SECTOR3_START_ADDRESS 	((uint32_t)0x08018000)
#define HW_FLASH_SECTOR4_START_ADDRESS 	((uint32_t)0x08020000)
#define HW_FLASH_SECTOR5_START_ADDRESS 	((uint32_t)0x08040000)
#define HW_FLASH_SECTOR6_START_ADDRESS 	((uint32_t)0x08080000)
#define HW_FLASH_SECTOR7_START_ADDRESS 	((uint32_t)0x080C0000)

#define HW_FLASH_SECTOR0_SIZE    	((uint16_t)0x00007FFF)
#define HW_FLASH_SECTOR1_SIZE    	((uint16_t)0x00007FFF)
#define HW_FLASH_SECTOR2_SIZE    	((uint16_t)0x00007FFF)
#define HW_FLASH_SECTOR3_SIZE    	((uint16_t)0x00007FFF)
#define HW_FLASH_SECTOR4_SIZE    	((uint16_t)0x0001FFFF)
#define HW_FLASH_SECTOR5_SIZE    	((uint16_t)0x0003FFFF)
#define HW_FLASH_SECTOR6_SIZE    	((uint16_t)0x0003FFFF)
#define HW_FLASH_SECTOR7_SIZE    	((uint16_t)0x0003FFFF)

#define HW_FLASH_WRITE_BUFSIZE			256


void flash_init()
{
	//flash_erase_whole_sectors();
}

uint8_t flash_erase_whole_sectors(void)
{
	FLASH_StatusTypeDef err_code = FLASH_OK;
	FLASH_EraseInitTypeDef pEraseInit;
	uint32_t SectorError;

	//except user bootloader sectors
	pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	pEraseInit.Sector = FLASH_SECTOR_2;
	pEraseInit.NbSectors = FLASH_SECTOR_TOTAL - FLASH_SECTOR_2;

	HAL_FLASH_Unlock();

	HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
	if(SectorError != 0xFFFFFFFF)
	{
		err_code = FLASH_ERROR;
	}

	HAL_FLASH_Lock();
	return err_code;
}

uint8_t flash_erase_sector(uint32_t sector)
{
	FLASH_StatusTypeDef err_code = FLASH_OK;
	FLASH_EraseInitTypeDef pEraseInit;
	uint32_t SectorError;

	pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	pEraseInit.Sector = sector;
	pEraseInit.NbSectors = 1;

	HAL_FLASH_Unlock();

	HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
	if(SectorError != 0xFFFFFFFF) err_code = FLASH_ERROR;

	HAL_FLASH_Lock();
	return err_code;
}


uint8_t flash_write(uint32_t addr, uint8_t *p_data, uint32_t length )
{

  //PBHP_160514
	HAL_StatusTypeDef HAL_FLASHStatus = HAL_OK;
	FLASH_StatusTypeDef err_code = FLASH_OK;
	uint32_t t_time;

	uint32_t StartAddress = addr;
	uint32_t WriteSize;
	uint32_t WriteData;
	uint32_t i;
	uint32_t DataIndex;

	WriteSize = length / 4;	// 32Bit占쏙옙 占쏙옙占쏙옙

	if( (WriteSize%4) > 0 ) WriteSize++;

	DataIndex = 0;
	HAL_FLASH_Unlock();
	for( i=0; i<WriteSize; i++ )
	{
		WriteData  = p_data[ DataIndex++ ] << 0;
		WriteData |= p_data[ DataIndex++ ] << 8;
		WriteData |= p_data[ DataIndex++ ] << 16;
		WriteData |= p_data[ DataIndex++ ] << 24;

		HAL_FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartAddress+i*4, (uint64_t)WriteData);

		if( HAL_FLASHStatus != FLASH_OK )
		{
		  	err_code = HAL_FLASHStatus;
			break;
		}
	}
	HAL_FLASH_Lock();
  return err_code;
}


uint8_t flash_read(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  uint8_t err_code = FLASH_OK;

	uint32_t Dataindex;
	uint32_t addr_cnt;

	Dataindex = 0;
	for (addr_cnt=0;addr_cnt<length;addr_cnt++)
	{
		p_data[Dataindex++] = *(__IO uint8_t*)(addr+addr_cnt);
	}
	return err_code;
}



