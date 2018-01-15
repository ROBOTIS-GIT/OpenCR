/*
 *  flash.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#include "flash.h"



//static void               FLASH_MassErase(uint8_t VoltageRange);

void flash_init()
{

}


err_code_t flash_write(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  err_code_t err_code = ERR_NONE;
  HAL_StatusTypeDef HAL_FLASHStatus = HAL_OK;
  uint32_t StartAddress = addr;
  uint32_t WriteSize;
  uint32_t WriteData;
  uint32_t i;
  uint32_t DataIndex;


  WriteSize = length / 4; // 32Bit

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

    if( HAL_FLASHStatus != HAL_OK )
    {
        err_code = ERR_FLASH_WRITE;
      break;
    }
  }
  HAL_FLASH_Lock();

  return err_code;
}


err_code_t flash_read(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  err_code_t err_code = ERR_NONE;
  uint32_t Dataindex;
  uint32_t addr_cnt;


  Dataindex = 0;
  for (addr_cnt=0;addr_cnt<length;addr_cnt++)
  {
    p_data[Dataindex++] = *(volatile uint8_t*)(addr+addr_cnt);
  }

  return err_code;
}


err_code_t flash_erase_whole_sectors(void)
{

  err_code_t err_code = ERR_NONE;
  HAL_StatusTypeDef HAL_FLASHStatus = HAL_OK;

  HAL_FLASH_Unlock();

  //HAL_FLASHStatus = FLASH_MassErase(FLASH_VOLTAGE_RANGE_3);
  if(HAL_FLASHStatus != HAL_OK)
  {
    err_code = ERR_FLASH_ERASE;
  }

  HAL_FLASH_Lock();

  return err_code;
}

err_code_t flash_erase_fw_block( uint32_t length )
{

  err_code_t err_code = ERR_NONE;
  HAL_StatusTypeDef HAL_FLASHStatus = HAL_OK;
  FLASH_EraseInitTypeDef pEraseInit;
  uint32_t SectorError;
  uint32_t sector_cnt;


  sector_cnt = length/(256*1024);
  if( length%(256*1024) > 0 )
  {
    sector_cnt++;
  }
  if( sector_cnt > (FLASH_SECTOR_TOTAL-FLASH_SECTOR_5) )
  {
    sector_cnt = (FLASH_SECTOR_TOTAL-FLASH_SECTOR_5);
  }

  //except user bootloader sectors
  pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  pEraseInit.Sector = FLASH_SECTOR_5;
  pEraseInit.NbSectors = sector_cnt;

  HAL_FLASH_Unlock();

  HAL_FLASHStatus = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
  if(HAL_FLASHStatus != HAL_OK)
  {
    err_code = ERR_FLASH_ERASE;
  }

  HAL_FLASH_Lock();

  return err_code;
}


err_code_t flash_erase_sector(uint32_t sector)
{
  err_code_t err_code = ERR_NONE;
  HAL_StatusTypeDef HAL_FLASHStatus = HAL_OK;
  FLASH_EraseInitTypeDef pEraseInit;
  uint32_t SectorError;

  pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  pEraseInit.Sector = sector;
  pEraseInit.NbSectors = 1;

  HAL_FLASH_Unlock();

  HAL_FLASHStatus = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
  if(HAL_FLASHStatus != HAL_OK)
  {
    err_code = ERR_FLASH_ERASE;
  }

  HAL_FLASH_Lock();

  return err_code;
}

