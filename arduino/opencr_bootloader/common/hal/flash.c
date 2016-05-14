/*
 *  flash.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBPH
 */

#include "flash.h"





void flash_init()
{

}


uint8_t flash_write(uint32_t addr, uint8_t *p_data, uint32_t length, uint32_t timeout )
{
  uint8_t err_code = FLASH_OK;
  uint32_t t_time;

  // TODO : flash 메모리에서 원하는 크기만큼 데이터를 Write한다.(주소 정렬은 1바이트)
  // addr : write 주소
  // p_data : 쓸려고하는 데이터를 넘겨줄 데이터 포인터
  // time_out : 타임아웃 정보이며 해당 시간을 초과하면 에러 발생시킴, 단위는 ms
  // 아래는 타임아웃을 체크하는 방법

  t_time = millis();
  while(1)
  {
    if( millis()-t_time > timeout )
    {
      err_code = FLASH_ERR_TIMEOUT;
    }
  }


  return err_code;
}

uint8_t flash_read(uint32_t addr, uint8_t *p_data, uint32_t length, uint32_t timeout )
{
  uint8_t err_code = FLASH_OK;
  uint32_t t_time;

  // TODO : flash 메모리에서 원하는 크기만큼 데이터를 Read한다.(주소 정렬은 1바이트)
  // addr : 읽을 주소
  // p_data : 읽은 데이터를 넘겨줄 데이터 포인터
  // time_out : 타임아웃 정보이며 해당 시간을 초과하면 에러 발생시킴, 단위는 ms

  t_time = millis();
  while(1)
  {
    if( millis()-t_time > timeout )
    {
      err_code = FLASH_ERR_TIMEOUT;
    }
  }


  return err_code;
}


