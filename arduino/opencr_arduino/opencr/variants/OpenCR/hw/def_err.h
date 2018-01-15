/*
 *  def_err.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#ifndef DEF_ERR_H
#define DEF_ERR_H

#include <stdint.h>



typedef uint16_t err_code_t;




#define ERR_NONE                            0x0000
#define ERR_INVALID_CMD                     0x0001
#define ERR_FLASH_ERROR                     0x0010
#define ERR_FLASH_BUSY                      0x0011
#define ERR_FLASH_ERR_TIMEOUT               0x0012
#define ERR_FLASH_NOT_EMPTY                 0x0013
#define ERR_FLASH_WRITE                     0x0014
#define ERR_FLASH_READ                      0x0015
#define ERR_FLASH_ERASE                     0x0016
#define ERR_FLASH_PACKET_SIZE               0x0017
#define ERR_FLASH_SIZE                      0x0018
#define ERR_FLASH_CRC                       0x0019

#define ERR_MEMORY                          0x0100
#define ERR_FULL                            0x0101
#define ERR_EMPTY                           0x0102
#define ERR_NULL                            0x0103
#define ERR_INVAILD_INDEX                   0x0104


#endif
