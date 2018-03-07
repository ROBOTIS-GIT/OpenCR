/*
 *  def_err.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBPH
 */

#ifndef DEF_ERR_H
#define DEF_ERR_H

#include <stdint.h>



typedef uint16_t err_code_t;




#define OK                                  0x0000
#define ERR_FLASH_ERROR                     0xF010
#define ERR_FLASH_BUSY                      0xF011
#define ERR_FLASH_ERR_TIMEOUT               0xF012
#define ERR_FLASH_NOT_EMPTY                 0xF013
#define ERR_FLASH_WRITE                     0xF014
#define ERR_FLASH_READ                      0xF015
#define ERR_FLASH_ERASE                     0xF016

#define ERR_TIMEOUT                         0xF020
#define ERR_MISMATCH_ID                     0xF021
#define ERR_SIZE_OVER                       0xF022



#endif

