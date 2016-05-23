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
#define ERR_FLASH_ERROR                     0x0010
#define ERR_FLASH_BUSY                      0x0011
#define ERR_FLASH_ERR_TIMEOUT               0x0012
#define ERR_FLASH_NOT_EMPTY                 0x0013
#define ERR_FLASH_WRITE                     0x0014
#define ERR_FLASH_READ                      0x0015
#define ERR_FLASH_ERASE                     0x0016

#define ERR_TIMEOUT          		    0x0020
#define ERR_MISMATCH_ID			    0x0021



#endif

