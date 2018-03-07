/*
 *  main.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBPH
 */

#ifndef __OPENCR_LD_MAIN_H_
#define __OPENCR_LD_MAIN_H_

#include <unistd.h>
#include "type.h"
#include "serial.h"



#define MAGIC_NUMBER   0x5555AAAA



typedef struct
{
  uint32_t magic_number;
  char     fw_name_str[128];
  char     fw_ver_str[128];
  uint32_t fw_size;
  uint8_t  reserved[1024];
} opencr_fw_header_t;



int opencr_ld_main( int argc, const char **argv );


#endif
