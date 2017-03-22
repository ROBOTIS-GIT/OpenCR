/*
 *  def.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram
 */

#ifndef DEF_H
#define DEF_H

#include <stdint.h>
#include <stdbool.h>

#ifndef BOOL
#define BOOL uint8_t
#endif

#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif



#include "hw_def.h"
#include "ap_def.h"




typedef void (*voidFuncPtr)(void);













#endif
