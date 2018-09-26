/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _DMA_STREAM_HANDLERS_
#define _DMA_STREAM_HANDLERS_

#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"
typedef void (*DMALoseStreamHandlerCallbackFunction_t)(uint8_t iStream);  
extern bool SetDMA1StreamHandlerHandle(uint8_t iStream, DMA_HandleTypeDef *hdma, 
        bool fOverwrite, DMALoseStreamHandlerCallbackFunction_t loseCB);
extern bool SetDMA2StreamHandlerHandle(uint8_t iStream, DMA_HandleTypeDef *hdma, 
        bool fOverwrite, DMALoseStreamHandlerCallbackFunction_t loseCB);
#ifdef __cplusplus
}
#endif


#endif  // _DMA_STREAM_HANDLERS_