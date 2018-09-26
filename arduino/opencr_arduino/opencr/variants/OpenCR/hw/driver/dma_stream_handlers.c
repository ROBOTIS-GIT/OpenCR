/*
 *  dma_stream_handlers.c
 *
 *  Created on: 2018. 9. 22.
 *      Author: Kurt
 */
#include "dma_stream_handlers.h"
#include "vcp.h"

//=============================================================================

//=============================================================================

DMA_HandleTypeDef *DMA1_hdmas[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

DMALoseStreamHandlerCallbackFunction_t DMA1_loseHandlerCBs[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};  

DMA_HandleTypeDef *DMA2_hdmas[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

DMALoseStreamHandlerCallbackFunction_t DMA2_loseHandlerCBs[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};  

//=============================================================================
// Set the handle for one of the DMA1 streams 
// In some cases we allow the handle to overwrite existing set ones
//=============================================================================
bool SetDMA1StreamHandlerHandle(uint8_t iStream, DMA_HandleTypeDef *hdma, 
	bool fOverwrite,DMALoseStreamHandlerCallbackFunction_t loseCB)
{
	//vcp_printf("SetDMA1Stream: %d %x %x %x\n", iStream, (uint32_t)hdma, fOverwrite, (uint32_t)loseCB);

	if (iStream >= (sizeof(DMA1_hdmas)/sizeof(DMA1_hdmas[0]))) return false; // out of range
	if (!fOverwrite && DMA1_hdmas[iStream] && (DMA1_hdmas[iStream] != hdma)) return false; // Don't overwrite existing one...

	// See if someone is losing their stream and there is a callback
	if (DMA1_hdmas[iStream] && (DMA1_hdmas[iStream] != hdma) && DMA1_loseHandlerCBs[iStream])
	{
		//vcp_printf(" DMA1 lose handler CB %d %x\n", iStream, (uint32_t)DMA1_hdmas[iStream]);
		(*DMA1_loseHandlerCBs[iStream])(iStream);
	}
	DMA1_hdmas[iStream] = hdma;
	DMA1_loseHandlerCBs[iStream] = loseCB;
	//vcp_printf("  DMA1 set\n");
	return true;
}

//=============================================================================
// Set the handle for one of the DMA1 streams 
// In some cases we allow the handle to overwrite existing set ones
//=============================================================================
bool SetDMA2StreamHandlerHandle(uint8_t iStream, DMA_HandleTypeDef *hdma, 
	bool fOverwrite,DMALoseStreamHandlerCallbackFunction_t loseCB)
{
	//vcp_printf("SetDMA2Stream: %d %x %x %x\n", iStream, (uint32_t)hdma, fOverwrite, (uint32_t)loseCB);

	if (iStream >= (sizeof(DMA2_hdmas)/sizeof(DMA2_hdmas[0]))) return false; // out of range
	if (!fOverwrite && DMA2_hdmas[iStream] && (DMA2_hdmas[iStream] != hdma)) return false; // Don't overwrite existing one...

	// See if someone is losing their stream and there is a callback
	if (DMA2_hdmas[iStream] && (DMA2_hdmas[iStream] != hdma) && DMA2_loseHandlerCBs[iStream])
	{
		//vcp_printf(" DMA2 lose handler CB %d %x\n", iStream, (uint32_t)DMA2_hdmas[iStream]);
		(*DMA2_loseHandlerCBs[iStream])(iStream);
	}
	DMA2_hdmas[iStream] = hdma;
	DMA2_loseHandlerCBs[iStream] = loseCB;
	//vcp_printf("  DMA2 set\n");
	return true;
}


//=============================================================================
// Simply call off to the HAL_DMA handler with appropriate handle
//=============================================================================
void DMA1_Stream0_IRQHandler(void)
{
	if (DMA1_hdmas[0]) 
	{
  		HAL_DMA_IRQHandler(DMA1_hdmas[0]);
	}
}

void DMA1_Stream1_IRQHandler(void)
{
	if (DMA1_hdmas[1]) 
	{
  		HAL_DMA_IRQHandler(DMA1_hdmas[1]);
	}
}

void DMA1_Stream2_IRQHandler(void)
{
	if (DMA1_hdmas[2]) 
	{
  		HAL_DMA_IRQHandler(DMA1_hdmas[2]);
	}
}

void DMA1_Stream3_IRQHandler(void)
{
	if (DMA1_hdmas[3]) 
	{
  		HAL_DMA_IRQHandler(DMA1_hdmas[3]);
	}
}

void DMA1_Stream4_IRQHandler(void)
{
	if (DMA1_hdmas[4]) 
	{
  		HAL_DMA_IRQHandler(DMA1_hdmas[4]);
	}
}
void DMA1_Stream5_IRQHandler(void)
{
	if (DMA1_hdmas[5]) 
	{
  		HAL_DMA_IRQHandler(DMA1_hdmas[5]);
	}
}

void DMA1_Stream6_IRQHandler(void)
{
	if (DMA1_hdmas[6]) 
	{
  		HAL_DMA_IRQHandler(DMA1_hdmas[6]);
	}
}

void DMA1_Stream7_IRQHandler(void)
{
	if (DMA1_hdmas[7]) 
	{
  		HAL_DMA_IRQHandler(DMA1_hdmas[7]);
	}
}


//=============================================================================
// Simply call off to the HAL_DMA handler with appropriate handle
//=============================================================================
void DMA2_Stream0_IRQHandler(void)
{
	if (DMA2_hdmas[0]) 
	{
  		HAL_DMA_IRQHandler(DMA2_hdmas[0]);
	}
}

void DMA2_Stream1_IRQHandler(void)
{
	if (DMA2_hdmas[1]) 
	{
  		HAL_DMA_IRQHandler(DMA2_hdmas[1]);
	}
}

void DMA2_Stream2_IRQHandler(void)
{
	if (DMA2_hdmas[2]) 
	{
  		HAL_DMA_IRQHandler(DMA2_hdmas[2]);
	}
}

void DMA2_Stream3_IRQHandler(void)
{
	if (DMA2_hdmas[3]) 
	{
  		HAL_DMA_IRQHandler(DMA2_hdmas[3]);
	}
}

void DMA2_Stream4_IRQHandler(void)
{
	if (DMA2_hdmas[4]) 
	{
  		HAL_DMA_IRQHandler(DMA2_hdmas[4]);
	}
}

void DMA2_Stream5_IRQHandler(void)
{
	if (DMA2_hdmas[5]) 
	{
  		HAL_DMA_IRQHandler(DMA2_hdmas[5]);
	}
}

void DMA2_Stream6_IRQHandler(void)
{
	if (DMA2_hdmas[6]) 
	{
  		HAL_DMA_IRQHandler(DMA2_hdmas[6]);
	}
}

void DMA2_Stream7_IRQHandler(void)
{
	if (DMA2_hdmas[7]) 
	{
  		HAL_DMA_IRQHandler(DMA2_hdmas[7]);
	}
}
