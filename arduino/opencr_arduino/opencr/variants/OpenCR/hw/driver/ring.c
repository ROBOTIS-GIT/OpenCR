/*
 * ring.c
 *
 *  Created on: 2017. 8. 21.
 *      Author: HanCheol Cho
 */
#include <stdlib.h>

#include "def.h"
#include "def_err.h"
#include "ring.h"




//-- Internal Variables
//


//-- External Variables
//


//-- Internal Functions
//


//-- External Functions
//





bool ringInit(void)
{

  return true;
}

err_code_t ringCreate(ring_node_t *p_node, uint32_t length)
{
  err_code_t err_code = ERR_NONE;


  p_node->ptr_in  = 0;
  p_node->ptr_out = 0;
  p_node->length  = length;

  return err_code;
}

uint32_t ringReadAvailable(ring_node_t *p_node)
{
  uint32_t length;


  length = (p_node->length + p_node->ptr_in - p_node->ptr_out) % p_node->length;

  return length;
}

uint32_t ringWriteAvailable(ring_node_t *p_node)
{
  uint32_t length;
  uint32_t read_length;

  read_length = ringReadAvailable(p_node);

  length = p_node->length - read_length - 1;

  return length;
}

uint32_t ringGetWriteIndex(ring_node_t *p_node)
{
  return p_node->ptr_in;
}

err_code_t ringWriteUpdate(ring_node_t *p_node)
{
  err_code_t err_code = ERR_NONE;
  uint32_t next_index;


  next_index = p_node->ptr_in + 1;

  if (next_index == p_node->length)
  {
    next_index = 0;
  }

  if (next_index != p_node->ptr_out)
  {
    p_node->ptr_in = next_index;
  }
  else
  {
    //err_code = ERR_FULL;
    ringReadUpdate(p_node);
    p_node->ptr_in = next_index;
  }

  return err_code;
}

err_code_t ringReadUpdate(ring_node_t *p_node)
{
  err_code_t err_code = ERR_NONE;
  uint32_t index;
  uint32_t next_index;


  index      = p_node->ptr_out;
  next_index = p_node->ptr_out + 1;

  if (next_index == p_node->length)
  {
    next_index = 0;
  }

  if (index != p_node->ptr_in)
  {
    p_node->ptr_out = next_index;
  }
  else
  {
    err_code = ERR_EMPTY;
  }

  return err_code;
}

uint32_t ringGetReadIndex(ring_node_t *p_node)
{
  return p_node->ptr_out;
}

uint32_t ringGetReadOffsetIndex(ring_node_t *p_node, uint32_t offset)
{
  uint32_t index;


  index = (p_node->length + p_node->ptr_out + offset) % p_node->length;

  return index;
}

err_code_t ringFlush(ring_node_t *p_node)
{
  err_code_t err_code = ERR_NONE;

  p_node->ptr_in  = 0;
  p_node->ptr_out = 0;

  return err_code;
}


