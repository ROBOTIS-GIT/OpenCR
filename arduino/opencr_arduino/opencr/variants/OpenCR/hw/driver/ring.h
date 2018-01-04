/*
 * ring.h
 *
 *  Created on: 2017. 8. 21.
 *      Author: HanCheol Cho
 */

#ifndef RING_H_
#define RING_H_



#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"


typedef struct
{
  err_code_t err_code;
  uint32_t   ptr_in;
  uint32_t   ptr_out;
  uint32_t   length;
} ring_node_t;



bool ringInit(void);

err_code_t ringCreate(ring_node_t *p_node, uint32_t length);

err_code_t ringWriteUpdate(ring_node_t *p_node);
uint32_t   ringWriteAvailable(ring_node_t *p_node);
uint32_t   ringGetWriteIndex(ring_node_t *p_node);

err_code_t ringReadUpdate(ring_node_t *p_node);
uint32_t   ringReadAvailable(ring_node_t *p_node);
uint32_t   ringGetReadIndex(ring_node_t *p_node);
uint32_t   ringGetReadOffsetIndex(ring_node_t *p_node, uint32_t offset);

err_code_t ringFlush(ring_node_t *p_node);

#ifdef __cplusplus
}
#endif



#endif /* RING_H_ */
