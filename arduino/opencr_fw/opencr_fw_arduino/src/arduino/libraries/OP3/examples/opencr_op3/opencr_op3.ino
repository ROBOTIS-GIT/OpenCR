/*
 *  opencr_op3
 *
 *
 *  Created on: 2016. 10. 21.
 *      Author: Baram
 */
#include "dxl_node_op3.h"



extern void dxl_hw_tx_enable(void);

void setup()
{
  Serial.begin(115200);
  Serial.println("op3 start");
  dxl_node_op3_init();
}

void loop()
{
  dxl_node_op3_loop();
}
