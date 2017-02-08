/*
 *  main.h
 *
 *  Created on: 2016. 7. 17.
 *      Author: Baram, PBHP
 */

#include <stdio.h>
#include "hal.h"
#include "drv.h"
#include "variant.h"


void setup(void);
void loop(void);



int main(void)
{

	bsp_init();
	hal_init();
	drv_init();
  var_init();
  
	setup();

	while (1)
	{
		loop();
		if(serialEventRun) serialEventRun();
	}
}
