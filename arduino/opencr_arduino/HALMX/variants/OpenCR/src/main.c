/*
 *  main.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#include <stdio.h>
#include "hal.h"



void setup(void);
void loop(void);



int main(void)
{

	bsp_init();
	hal_init();
	
	setup();

	while (1)
	{
		loop();
	}
}
