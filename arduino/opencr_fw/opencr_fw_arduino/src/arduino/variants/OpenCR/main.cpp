/*
 *  main.h
 *
 *  Created on: 2016. 7. 17.
 *      Author: Baram, PBHP
 */

#include <stdio.h>

#include "bsp.h"
#include "hw.h"
#include "variant.h"


void setup(void);
void loop(void);



int main(void)
{

	bsp_init();
	hw_init();
  var_init();

	setup();

	while (1)
	{
		loop();
		if(serialEventRun) serialEventRun();
	}
}
