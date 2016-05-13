#include "main.h"


void main_init();




int main(void)
{

  main_init();


	while (1)
	{
		HAL_Delay(1000);
	}
}


void main_init()
{
  bsp_init();
}
